#!/usr/bin/env python3
"""
OmniCare HRI node
-----------------
Purpose:
  - Decide the LED state of the robot (Blue=idle, Yellow=moving, Red=error)
  - Send that state to a microcontroller (e.g., Arduino) over serial
  - Use ROS 2 signals (cmd_vel, Nav2 action statuses, optional mission feedback/alarm)

Design choices:
  - Colors:
      Blue  (B): Idle / robot is operational but not executing a mission
      Yellow(Y): Moving / normal operation (also covers short "micro-stops" while adjusting)
      Red   (R): Error only (action aborted/canceled, alarm, or mission feedback indicating failure)
  - For omni robots, we detect motion in x/y (linear) and z (angular).
  - Short stops (tiny re-alignments) remain Yellow via a "moving hold" window.

Parameters (override via --ros-args):
  - port (str): serial port (default '/dev/ttyACM0')
  - baud (int): serial baudrate (default 115200)
  - lin_eps (float): linear XY speed threshold [m/s] (default 0.05)
  - ang_eps (float): angular Z speed threshold [rad/s] (default 0.08)
  - moving_hold_s (float): keep Yellow for this time after last motion (default 20.0)
  - idle_confirm_s (float): time without motion to show Blue (default 10.0)
  - reinforce_s (float): periodically re-send current state to serial (default 1.0)
  - status_topics (string[] or CSV): action status topics to watch (Nav2 + elevator by default)
  - runmission_feedback_topic (str): optional mission feedback topic; empty disables
  - alarm_topic (str): optional Bool alarm topic; empty disables
"""

import math
import time
import re
import serial
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import Bool

try:
    from omnicare_msgs.action._run_mission import RunMission_FeedbackMessage as RunMissionFeedbackMsg
    HAS_RUNMISSION = True
except Exception as e:
    HAS_RUNMISSION = False
    print(f'Não consegui importar RunMission_FeedbackMessage: {e}')


# ---------------- Defaults ----------------
DEFAULT_PORT            = '/dev/ttyUSB1'
DEFAULT_BAUD            = 115200
DEFAULT_LIN_EPS         = 0.05   # m/s
DEFAULT_ANG_EPS         = 0.08   # rad/s
DEFAULT_MOVING_HOLD_S   = 20.0   # keep Yellow after last motion
DEFAULT_IDLE_CONFIRM_S  = 10.0   # require this time truly idle to show Blue
DEFAULT_REINFORCE_S     = 1.0

DEFAULT_STATUS_TOPICS = [
    '/omnicare/elevator/enter_elevator/_action/status'
]

DEFAULT_RUNMISSION_FB_TOPIC = '/omnicare/behavior/run_mission/_action/feedback'
DEFAULT_ALARM_TOPIC         = '/omnicare/hri/alarm'  # e.g. '/omnicare/navigation/alarm'
DEFAULT_ANTHEM_TOPIC        = '/omnicare/hri/anthem'   # e.g. '/omnicare/hri/anthem'
# ------------------------------------------


class AlertNode(Node):
    """
    HRI state machine:
      - 'B' (Blue): idle     -> idle for >= idle_confirm_s (no need for motion)
      - 'Y' (Yellow): moving -> motion in {vx, vy, wz} OR within moving_hold_s from last motion
      - 'R' (Red): error     -> only when an actual error is detected
    """
    def __init__(self):
        super().__init__('omnicare_expression')

        # Parameters
        self.port   = self.declare_parameter('port',   DEFAULT_PORT).value
        self.baud   = self.declare_parameter('baud',   DEFAULT_BAUD).value
        self.lin_eps = float(self.declare_parameter('lin_eps', DEFAULT_LIN_EPS).value)
        self.ang_eps = float(self.declare_parameter('ang_eps', DEFAULT_ANG_EPS).value)
        self.moving_hold_s  = float(self.declare_parameter('moving_hold_s', DEFAULT_MOVING_HOLD_S).value)
        self.idle_confirm_s = float(self.declare_parameter('idle_confirm_s', DEFAULT_IDLE_CONFIRM_S).value)
        self.reinforce_s    = float(self.declare_parameter('reinforce_s', DEFAULT_REINFORCE_S).value)

        param_status = self.declare_parameter('status_topics', DEFAULT_STATUS_TOPICS).value
        if isinstance(param_status, str):
            self.status_topics = [s.strip() for s in param_status.split(',') if s.strip()]
        else:
            self.status_topics = list(param_status) if param_status else []

        self.alarm_topic = self.declare_parameter('alarm_topic', DEFAULT_ALARM_TOPIC).value
        self.runmission_feedback_topic = self.declare_parameter('runmission_feedback_topic', DEFAULT_RUNMISSION_FB_TOPIC).value
        self.anthem_topic = self.declare_parameter('anthem_topic', DEFAULT_ANTHEM_TOPIC).value

        # Serial I/O to the microcontroller
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(1.5)  # give the MCU time to reboot after opening the port
        self.get_logger().info(f'Connected to {self.port} @ {self.baud}')

        # Internal state
        self.current     = 'B'   # start as Blue (idle/ready)
        self.last_sent   = None
        self.goal_active = False
        self.error_flag  = False
        self.last_motion = self.get_clock().now()  # last time motion was detected

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 20)

        for topic in self.status_topics:
            self.create_subscription(GoalStatusArray, topic, self.cb_status, 10)
            self.get_logger().info(f'Subscribed action status: {topic}')

        if HAS_RUNMISSION:
            self.create_subscription(
                RunMissionFeedbackMsg,
                '/omnicare/behavior/run_mission/_action/feedback',
                self.cb_runmission_fb,
                10
            )

        if self.alarm_topic:
            self.create_subscription(Bool, self.alarm_topic, self.cb_alarm, 10)
            self.get_logger().info(f'Subscribed alarm: {self.alarm_topic}')

        if self.anthem_topic:
            self.create_subscription(Bool, self.anthem_topic, self.cb_anthem, 10)
            self.get_logger().info(f'Subscribed anthem trigger: {self.anthem_topic}')

        # Timers
        self.create_timer(0.05, self.tick_logic)           # 20 Hz decision loop
        self.create_timer(self.reinforce_s, self.tick_tx)  # periodic state reinforcement
        self.create_timer(0.01, self.tick_rx)  # 100 Hz
    # ----------- Serial RX -----------
    def tick_rx(self):
        """Receive the data from MCU."""
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n > 0:
                data = self.ser.read(n)  # lê tudo disponível de uma vez
                for b in data:
                    self.rx_last = b
                    self._process_rx_byte(b)
        except Exception as e:
            self.get_logger().warning(f"Serial read failed: {e}")

    def _process_rx_byte(self, b: int):
        """Events comming from MCU."""
        self.get_logger().info(f"MCU -> byte recebido: 0x{b:02X}")
        if b == 0x22:
            # Exemplo: ACK/trigger do MCU
            self.get_logger().info("MCU -> 0x22 recebido")
            # Faça algo útil: liberar um wait, limpar erro, etc.
            # self.error_flag = False
    # ----------- Callbacks -----------

    def cb_anthem(self, msg: Bool):
        """
        If data=True, send 'X' (one-shot) to the MCU to play the anthem.
        This does not change the LED state.
        """
        if not msg.data:
            return
        if self.ser is None:
            self.get_logger().warning("Anthem trigger received, but serial is not open.")
            return
        try:
            self.ser.write(b'X')
            self.get_logger().info("ANTHEM: sent 'X' to MCU.")
        except Exception as e:
            self.get_logger().warning(f"Failed to send 'X' to MCU: {e}")

    def cb_cmd(self, msg: Twist):
        """
        Detect motion for omni-directional robot:
          - linear XY magnitude
          - angular Z
        If above thresholds, consider "moving" and push Yellow (Y).
        """
        lin_xy = math.hypot(msg.linear.x, msg.linear.y)
        moving = (lin_xy >= self.lin_eps) or (abs(msg.angular.z) >= self.ang_eps)
        if moving:
            self.last_motion = self.get_clock().now()
            if self.current != 'Y':
                self.current = 'Y'
                self.maybe_send()

    def cb_status(self, msg: GoalStatusArray):
        pass

    def cb_runmission_fb(self, msg):
        """
        Optional: parse mission textual feedback for error keywords.
        If any found, raise error_flag (Red).
        """
        text = (msg.feedback.robot_feedback or '').lower()
        self.get_logger().info(f'RunMission feedback: {msg.feedback.robot_feedback}')
        if re.search(r'error|erro|fail|alarm|alarme|stuck|abort', text):
            self.error_flag = True

    def cb_alarm(self, msg: Bool):
        """Direct alarm channel (Bool). True => Alarm on."""
        if msg is None:
            return
        
        if msg.data == True:
            self.ser.write(b'E')
        else:
            self.ser.write(b'D')


    # ----------- Decision logic -----------
    def tick_logic(self):
        """
        Decide LED:
        - R: only if error_flag is True
        - Y: moving or within moving_hold_s since last motion
        - B: idle for >= idle_confirm_s
        - Otherwise: Y if a goal is active (safer around people), else B
        """
        now = self.get_clock().now()
        idle_time = (now - self.last_motion).nanoseconds / 1e9

        if self.error_flag:
            new = 'R'
        elif idle_time <= self.moving_hold_s:
            new = 'Y'
        elif idle_time >= self.idle_confirm_s:
            new = 'B'
        else:
            new = 'Y' if self.goal_active else 'B'

        if new != self.current:
            self.current = new
            self.maybe_send()

    # ----------- Serial TX -----------
    def maybe_send(self):
        """Send state letter to MCU only on change to avoid flooding."""
        if self.current != self.last_sent:
            try:
                self.ser.write(self.current.encode('ascii'))  # 'B'|'Y'|'R'
                self.last_sent = self.current
                self.get_logger().info(
                    f'LED -> {self.current} (goal_active={self.goal_active}, error={self.error_flag})'
                )
            except Exception as e:
                self.get_logger().warn(f'Serial write failed: {e}')

    def tick_tx(self):
        """Periodic reinforcement; helps if MCU rebooted or missed a byte."""
        try:
            self.ser.write(self.current.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f'Serial write failed: {e}')


def main():
    rclpy.init()
    rclpy.spin(AlertNode())
    rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile
# from action_msgs.msg import GoalStatusArray
# from omnicare_msgs.action import RunMission  # ==> troque pelo seu pacote/Action



# class ActionMonitor(Node):
#     def __init__(self):
#         super().__init__('action_monitor')

#         qos = QoSProfile(depth=10)

#         if HAS_RUNMISSION:
#             self.create_subscription(
#                 RunMissionFeedbackMsg,
#                 '/omnicare/behavior/run_mission/_action/feedback',
#                 self.cb_runmission_fb,
#                 10
#             )
      

#     # def feedback_cb(self, msg):
#     #     # msg.goal_id.uuid é um array[16] (UUID do goal que gerou esse feedback)
#     #     uuid_hex = ''.join(f'{b:02x}' for b in msg.goal_id.uuid)
#     #     fb = msg.feedback                 # campos definidos pela sua Action
#     #     self.get_logger().info(f'[FB] goal={uuid_hex} -> {fb}')

#     def cb_runmission_fb(self, msg):
#             # msg.feedback é do tipo RunMission.Feedback
#             text = (getattr(msg.feedback, 'robot_feedback', '') or '').lower()
#             self.get_logger().info(f'RunMission feedback: {text}')
#             # if re.search(r'error|erro|fail|alarm|alarme|stuck|abort', text):
#             #     self.error_flag = True



# def main():
#     rclpy.init()
#     node = ActionMonitor()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '_main_':
#     main()