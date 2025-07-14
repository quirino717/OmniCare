import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from omnicare_msgs.action import EnterElevator  # Auto-gerado após build
import time
import math

class EnterElevatorServer(Node):

    def __init__(self):
        super().__init__('enter_elevator_server')


        self.range_left, self.range_right, self.range_front = float('inf'), float('inf'), float('inf')

        # Subs
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )


        # Pub
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Action
        self._action_server = ActionServer(
            self,
            EnterElevator,
            'omnicare/elevator/enter_elevator',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def is_aligned(self,range_left, range_right, tolerance=0.15):
        """
        Verifica se o robô está alinhado com o elevador comparando simetria dos lados.
        """
        if range_left == float('inf') or range_right == float('inf'):
            return False  # Um dos lados está sem leitura, não confiável

        diff = abs(range_left - range_right)
        return diff <= tolerance

    def get_scan_index(self,angle_rad, scan_msg):
        """
        Converte um ângulo (em rad) no índice do ranges[].
        """
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        index = max(0, min(index, len(scan_msg.ranges) - 1))  # clamp
        return index

    def laser_callback(self, msg):
        angle_left = math.radians(45)    # +45°
        angle_right = math.radians(-45)  # -45°

        idx_left = self.get_scan_index(angle_left, msg)
        idx_right = self.get_scan_index(angle_right, msg)
        idx_front = self.get_scan_index(0.0, msg)

        self.range_left = msg.ranges[idx_left]
        self.range_right = msg.ranges[idx_right]
        self.range_front = msg.ranges[idx_front]

        # self.get_logger().info(
        #     f"Frente: {self.range_front:.2f} m | Esq (+45°): {self.range_left:.2f} m | Dir (-45°): {self.range_right:.2f} m"
        # )

    def goal_callback(self, goal_request):
        self.get_logger().info('Recebido pedido de entrar no elevador')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelamento recebido')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executando ação de entrar no elevador...')

        feedback_msg = EnterElevator.Feedback()
        result = EnterElevator.Result()

        entering_elevator = False
        while rclpy.ok():
            rclpy.spin_once(self)
            # self.get_logger().info(
            #     f"Frente: {self.range_front:.2f} m | Esq (+45°): {self.range_left:.2f} m | Dir (-45°): {self.range_right:.2f} m"
            # )


            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Ação cancelada')
                return EnterElevator.Result(success=False, message='Cancelado')
            
            twist_msg = Twist()
            if self.is_aligned(self.range_left, self.range_right) or entering_elevator:
                self.get_logger().info('Robô alinhado com o elevador, entrando...')
                feedback_msg.robot_feedback = 'Robô já está alinhado com o elevador.'
                goal_handle.publish_feedback(feedback_msg)

                entering_elevator = True
                
                # Robô já está alinhado, avança para entrar no elevador
                if self.range_front > 1.88:
                    twist_msg.linear.x = 0.2  
                    self.cmd_vel_publisher.publish(twist_msg)
                
                # Entrou no elevador
                else:
                    twist_msg.linear.x = 0.0
                    self.cmd_vel_publisher.publish(twist_msg)
                    feedback_msg.robot_feedback = 'Entrou no elevador!' 
                    
                    goal_handle.succeed()
                    result.success = True
                    result.message = 'Elevador entrado com sucesso!'
                    return result
    

            elif not entering_elevator and not self.is_aligned(self.range_left, self.range_right):
                self.get_logger().info('Robô não está alinhado com o elevador, ajustando...')
                # Ajuste de alinhamento
                if self.range_left < self.range_right:
                    twist_msg.angular.z = -0.5
                else:
                    twist_msg.angular.z = 0.5
                    
                # self.cmd_vel_publisher.publish(twist_msg)
                feedback_msg.robot_feedback = 'Ajustando alinhamento...'

            self.cmd_vel_publisher.publish(twist_msg)
            goal_handle.publish_feedback(feedback_msg)

            # await rate.sleep()

        


def main(args=None):
    rclpy.init(args=args)
    node = EnterElevatorServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
