import re
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time


from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rclpy.action import ActionServer, ActionClient

from omnicare_msgs.msg import FloorDetectorInfo
from omnicare_msgs.srv import SwitchFloor, Checkpoints, TeleportFloor, OnOffNode
from omnicare_msgs.action import EnterElevator, RunMission


# from omnicare_behavior.action import EnterElevator
from omnicare_behavior.utils.watchdog import Watchdog
from omnicare_behavior.states.floor_navigation import switch_floor, start_checkpoints, teleport_robot
from omnicare_behavior.states.align_elevator import activate_display_inference, deactivate_display_inference, align_the_robot


# ros2 action send_goal   /omnicare/behavior/run_mission   omnicare_msgs/action/RunMission   '{simulation: False, initial: 5, target: 4}'   --feedback

class ElevatorBehaviorManager(Node):

    def __init__(self):
        super().__init__('elevator_behavior_manager')

        # Define os estados e o mapa de transições
        self.states = {
            'FLOOR_NAVIGATION': self._floor_navigation,
            'WAITING_CHECKPOINT_GOAL': self._waiting_checkpoints,
            'ACTIVATE_INFERENCE': self._activate_inference,
            'ALIGN_IN_ELEVATOR': self._align_in_elevator,
            'WAIT_FOR_FLOOR': self._wait_for_floor,
            'DONE': self._done,
            'ERROR': self._error
        }


        # Compila a regex para extrair números do final da string do andar
        self.regex = re.compile(r'andar_(\w+)$')

        # Variables
        self.direction,self.last_amcl_time, self.start_time, self.actual_time = None, None, None, None
        self.current_state, self.linear, self.angular, self.l_x, self.l_y, self.a_z = None, None, None, 0.0, 0.0, 0.0
        self.detect_counter = 0
        

        # Pubs
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subs
        self.checkpoints_sub = self.create_subscription(Bool,'/checkpoint_done',self._checkpoints_callback,10)
        self.cmd_vel_sub = self.create_subscription(Twist,'/cmd_vel',self._cmd_vel_sub_cb,10)
        self.display_ref_sub = self.create_subscription(String, '/omnicare/floor_detector/align_to_display', self._display_ref_cb, 10)
        self.display_detection_sub = self.create_subscription(FloorDetectorInfo, '/omnicare/floor_detector/info', self._display_detection_cb, 10)


        # Service Clients
        self.start_checkpoint = self.create_client(Checkpoints, '/omnicare/checkpoints/start')
        self.activate_inference = self.create_client(OnOffNode, '/omnicare/floor_detector/OnOffNode')
        self.switch_floor = self.create_client(SwitchFloor, '/omnicare/navigation/switch_floor')
        self.teleport_robot = self.create_client(TeleportFloor, '/omnicare/simulation/teleport_floor')
        
        # Action Clients                                                
        self.enter_elevator_client = ActionClient(self, EnterElevator, '/omnicare/elevator/enter_elevator')
        self.exit_elevator_client  = ActionClient(self, EnterElevator, '/omnicare/elevator/exit_elevator')

        # Action Server
        self.goal_,self.feedback_msg_, self.result_ = None, None, None
        self.simulation, self.actual_floor, self.target_floor = None, None, None
        self.run_srv = ActionServer(self, RunMission, '/omnicare/behavior/run_mission',
                                    execute_callback=self._state_machine_loop,
                                    cancel_callback=self._cancel_cb)
        
        self.get_logger().info("Elevator Behavior Manager Node has been started.")
        self.get_logger().info("Waiting for a mission...")


    #  ------------------------------  ACTION SERVER ------------------------------
    async def _state_machine_loop(self, goal_handle):
        self.get_logger().info('RunMission action started.')

        # Instanciando o Watchdog
        self.watchdog = Watchdog(
            node=self,
            timeout_sec=10.0,
            check_period_sec=0.2,
            on_timeout=self._on_watchdog_timeout
        )

        # Initialize goal, feedback, and result
        self.goal_ = goal_handle.request
        self.result_ = RunMission.Result()
        self.feedback_msg_ = RunMission.Feedback()
        
        # Reset state machine
        self.current_state = 'FLOOR_NAVIGATION'
        self.start_navigation = True

        # Get parameters from the goal
        self.simulation = self.goal_.simulation
        self.world = self.goal_.world
        self.actual_floor = self.goal_.initial
        self.target_floor = self.goal_.target

        self.get_logger().info(f"Mission received: World: {self.world} | from {self.actual_floor} to {self.target_floor} | Simulation: {self.simulation}")
        rate = self.create_rate(10) #10 Hz
        while rclpy.ok():
            self._check_states()
            goal_handle.publish_feedback(self.feedback_msg_)

            if self.current_state in ['DONE', 'ERROR']: 
                break
            rate.sleep()  # Sleep for a while to avoid 

        # Cleanup o watchdog
        self.watchdog.cancel()

        goal_handle.succeed()
        self.result_.message = "Mission completed successfully."
        return self.result_
        
    def _cancel_cb(self, goal_handle):
        self.get_logger().info('RunMission action canceled.')
        self.current_state = 'ERROR'  # Transition to an error state or handle cleanup
        goal_handle.canceled()
        result = RunMission.Result()
        result.message = "Mission was canceled."
        return result
    
    #  ------------------------------  FINITE STATE MACHINE ------------------------------

    def _check_states(self):
        if self.current_state in self.states:
            self.states[self.current_state]()
        else:
            self.get_logger().error(f"Estado desconhecido: {self.current_state}")
            self.current_state = 'DONE'

    
    #  ------------------------------  Floor Navigation State -------------------------------

    def _floor_navigation(self):
        self.feedback_msg_.robot_feedback = "Navigating"
        if self.start_navigation:
            self.get_logger().info("Iniciando navegação no andar atual...")

            switch_floor(
                floor=self.actual_floor,  # Which floor to start navigation 
                node=self, # Pass the current node instance
                switch_floor_client=self.switch_floor, # Pass the SwitchFloor client instance
                world=self.world, # Pass the world parameter
                simulation=self.simulation # Pass the simulation flag
            )
                        
            start_checkpoints(
                floor=self.actual_floor,  # Which floor to start checkpoints
                node=self, # Pass the current node instance
                start_checkpoint_client=self.start_checkpoint, # Pass the Checkpoints client instance
                world=self.world, # Pass the world parameter
                simulation=self.simulation # Pass the simulation flag
            )
        else:
            self.get_logger().info("Iniciando navegação no andar alvo...")
            switch_floor(
                floor=self.target_floor,  
                node=self, 
                switch_floor_client=self.switch_floor, 
                world=self.world,
                simulation=self.simulation 
            )
                        
            start_checkpoints(
                floor=self.target_floor,  # Which floor to start checkpoints
                node=self, # Pass the current node instance
                start_checkpoint_client=self.start_checkpoint, # Pass the Checkpoints client instance
                world=self.world, # Pass the world parameter 
                simulation=self.simulation # Pass the simulation flag
            )

        # Watchdog keep-alive
        self.watchdog.keep_alive()

        self.current_state = 'WAITING_CHECKPOINT_GOAL'
    
    #  ------------------------------  END -------------------------------

    #  ------------------------------  Waiting Checkpoint State -------------------------------

    def _checkpoints_callback(self, msg):
        # Safety verifications to not crash the node
        if msg.data is None:
            return
        
        # Only process checkpoint done if we are in the WAITING_CHECKPOINT_GOAL state
        if self.current_state != 'WAITING_CHECKPOINT_GOAL':
            return
        
        # If a checkpoint is done, transition to the next state
        self.get_logger().info(f"Received checkpoint done signal: {msg.data}")
        if msg.data:
            self.get_logger().info("Checkpoint atingido!")
            if self.start_navigation: self.current_state = 'ACTIVATE_INFERENCE'
            else: self.current_state = 'DONE'
    
    def _cmd_vel_sub_cb(self, msg):
        self.linear = msg.linear
        self.angular = msg.angular

        self.l_x, self.l_y = self.linear.x, self.linear.y
        self.a_z = self.angular.z

    def _is_robot_stopped(self):
        return self.l_x == 0.0 and self.l_y == 0.0 and self.a_z == 0.0

    def _waiting_checkpoints(self):
        self.get_logger().info("Aguardando sinal do checkpoint...")
        self.feedback_msg_.robot_feedback = "Waiting-checkpoints"

        if self._is_robot_stopped():
            self.get_logger().info("Robot stopped")
            pass
        else:
            self.get_logger().info("Watchdog keep alive")
            # Watchdog keep-alive
            self.watchdog.keep_alive()

        pass

    #  ------------------------------  END -------------------------------

    #  --------------------------  Activate Inference State -----------------------------
    
    def _activate_inference(self):
        self.get_logger().info("Ativando inferência para identificar o display...")
        self.feedback_msg_.robot_feedback = "Activating-inference"

        activate_display_inference(
            node=self,  # Pass the current node instance
            activate_inference_client=self.activate_inference,  # Pass the OnOffNode action client instance,
            simulation=self.simulation # Pass the simulation flag
        )

        # Watchdog keep-alive
        self.watchdog.keep_alive()

        self.current_state = 'ALIGN_IN_ELEVATOR'



    #  --------------------------  Align in Elevator State -----------------------------

    def _display_ref_cb(self, msg):
        self.direction = msg.data

        # Safety verifications to not crash the node
        if self.direction is None:
            return

        # Only process alignment if we are in the ALIGN_IN_ELEVATOR state
        if self.current_state != 'ALIGN_IN_ELEVATOR':
            return
        
        # If the direction is "Align", it means the robot is aligned with the display
        if self.direction == "Align":
            self.get_logger().info("Alinhamento com o display concluído.")
            self.current_state = 'WAIT_FOR_FLOOR'
            
        self.get_logger().info(f"Alinhando com o display: {self.direction}")

    def _align_in_elevator(self):        
        self.feedback_msg_.robot_feedback = "Aligning-in-elevator"

        align_the_robot(
            speed_publisher=self.teleop_pub,  # Pass the cmd_vel publisher
            direction=self.direction  # Direction to align ("Left", "Right", or None)
        )

        if self.simulation:
            # Teleport the robot to the correct floor in simulation
            teleport_robot( 
                world=self.world, # Pass the world parameter
                floor=self.target_floor,  # Which floor to teleport
                node=self, # Pass the current node instance
                teleport_robot_client=self.teleport_robot # Pass the TeleportFloor client instance
            )
            self.start_navigation = False # Flag to alert the navigation is already started
            self.current_state = 'FLOOR_NAVIGATION'

        # Watchdog keep-alive
        self.watchdog.keep_alive()


    #  ------------------------------  END -------------------------------


    #  --------------------------  Wait for floor State -----------------------------

    def _display_detection_cb(self, msg):
        # self.get_logger().info(f"msg: {msg.floor_name}" )

        # Safety verifications to not crash the node
        if msg.floor_name is None or msg.floor_name == '':
            return

        # Only process floor detection if we are in the WAIT_FOR_FLOOR state
        if self.current_state != 'WAIT_FOR_FLOOR':
            return
        
        # Extract the floor number using regex (possible this regex can recuse floors like andar_t)
        floor_filtred = self.regex.search(msg.floor_name)

        # More verifications....
        if floor_filtred is None:
            return


        # If is on the same floor (or going to a walk in the elevator), do a keep-alive on the watchdog  
        valid_floors = ['t','1', '2', '3', '4', '5']
        if floor_filtred.group(1) in valid_floors:
            self.get_logger().info("Im in valid floor")
            # Watchdog keep-alive
            self.watchdog.keep_alive()

            # If the detected floor matches the target floor, increment the counter
            if floor_filtred.group(1) == self.target_floor:
                self.detect_counter += 1
            else:
                self.detect_counter = 0

        # If the detected floor matches the target floor consistently, consider it confirmed
        if self.detect_counter >= 8 and self.start_navigation:
            self.get_logger().info("Detecção: Chegou no andar correto!")
            self.detect_counter = 0  # Reset the counter

            if self.simulation:
                # Teleport the robot to the correct floor in simulation
                teleport_robot( 
                    world=self.world, # Pass the world parameter
                    floor=self.target_floor,  # Which floor to teleport
                    node=self, # Pass the current node instance
                    teleport_robot_client=self.teleport_robot # Pass the TeleportFloor client instance
                )

            # Deactivate the inference after aligning and reaching the floor
            deactivate_display_inference(self, self.activate_inference, self.simulation)

            self.start_navigation = False # Flag to alert the navigation is already started
            self.current_state = 'FLOOR_NAVIGATION'

            # Watchdog keep-alive
            self.watchdog.keep_alive()
            

        # If the robot for any reason entered in the condition above but does not switch the map and start
        # the navigation on the target floor, the watchdog will timeout and in the timeout cb it will threat this situation
        if not self.start_navigation:
            # Watchdog keep-alive
            self.watchdog.keep_alive()

        self.get_logger().info(f"Detecção: Andar atual {floor_filtred.group(1)}, Alvo {self.target_floor}, Floor_navigation {self.start_navigation}, Count {self.detect_counter}.")


   
    def _wait_for_floor(self):
        if self.feedback_msg_.robot_feedback != "Waiting-floor":
            # Aqui você pode monitorar a mudança de andar via tópico de visão
            self.get_logger().info("Se posicionando e aguardando chegada no 4º andar (via visão)...")

        self.feedback_msg_.robot_feedback = "Waiting-floor"
        
        if (self.simulation):
            # Watchdog keep-alive
            self.watchdog.keep_alive()



    #  ------------------------------  END -------------------------------

    #  --------------------------  Done State -------------------------------

    def _done(self):
        self.feedback_msg_.robot_feedback = "Done"
        self.get_logger().info('Mission finished.')

    # ------------------------------  END -------------------------------

    #  --------------------------  Error State -------------------------------

    def _error(self):
        self.feedback_msg_.robot_feedback = "Error"
        self.get_logger().error("Mission encountered an error.")
        self.get_logger().error("Please check the logs for more details.")

    # ------------------------------  END -------------------------------

    #  --------------------  Watchdog Timeout Handler -------------------------------

    def _on_watchdog_timeout(self):
        self.get_logger().error("Watchdog timeout! No state activity detected.")
        self.get_logger().info(f"Current state during timeout: {self.current_state}")
        if self.current_state == "WAITING_CHECKPOINT_GOAL":
            # Reinicia o watchdog
            self.watchdog.keep_alive()
            # Reinicia a navegação
            self.current_state = 'FLOOR_NAVIGATION'
            self.get_logger().info("Reiniciando navegação devido ao timeout no estado de espera de checkpoints.")

        elif self.current_state == "WAIT_FOR_FLOOR":
            # Reinicia o watchdog
            self.watchdog.keep_alive()
            # Setando a variavel que possivelmente esta travando o sistema de entrar na condição de navegação
            self.start_navigation = True
            self.get_logger().info("Timeout no estado de espera de andar.")


        else:
            self.current_state = 'ERROR'

    # ------------------------------  END -------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorBehaviorManager()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
