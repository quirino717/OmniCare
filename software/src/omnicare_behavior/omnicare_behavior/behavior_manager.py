import re
import rclpy
from rclpy.node import Node


from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, ActionClient

from omnicare_msgs.msg import FloorDetectorInfo
from omnicare_msgs.srv import SwitchFloor, Checkpoints, TeleportFloor, OnOffNode
from omnicare_msgs.action import EnterElevator, RunMission


# from omnicare_behavior.action import EnterElevator
from omnicare_behavior.states.floor_navigation import switch_floor, start_checkpoints, teleport_robot
from omnicare_behavior.states.align_elevator import activate_display_inference, deactivate_display_inference, align_the_robot


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
            'ERROR': self._error
        }

        # Compila a regex para extrair números do final da string do andar
        self.regex = re.compile(r'(-?\d+)$')


        # Pubs
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel', 10)


        # Sub to debug exit elevator
        self.direction = None
        self.debug_sub = self.create_subscription(Bool, '/right_floor', self._debug_sub_cb, 10)
        self.checkpoints_sub = self.create_subscription(Bool,'/checkpoint_done',self._checkpoints_callback,10)
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
        self.ret = False

        # Initialize goal, feedback, and result
        self.goal_ = goal_handle.request
        self.result_ = RunMission.Result()
        self.feedback_msg_ = RunMission.Feedback()
        
        # Reset state machine
        self.current_state = 'FLOOR_NAVIGATION'
        self.start_navigation = True

        # Get parameters from the goal
        self.simulation = self.goal_.simulation
        self.actual_floor = self.goal_.initial
        self.target_floor = self.goal_.target

        rate = self.create_rate(10) #10 Hz
        while rclpy.ok():
            self._check_states()
            goal_handle.publish_feedback(self.feedback_msg_)

            if self.current_state in ['DONE', 'ERROR']: 
                break
            rate.sleep()  # Sleep for a while to avoid 


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


    def _checkpoints_callback(self, msg):
        self.get_logger().info(f"Received checkpoint done signal: {msg.data}")
        if msg.data:
            self.get_logger().info("Checkpoint atingido!")
            if self.start_navigation: self.current_state = 'WAIT_FOR_FLOOR'
            else: self.current_state = 'DONE'

    def _floor_navigation(self):
        self.feedback_msg_.robot_feedback = "Navigating"
        if self.start_navigation:
            self.get_logger().info("Iniciando navegação no andar atual...")

            switch_floor(
                floor=self.actual_floor,  # Which floor to start navigation 
                node=self, # Pass the current node instance
                switch_floor_client=self.switch_floor, # Pass the SwitchFloor client instance
                simulation=self.simulation # Pass the simulation flag
            )
                        
            start_checkpoints(
                floor="simulation_quintoAndar",  # Which floor to start checkpoints
                node=self, # Pass the current node instance
                start_checkpoint_client=self.start_checkpoint, # Pass the Checkpoints client instance
            )
        else:
            self.get_logger().info("Iniciando navegação no andar alvo...")
            switch_floor(
                floor=self.target_floor,  
                node=self, 
                switch_floor_client=self.switch_floor, 
                simulation=self.simulation 
            )
                        
            start_checkpoints(
                floor="simulation_quartoAndar",  # Which floor to start checkpoints
                node=self, # Pass the current node instance
                start_checkpoint_client=self.start_checkpoint, # Pass the Checkpoints client instance
            )


        self.current_state = 'WAITING_CHECKPOINT_GOAL'
    
    #  ------------------------------  END -------------------------------

    #  ------------------------------  Waiting Checkpoint State -------------------------------

    def _waiting_checkpoints(self):
        # self.get_logger().info("Aguardando sinal do checkpoint...")
        pass


    #  ------------------------------  END -------------------------------

    #  --------------------------  Align in Elevator State -----------------------------
    
    def _activate_inference(self):
        self.get_logger().info("Ativando inferência para identificar o display...")

        activate_display_inference(
            node=self,  # Pass the current node instance
            activate_inference_client=self.activate_inference,  # Pass the OnOffNode action client instance,
            simulation=self.simulation # Pass the simulation flag
        )

        self.current_state = 'ALIGN_IN_ELEVATOR'

    def _display_ref_cb(self, msg):
        self.direction = msg.data
        if self.direction == "Align":
            self.get_logger().info("Alinhamento com o display concluído.")
            self.current_state = 'WAIT_FOR_FLOOR'
        else:
            self.get_logger().info(f"Alinhando com o display: {self.direction}")

    def _align_in_elevator(self):
        self.feedback_msg_.robot_feedback = "Aligning"
        # self.get_logger().info("Alinhando com o display do elevador...")

        align_the_robot(
            speed_publisher=self.teleop_pub,  # Pass the cmd_vel publisher
            direction=self.direction  # Direction to align ("Left", "Right", or None)
        )

    #  ------------------------------  END -------------------------------


    #  --------------------------  Wait for floor State -----------------------------

    def _debug_sub_cb(self, msg):
        if msg.data:
            if self.simulation:
                # Teleport the robot to the correct floor in simulation
                teleport_robot( 
                    floor=self.target_floor,  # Which floor to teleport
                    node=self, # Pass the current node instance
                    teleport_robot_client=self.teleport_robot # Pass the TeleportFloor client instance
                )

            # Deactivate the inference after aligning and reaching the floor
            deactivate_display_inference(self, self.activate_inference, self.simulation)

            self.start_navigation = False # Flag to alert the navigation is already started
            self.get_logger().info("Debug: Chegou no andar correto!")
            self.current_state = 'FLOOR_NAVIGATION'
        else:
            self.get_logger().info("Debug: Ainda não chegou no andar correto.")


    def _display_detection_cb(self, msg):
        if msg.floor_name is None or msg.floor_name == "":
            return
        
        floor_filtred = self.regex.search(msg.floor_name)
        if floor_filtred.group(1) == self.target_floor:
            if self.simulation:
                # Teleport the robot to the correct floor in simulation
                teleport_robot( 
                    floor=self.target_floor,  # Which floor to teleport
                    node=self, # Pass the current node instance
                    teleport_robot_client=self.teleport_robot # Pass the TeleportFloor client instance
                )

            # Deactivate the inference after aligning and reaching the floor
            deactivate_display_inference(self, self.activate_inference, self.simulation)

            self.start_navigation = False # Flag to alert the navigation is already started
            self.get_logger().info("Detecção: Chegou no andar correto!")
            self.current_state = 'FLOOR_NAVIGATION'

        self.get_logger().info(f"Detecção: Andar atual {int(floor_filtred.group(1))}, Alvo {self.target_floor}.")
   
    def _wait_for_floor(self):
        self.feedback_msg_.robot_feedback = "Waiting"
        # Aqui você pode monitorar a mudança de andar via tópico de visão
        self.get_logger().info("Se posicionando e aguardando chegada no 4º andar (via visão)...")
        # self.current_state = 'EXIT_ELEVATOR'


    #  ------------------------------  END -------------------------------

    def _error(self):
        self.get_logger().info("Error! Please check the logs for more details.")


    # def done(self):
    #     self.get_logger().info('FSM finalizada.')

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
