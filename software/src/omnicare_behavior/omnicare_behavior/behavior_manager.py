import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rclpy.action import ActionClient

from omnicare_msgs.srv import SwitchFloor, Checkpoints, TeleportFloor
from omnicare_msgs.action import EnterElevator 


# from omnicare_behavior.action import EnterElevator
from omnicare_behavior.states.floor_navigation import switch_floor, start_checkpoints, teleport_robot
from omnicare_behavior.states.enter_elevator import enter_elevator_behavior
from omnicare_behavior.utils.parsing_yaml import extract_map_configuration


class ElevatorBehaviorManager(Node):

    def __init__(self):
        super().__init__('elevator_behavior_manager')

        self.simulation = self.declare_parameter('simulation',True).get_parameter_value().bool_value
        self.actual_floor = self.declare_parameter('actual_floor', 5).get_parameter_value().integer_value
        self.target_floor = self.declare_parameter('target_floor', 4).get_parameter_value().integer_value

        # Define os estados e o mapa de transições
        self.states = {
            'FLOOR_NAVIGATION': self.floor_navigation,
            'WAITING_CHECKPOINT_GOAL': self.waiting_checkpoints,
            'ENTER_ELEVATOR': self.enter_elevator,
            'WAIT_ENTER_ELEVATOR': self.wait_enter_elevator,
            'WAIT_FOR_FLOOR': self.wait_for_floor,
            'EXIT_ELEVATOR': self.exit_elevator,
            'ERROR': self.error
        }


        self.start_navigation = True 
        self.current_state = 'FLOOR_NAVIGATION'

        # Subscribes
        self.checkpoints_sub = self.create_subscription(
            Bool,
            '/checkpoint_done',
            self.checkpoints_callback,
            10
        )

        # Sub to debug exit elevator
        self.debug_sub = self.create_subscription(Bool, '/right_floor', self.debug_sub_cb, 10)       

        # Service Clients
        self.switch_floor = self.create_client(SwitchFloor, '/omnicare/navigation/switch_floor')
        self.teleport_robot = self.create_client(TeleportFloor, '/omnicare/simulation/teleport_floor')
        self.startCheckpoint = self.create_client(Checkpoints, '/omnicare/checkpoints/start')


        # Action Clients                                                
        self.enter_elevator_client = ActionClient(self, EnterElevator, '/omnicare/elevator/enter_elevator')
        self.exit_elevator_client  = ActionClient(self, EnterElevator, '/omnicare/elevator/exit_elevator')


        # self.enter_elevator_client = ActionClient(self, EnterElevator, '/enter_elevator')
        # self.activate_floor_client = self.create_client(Trigger, '/activate_floor_nav')

        self.timer = self.create_timer(1.0, self.state_machine_loop)

    
    #  ------------------------------  FINITE STATE MACHINE ------------------------------

    def set_result(self, success: bool):
        self.ret = success

    def state_machine_loop(self):
        if self.current_state in self.states:
            self.states[self.current_state]()
        else:
            self.get_logger().error(f"Estado desconhecido: {self.current_state}")
            self.current_state = 'DONE'

    
    #  ------------------------------  Floor Navigation State -------------------------------


    def checkpoints_callback(self, msg):
        self.get_logger().info(f"Received checkpoint done signal: {msg.data}")
        if msg.data:
            self.get_logger().info("Checkpoint atingido!")
            if self.start_navigation: self.current_state = 'WAIT_FOR_FLOOR'
            else: self.current_state = 'DONE'

    def floor_navigation(self):

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
                start_checkpoint_client=self.startCheckpoint, # Pass the Checkpoints client instance
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
                start_checkpoint_client=self.startCheckpoint, # Pass the Checkpoints client instance
            )


        self.current_state = 'WAITING_CHECKPOINT_GOAL'
    
    #  ------------------------------  END -------------------------------

    #  ------------------------------  Waiting Checkpoint State -------------------------------

    def waiting_checkpoints(self):
        # self.get_logger().info("Aguardando sinal do checkpoint...")
        pass


    #  ------------------------------  END -------------------------------


    #  --------------------------  Enter in Elevator State -----------------------------

    def enter_elevator(self):
        self.get_logger().info('Chamando action para entrar no elevador...')

        enter_elevator_behavior(
            node=self,  # Pass the current node instance
            action_client=self.enter_elevator_client  # Pass the EnterElevator action client instance,
        )

        self.current_state = 'WAIT_ENTER_ELEVATOR'

    #  ------------------------------  END -------------------------------

    def wait_enter_elevator(self):
        self.get_logger().info("Aguardando entrar no elevador...")

    #  --------------------------  Wait for floor State -----------------------------

    def debug_sub_cb(self, msg):
        if msg.data:
            if self.simulation:
                # Teleport the robot to the correct floor in simulation
                teleport_robot( 
                    floor=4,  # Which floor to teleport
                    node=self, # Pass the current node instance
                    teleport_robot_client=self.teleport_robot # Pass the TeleportFloor client instance
                )

            else:
                pass # In real robot, we don't teleport, we just switch the floor

            self.start_navigation = False # Flag to alert the navigation is already started
            self.get_logger().info("Debug: Chegou no andar correto!")
            self.current_state = 'FLOOR_NAVIGATION'
        else:
            self.get_logger().info("Debug: Ainda não chegou no andar correto.")


    def wait_for_floor(self):
        # Aqui você pode monitorar a mudança de andar via tópico de visão
        self.get_logger().info("Se posicionando e aguardando chegada no 4º andar (via visão)...")
        # self.current_state = 'EXIT_ELEVATOR'


    #  ------------------------------  END -------------------------------

    #  --------------------------  Exit Elevator State -----------------------------

    def exit_elevator(self):
        self.get_logger().info("Executando saída do elevador (action)...")
        # switch_floor(
        #     floor=4,  # Which floor to start navigation
        #     node=self, # Pass the current node instance
        #     switch_floor_client=self.switch_floor, # Pass the SwitchFloor client instance
        #     simulation=self.simulation # Pass the simulation flag
        # )

        # start_checkpoints(
        #     floor="simulation_exit",  # Which floor to start checkpoints
        #     node=self, # Pass the current node instance
        #     start_checkpoint_client=self.startCheckpoint, # Pass the Checkpoints client instance
        # )

        # Você pode fazer igual ao `enter_elevator`
        # self.current_state = 'FLOOR_NAVIGATION'

    #  ------------------------------  END -------------------------------

    def error(self):
        self.get_logger().info("Error! Please check the logs for more details.")


    # def done(self):
    #     self.get_logger().info('FSM finalizada.')

def main(args=None):
    rclpy.init(args=args)
    node = ElevatorBehaviorManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
