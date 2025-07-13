import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rclpy.action import ActionClient

from omnicare_msgs.srv import SwitchFloor, Checkpoints


# from omnicare_behavior.action import EnterElevator
from omnicare_behavior.states.floor_navigation import start_floor_navigation, start_checkpoints


class ElevatorBehaviorManager(Node):

    def __init__(self):
        super().__init__('elevator_behavior_manager')

        # Define os estados e o mapa de transições
        self.states = {
            'FLOOR_NAVIGATION': self.floor_navigation,
            'ENTER_ELEVATOR': self.enter_elevator,
            'WAITING_CHECKPOINT_GOAL': self.waiting_checkpoints,
            'WAIT_FOR_FLOOR': self.wait_for_floor,
            'EXIT_ELEVATOR': self.exit_elevator,
            'ERROR': self.error
        }


        self.ret = False
        self.current_state = 'FLOOR_NAVIGATION'

        # Subscribes
        self.checkpoints_sub = self.create_subscription(
            Bool,
            '/checkpoint_done',
            self.checkpoints_callback,
            10
        )

        # Clients
        self.switch_floor = self.create_client(SwitchFloor, '/omnicare/navigation/switch_floor')
        self.startCheckpoint = self.create_client(Checkpoints, '/omnicare/checkpoints/start')



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
            self.current_state = 'ENTER_ELEVATOR'

    def floor_navigation(self):
        self.get_logger().info("Iniciando navegação no andar...")

        start_floor_navigation(
            floor=3,  # Which floor to start navigation
            node=self, # Pass the current node instance
            switch_floor_client=self.switch_floor, # Pass the SwitchFloor client instance
        )

        start_checkpoints(
            floor="quarto_andar",  # Which floor to start checkpoints
            node=self, # Pass the current node instance
            start_checkpoint_client=self.startCheckpoint, # Pass the Checkpoints client instance
        )

        self.current_state = 'WAITING_CHECKPOINT_GOAL'

        



        # if not self.activate_floor_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().error('Serviço de ativação de andar não disponível.')
        #     self.current_state = 'DONE'
        #     return

        # req = Trigger.Request()
        # future = self.activate_floor_client.call_async(req)
        # future.add_done_callback(self.floor_nav_response)


    # def floor_nav_response(self, future):
    #     resp = future.result()
    #     if resp.success:
    #         self.get_logger().info("Navegação no novo andar ativada com sucesso!")
    #     else:
    #         self.get_logger().warn(f"Falha ao ativar navegação: {resp.message}")
    #     self.current_state = 'DONE'



    # def go_to_elevator(self):
    #     self.get_logger().info('NAV2 levando até o elevador...')
    #     # Suponha que NAV2 já foi acionado antes de iniciar FSM
    #     self.current_state = 'ENTER_ELEVATOR'

    #  ------------------------------  END -------------------------------

    def waiting_checkpoints(self):
        self.get_logger().info("Aguardando sinal de checkpoint atingido...")
        pass

    #  --------------------------  Enter in Elevator State -----------------------------

    def enter_elevator(self):
        self.get_logger().info('Chamando action para entrar no elevador...')

        # if not self.enter_elevator_client.wait_for_server(timeout_sec=2.0):
        #     self.get_logger().warn('Servidor da action não está disponível.')
        #     return

        # goal_msg = EnterElevator.Goal()
        # self._send_goal_future = self.enter_elevator_client.send_goal_async(goal_msg)
        # self._send_goal_future.add_done_callback(self.enter_elevator_response)

        # self.current_state = 'WAIT_FOR_FLOOR'

    # def enter_elevator_response(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().warn('Goal foi rejeitado.')
    #         self.current_state = 'DONE'
    #         return

    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.enter_elevator_done)

    # def enter_elevator_done(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f"Resultado da action: {result.message}")
    #     self.current_state = 'WAIT_FOR_FLOOR'


    #  ------------------------------  END -------------------------------

    #  --------------------------  Wait for floor State -----------------------------


    def wait_for_floor(self):
        # Aqui você pode monitorar a mudança de andar via tópico de visão
        self.get_logger().info("Aguardando chegada no 4º andar (via visão)...")
        # time.sleep(3)  # Simulação
        self.current_state = 'EXIT_ELEVATOR'


    #  ------------------------------  END -------------------------------

    #  --------------------------  Exit Elevator State -----------------------------

    def exit_elevator(self):
        self.get_logger().info("Executando saída do elevador (action)...")
        # Você pode fazer igual ao `enter_elevator`
        self.current_state = 'FLOOR_NAVIGATION'

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
