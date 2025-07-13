import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse

from omnicare_msgs.action import EnterElevator  # Auto-gerado após build
import time

class EnterElevatorServer(Node):

    def __init__(self):
        super().__init__('enter_elevator_server')

        self._action_server = ActionServer(
            self,
            EnterElevator,
            'enter_elevator',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        self.get_logger().info('Recebido pedido de entrar no elevador')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelamento recebido')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executando ação de entrar no elevador...')

        feedback_msg = EnterElevator.Feedback()
        for i in range(5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Ação cancelada')
                return EnterElevator.Result(success=False, message='Cancelado')

            feedback_msg.progress_percentage = float((i+1) * 20)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progresso: {feedback_msg.progress_percentage}%')
            time.sleep(1)

        goal_handle.succeed()
        result = EnterElevator.Result()
        result.success = True
        result.message = 'Elevador entrado com sucesso!'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = EnterElevatorServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
