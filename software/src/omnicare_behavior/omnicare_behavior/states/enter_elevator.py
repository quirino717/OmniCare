from omnicare_msgs.action import EnterElevator  

def enter_elevator_behavior(node, action_client):
    """
    Inicia o comportamento de entrar no elevador.

    Args:
        node (rclpy.node.Node): instância do nó que chama a função
        action_client (ActionClient): cliente do serviço EnterElevator
    """
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Servidor EnterElevator não está disponível.")
        return False

    goal_msg = EnterElevator.Goal()
    action_client.wait_for_server()


    def feedback_cb(feedback_msg):
        feedback = feedback_msg.feedback
        node.get_logger().info(f"Feedback elevator: {feedback.robot_feedback}")

    def response_cb(future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                node.get_logger().info('Goal rejected :(')
                return

            node.get_logger().info('Goal accepted :)')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(get_result_callback)                
            
        except Exception as e:
            node.get_logger().error(f"Erro na ação EnterElevator: {e}")
            node.current_state = 'ERROR'

    def get_result_callback(future):
        result = future.result().result
        if result.success:
            node.get_logger().info("Entrou no elevador com sucesso.")
            node.current_state = 'WAIT_FOR_FLOOR'
        else:
            node.get_logger().warn(f"Falha ao entrar no elevador: {result.message}")
            node.current_state = 'ERROR'


    future = action_client.send_goal_async(goal_msg,feedback_callback=feedback_cb)
    future.add_done_callback(response_cb)