import rclpy

from geometry_msgs.msg import Twist
from omnicare_msgs.srv import OnOffNode 

def align_the_robot(speed_publisher, direction):
    """
    Publica comandos de velocidade para o robô se alinhar com o display do elevador.

    Args:
        speed_publisher (Publisher): publicador para o tópico cmd_vel
        direction (str): direção para alinhar ("Left" ou "Right")
    """
    twist = Twist()
    if direction == "Right":
        twist.angular.z = -0.1 
    elif direction == "Left":
        twist.angular.z = 0.1   
    else:
        twist.angular.z = 0.0   

    speed_publisher.publish(twist)

def activate_display_inference(node, activate_inference_client,simulation):
    """
    Envia uma requisição para alinhamento do robô com o display.

    Args:
        node (rclpy.node.Node): instância do nó que chama a função
        switch_floor_client (Client): cliente do serviço SwitchFloor
        simulation: variavel booleana para distinguir se está na simulação ou não
    """
    
    inference_request = OnOffNode.Request()
    inference_request.activate = True

    if not activate_inference_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Serviço de ativação da inferência não está disponível.")
        return

    future = activate_inference_client.call_async(inference_request)

    def response_callback(future):
        try:
            result = future.result()
            if result.result:
                node.get_logger().info(f"Inferência ativada com sucesso.")
            else:
                node.get_logger().warn(f"Falha ao ativar a inferência: {result.message}")
                node.current_state = 'ERROR'
        except Exception as e:
            node.get_logger().error(f"Erro ao ativar a inferência: {e}")
            node.current_state = 'ERROR'

    future.add_done_callback(response_callback)

def deactivate_display_inference(node, activate_inference_client,simulation):
    """
    Envia uma requisição para alinhamento do robô com o display.

    Args:
        node (rclpy.node.Node): instância do nó que chama a função
        switch_floor_client (Client): cliente do serviço SwitchFloor
        simulation: variavel booleana para distinguir se está na simulação ou não
    """
    
    inference_request = OnOffNode.Request()
    inference_request.activate = ''

    if not activate_inference_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Serviço de desativação da inferência não está disponível.")
        return

    future = activate_inference_client.call_async(inference_request)

    def response_callback(future):
        try:
            result = future.result()
            if result.result:
                node.get_logger().info(f"Inferência desativada com sucesso.")
            else:
                node.get_logger().warn(f"Falha ao desativar a inferência: {result.message}")
                node.current_state = 'ERROR'
        except Exception as e:
            node.get_logger().error(f"Erro ao desativar a inferência: {e}")
            node.current_state = 'ERROR'

    future.add_done_callback(response_callback)



    