from omnicare_msgs.srv import SwitchFloor, Checkpoints  
from ament_index_python.packages import get_package_share_directory
from omnicare_behavior.utils.parsing_yaml import extract_map_configuration


# Só para não perder os checkpoints. Não apagar!!!!
# floor_map = {
#     1: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/quartoAndar.yaml', 'x': 10.0, 'y': 10.0, 'yaw': 0.0},
#     2: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/quintoAndarElevador.yaml', 'x': 1.72, 'y':   -0.68, 'yaw': 0.0},
#     3: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/simulation_map.yaml', 'x': 0.0, 'y':  0.0, 'yaw': 0.0},
#     4: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/simulation_map.yaml', 'x': 5.18, 'y': 3.55, 'yaw': 3.14},
# }


def switch_floor(floor: int, node, switch_floor_client,simulation):
    """
    Envia uma requisição para troca de mapa e posicionamento com base no andar detectado.

    Args:
        floor (int): andar detectado
        node (rclpy.node.Node): instância do nó que chama a função
        switch_floor_client (Client): cliente do serviço SwitchFloor
        simulation: variavel booleana para distinguir se está na simulação ou não
    """
    
    floor_map = extract_map_configuration(simulation)

    if floor not in floor_map:
        node.get_logger().warn(f"Andar {floor} não mapeado.")
        return False

    config = floor_map[floor]
    req = SwitchFloor.Request()
    req.map_path = config['map_path']
    req.x = config['x']
    req.y = config['y']
    req.yaw = config['yaw']

    node.get_logger().info(f"Trocando para o andar {floor} com mapa {req.map_path}...")

    if not switch_floor_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Serviço de troca de mapa não está disponível.")
        return


    future = switch_floor_client.call_async(req)

    def response_callback(future):
        try:
            result = future.result()
            if result.success:
                node.get_logger().info(f"Mapa do andar {floor} trocado com sucesso.")
            else:
                node.get_logger().warn(f"Falha ao carregar mapa: {result.message}")
                node.current_state = 'ERROR'
        except Exception as e:
            node.get_logger().error(f"Erro na chamada do serviço SwitchFloor: {e}")
            node.current_state = 'ERROR'

    future.add_done_callback(response_callback)

def start_checkpoints(floor: str, node, start_checkpoint_client):
    """
    Inicia o serviço de seguir os checkpoints.

    Args:
        floor (string): andar para iniciar os checkpoints
        node (rclpy.node.Node): instância do nó que chama a função
        start_checkpoint_client (Client): cliente do serviço Checkpoints
    """
    
    req = Checkpoints.Request()
    req.floor = floor
    

    node.get_logger().info(f"Iniciando checkpoints para o andar {floor}...")

    if not start_checkpoint_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Serviço de Checkpoints não está disponível.")
        return

    future = start_checkpoint_client.call_async(req)
    

    def response_callback(future):
        try:
            result = future.result()
            if result.success:
                node.get_logger().info(f"Checkpoints iniciados com sucesso para o andar {floor}.")
            else:
                node.get_logger().warn(f"Falha ao iniciar checkpoints: {result.message}")
                node.current_state = 'ERROR'
        except Exception as e:
            node.get_logger().error(f"Erro na chamada do serviço Checkpoints: {e}")
            node.current_state = 'ERROR'

    future.add_done_callback(response_callback)
