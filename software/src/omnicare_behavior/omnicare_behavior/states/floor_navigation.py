from omnicare_msgs.srv import SwitchFloor, Checkpoints  

# Andares mapeados
floor_map = {
    1: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/quartoAndar.yaml', 'x': 10.0, 'y': 10.0, 'yaw': 0.0},
    2: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/quintoAndarElevador.yaml', 'x': 5.0, 'y':   5.0, 'yaw': 0.0},
    3: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/simulation_map.yaml', 'x': 0.0, 'y':  0.0, 'yaw': 0.0},
    4: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/map.yaml', 'x': -3.0, 'y': 13.0, 'yaw': 0.0},
}


def start_floor_navigation(floor: int, node, switch_floor_client):
    """
    Envia uma requisição para troca de mapa e posicionamento com base no andar detectado.

    Args:
        floor (int): andar detectado
        node (rclpy.node.Node): instância do nó que chama a função
        switch_floor_client (Client): cliente do serviço SwitchFloor
    """
    
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
