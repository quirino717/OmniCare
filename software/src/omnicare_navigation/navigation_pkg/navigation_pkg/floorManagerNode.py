import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from omnicare_msgs.srv import SwitchFloor  

class FloorManager(Node):
    def __init__(self):
        super().__init__('floor_manager')

        self.subscription = self.create_subscription(
            Int32,
            'omnicare/elevator_display_detected',
            self.floor_callback,
            10)

        self.cli = self.create_client(SwitchFloor, '/omnicare/navigation/switch_floor')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando serviço /switch_floor...')
        
        # Path of the map yaml files
        self.declare_parameter('map_file', '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/checkpoints/quarto_andar_checkpoints.json')

        # Andares mapeados
        self.floor_map = {
            1: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/quartoAndar.yaml', 'x': 10.0, 'y': 10.0, 'yaw': 0.0},
            2: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/quintoAndarElevador.yaml', 'x': 5.0, 'y':   5.0, 'yaw': 0.0},
            3: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/map.yaml', 'x': 2.0, 'y':  2.0, 'yaw': 0.0},
            4: {'map_path': '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/maps/map.yaml', 'x': -3.0, 'y': 13.0, 'yaw': 0.0},
        }

        self.last_floor = None  # Evita chamadas duplicadas

    def floor_callback(self, msg):
        floor = msg.data
        if floor == self.last_floor:
            return  # Ignora se já estamos nesse andar
        self.last_floor = floor

        if floor not in self.floor_map:
            self.get_logger().warn(f"Andar {floor} não mapeado.")
            return

        config = self.floor_map[floor]
        req = SwitchFloor.Request()
        req.map_path = config['map_path']
        req.x = config['x']
        req.y = config['y']
        req.yaw = config['yaw']

        self.get_logger().info(f"Trocando para o andar {floor} com mapa {req.map_path}...")
        future = self.cli.call_async(req)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Mapa trocado com sucesso: {response.message}")
            else:
                self.get_logger().error(f"Falha ao trocar o mapa: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar serviço: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FloorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
