import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from omnicare_msgs.srv import SwitchFloor  # <== substitua pelo nome real do seu pacote
import math
import time


# ------------- Quinto Andar --------------------
#
# 
# ros2 service call /omnicare/navigation/switch_floor omnicare_msgs/srv/SwitchFloor "{map_path: '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/
#                                                       omnicare_navigation/navigation_pkg/config/map/maps/quintoAndarElevador.yaml', x: -3.593, y: 12.41, yaw: 0.0}"
#

# ------------- Quarto Andar Simulation--------------------
# 
# ros2 service call /switch_floor omnicare_msgs/srv/SwitchFloor "{map_path: '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/
#                                                                       omnicare_navigation/navigation_pkg/config/map/maps/quartoAndar.yaml', x: -4.84, y: 2.79, yaw: 0.0}"


class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')
        self.cli = self.create_client(LoadMap, '/map_server/load_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()

    def send_request(self, map_url):
        self.req.map_url = map_url
        self.future = self.cli.call_async(self.req)

class SwitchFloorService(Node):
    def __init__(self):
        super().__init__('switch_floor_service')

        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_global = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.srv = self.create_service(SwitchFloor, '/omnicare/navigation/switch_floor', self.callback)
        self.get_logger().info('Serviço /switch_floor pronto!')

    def callback(self, request, response):
        map_loader = MapLoader()
        map_loader.send_request(request.map_path)
        rclpy.spin_once(map_loader)
        if map_loader.future.done():
            try:
                result = map_loader.future.result()
                map_loader.get_logger().info(f"Map loaded: {result.result}")
            except Exception as e:
                map_loader.get_logger().info(f'Service call failed: {e}')
                        
        map_loader.destroy_node()
        
        # 2. Limpa costmaps
        self.clear_global.call_async(ClearEntireCostmap.Request())
        self.clear_local.call_async(ClearEntireCostmap.Request())

        # 3. Publica pose inicial
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = request.x
        msg.pose.pose.position.y = request.y
        msg.pose.pose.orientation.z = math.sin(request.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(request.yaw / 2.0)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.pose_pub.publish(msg)



        self.get_logger().info(f'Mapa trocado e pose setada: ({request.x}, {request.y}, yaw={request.yaw})')
        response.success = True
        response.message = 'Mapa trocado e pose publicada com sucesso!'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SwitchFloorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
