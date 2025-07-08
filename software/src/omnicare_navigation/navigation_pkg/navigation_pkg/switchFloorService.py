import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from omnicare_msgs.srv import SwitchFloor  # <== substitua pelo nome real do seu pacote
import math
import time

class SwitchFloorService(Node):
    def __init__(self):
        super().__init__('switch_floor_service')

        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_global = self.create_client(Empty, '/global_costmap/clear_entirely_nav2')
        self.clear_local = self.create_client(Empty, '/local_costmap/clear_entirely_nav2')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.srv = self.create_service(SwitchFloor, '/switch_floor', self.callback)
        self.get_logger().info('Serviço /switch_floor pronto!')

    def callback(self, request, response):
        # Espera serviços estarem prontos
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = 'Serviço /map_server/load_map indisponível'
            return response

        self.get_logger().info(f'Trocando para o mapa: {request.map_path}')

        # 1. Troca de mapa
        map_req = LoadMap.Request()
        map_req.map_url = request.map_path
        future = self.map_client.call_async(map_req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)

        # 2. Limpa costmaps
        self.clear_global.call_async(Empty.Request())
        self.clear_local.call_async(Empty.Request())
        time.sleep(0.5)

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

        self.get_logger().info(f'Mapa trocado e pose setada em x={request.x}, y={request.y}, yaw={request.yaw}')
        response.success = True
        response.message = 'Mapa trocado e pose inicial publicada com sucesso'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SwitchFloorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
