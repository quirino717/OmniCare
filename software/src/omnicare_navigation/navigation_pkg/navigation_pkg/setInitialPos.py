import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseService(Node):
    def __init__(self):
        super().__init__('set_initial_pose_service')
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.srv = self.create_service(Empty, '/set_initial_pose', self.set_pose_callback)
        self.get_logger().info('Service /set_initial_pose pronto!')

    def set_pose_callback(self, request, response):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Pose inicial: (0, 0, 0)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0  # yaw = 0

        # Covariância
        msg.pose.covariance[0] = 0.25     # x
        msg.pose.covariance[7] = 0.25     # y
        msg.pose.covariance[35] = 0.0685  # yaw

        self.pose_publisher.publish(msg)
        self.get_logger().info('Pose inicial publicada pelo serviço!')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
