import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_path

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
SensorDataQoS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE
)

from sensor_msgs.msg import Image

from .get_parameters import Parameters

class DatasetGenerator(Node):
    def __init__(self):
        super().__init__('dataset_generator')
        self.params = Parameters(self)

        self.img_subscriber = self.create_subscription(Image, 
                                                       self.params.img_topic_name,
                                                       self.img_subscriber_callback,
                                                       SensorDataQoS)        

    
    def img_subscriber_callback(self, msg):
        self.get_logger().info(f'{msg.header}')
        

def main(args=None):
    rclpy.init(args=args)

    dataset_generator = DatasetGenerator()

    rclpy.spin(dataset_generator)

    rclpy.shutdown()


if __name__ == '__main__':
    main()