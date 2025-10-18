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

import cv2
from cv_bridge import CvBridge

class DatasetGenerator(Node):
    def __init__(self):
        super().__init__('dataset_generator')
        self.params = Parameters(self)
        self.bridge = CvBridge()

        self.img_subscriber = self.create_subscription(
            Image, 
            self.params.img_topic_name,
            self.img_subscriber_callback,
            SensorDataQoS
        )

        self.frames_count = 0
        self.period_s = 1/self.params.output_frame_fps
        self.last_time_save_frame = 0
    
    def img_subscriber_callback(self, msg):
        
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().debug(f'Recieved img time:{timestamp}')
        
        if timestamp - self.last_time_save_frame > self.period_s:
            self.last_time_save_frame = timestamp

            filename = self.params.output_prefix_filename + f'{self.frames_count:03d}.png'
            cv2.imwrite(self.params.output_dir_path + filename, self.bridge.imgmsg_to_cv2(msg, "bgr8"))
            self.get_logger().debug(f'Saving {self.params.output_dir_path + filename}')
            
            self.frames_count = self.frames_count + 1        

def main(args=None):
    rclpy.init(args=args)

    dataset_generator = DatasetGenerator()

    rclpy.spin(dataset_generator)

    rclpy.shutdown()


if __name__ == '__main__':
    main()