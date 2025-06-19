import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
SensorDataQoS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE
)

from sensor_msgs.msg import Image
from omnicare_msgs.srv import OnOffNode

from ament_index_python.packages import get_package_share_directory
import os
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge

class FloorDetector(Node):
    def __init__(self):
        super().__init__('flor_detector_node')
        self.srv = self.create_service(OnOffNode,
                                       "floor_detector/OnOffNode",
                                       self.on_off_node_callbalck)
        self.__nodeActivate = False

        self.cam_subscriber = None
        self.image_raw = None
        self.bridge = CvBridge()
        self.get_logger().debug("CV Bridge initialized")

        self.timer_main = self.create_timer(0.008, self.main_callback)

        self.declare_parameter("model", f"{get_package_share_directory('floor_detector')}/weights/yolo11n.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value) # Load model
        self.value_classes = self.get_classes()  # define antes de usar
        self.get_logger().info(f"CLASSES DO MODELO: {self.value_classes}")
        print(self.get_parameter("model").get_parameter_value().string_value)

        
    def on_off_node_callbalck(self, request, response):
        self.get_logger().info(f"Incoming request\nActivate: {request.activate}")
        response.result = True

        self.__nodeActivate = request.activate
        if self.__nodeActivate:
            try:
                self.cam_subscriber = self.create_subscription(Image,
                                                               'camera1/image_raw',
                                                               self.cam_subscriber_callback,
                                                               SensorDataQoS)
            except:
                response.result = False
                self.destroy_subscription(self.cam_subscriber)
                self.cam_subscriber = None
        else:
            self.destroy_subscription(self.cam_subscriber)
            self.cam_subscriber = None

        return response
    
    def cam_subscriber_callback(self, msg):
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        
    def get_classes(self): 
        classes = self.model.names
        value_classes = {value: key for key, value in classes.items()}
        return value_classes


    def main_callback(self):
        if self.__nodeActivate:
            try:
                cv2.imshow("Image Raw", self.image_raw)
                cv2.waitKey(1)
            except:
                pass    
        


def main(args=None):
    rclpy.init(args=args)

    floor_detector = FloorDetector()

    rclpy.spin(floor_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()