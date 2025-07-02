import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
SensorDataQoS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE
)

from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from omnicare_msgs.srv import OnOffNode

from ament_index_python.packages import get_package_share_directory
import os
from collections import deque, Counter
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge

class FloorDetector(Node):
    def __init__(self):
        super().__init__('flor_detector')
        self.srv = self.create_service(OnOffNode,
                                       "floor_detector/OnOffNode",
                                       self.on_off_node_callbalck)
        self.__nodeActivate = True

        self.cam_subscriber = None
        self.image_raw = None
        self.bridge = CvBridge()
        self.get_logger().debug("CV Bridge initialized")

        self.floor_publisher = self.create_publisher(Int8,
                                                     "floor_detector/atual_floor",
                                                     SensorDataQoS)
        
        self.display_publisher = self.create_publisher(Image,
                                                       "floor_detector/elevator_display",
                                                       SensorDataQoS)
        

        self.declare_parameter("model_display", f"{get_package_share_directory('floor_detector')}/weights/best_display.pt")
        self.model_display = YOLO(self.get_parameter("model_display").get_parameter_value().string_value)
        
        self.declare_parameter("model_floor", f"{get_package_share_directory('floor_detector')}/weights/best_floor.pt")
        self.model_floor = YOLO(self.get_parameter("model_display").get_parameter_value().string_value)

        self.last_detections = deque(maxlen=10)

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
        self.detect_floor(self.image_raw.copy())

    # TODO: Terminar        
    def detect_floor(self, img):
        floor_msg = Int8()
        
        display_results = self.model_display(self.image_raw, verbose=False)[0]
        annotated_frame = display_results.plot()
        cv2.imshow("YOLO11 Tracking", annotated_frame)
        cv2.waitKey(1)

        if len(display_results.boxes) == 0:
            return
        
        display_img = self.find_display(img, display_results)
        self.display_publisher.publish(self.bridge.cv2_to_imgmsg(display_img))
        
        floor_results = self.model_floor(display_img, vebose=False)[0]
        floor_filtered = self.filter_floor_detection(floor_results)
        
        # floor_msg.data = int(results.boxes[0].cls)
        # self.floor_publisher.publish(floor_msg)

    def find_display(self, img, results):
        
        if len(results) == 0:
            return
        _ = results.boxes[0].xyxy.tolist()[0] # Get xyxy position of BoundingBox
        int_xyxy = [int(float(x)) for x in _]
        return img[int_xyxy[1]:int_xyxy[3], int_xyxy[0]:int_xyxy[2]].copy() 
    
    # TODO: terminar essa funcao
    def filter_floor_detection(self, results):
        floor = self.model_floor.names[results.boxes[0].cls]
        print(floor)
        
        # self.last_detections.append(floor)
        # last_detections_counter = Counter(self.last_detections)
        # mode = last_detections_counter(1)[0][0] # Get the mode of last detections
        
        # return mode
    

def main(args=None):
    rclpy.init(args=args)

    floor_detector = FloorDetector()

    rclpy.spin(floor_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()