import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
SensorDataQoS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE
)

from sensor_msgs.msg import Image
from std_msgs.msg import String
from omnicare_msgs.srv import OnOffNode

from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import platform
from collections import deque, Counter
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge

class FloorDetector(Node):
    def __init__(self):
        super().__init__('floor_detector')
        self.srv = self.create_service(OnOffNode,
                                       "floor_detector/OnOffNode",
                                       self.on_off_node_callbalck)
        self.__nodeActivate = True

        self.cam_subscriber = None
        self.image_raw = None
        self.bridge = CvBridge()
        self.get_logger().debug("CV Bridge initialized")

        self.floor_publisher = self.create_publisher(String,
                                                     "floor_detector/atual_floor",
                                                     SensorDataQoS)
        
        self.display_publisher = self.create_publisher(Image,
                                                       "floor_detector/elevator_display",
                                                       SensorDataQoS)
        
        yolo_format = '.pt'
        system_architecture = platform.processor() # Get system architecture 
        if(system_architecture == 'aarch64'):
            yolo_format = '.engine'            
    
        self.declare_parameter("model_display_path", f"{get_package_share_directory('floor_detector')}/weights/best_display")
        model_display_path = self.get_parameter("model_display_path").get_parameter_value().string_value + yolo_format
        self.model_display = YOLO(model_display_path, task='detect')
        self.get_logger().debug(f'Display Model path: {model_display_path}')

        
        self.declare_parameter("model_floor_path", f"{get_package_share_directory('floor_detector')}/weights/best_floor")
        model_floor_path = self.get_parameter("model_floor_path").get_parameter_value().string_value + yolo_format
        self.model_floor = YOLO(model_floor_path, task='classify')
        self.get_logger().debug(f'Floor Model path: {model_floor_path}')

        self.last_detections = deque(maxlen=10)

        self.get_logger().info('Floor detector node initialized')

    def on_off_node_callbalck(self, request, response):
        self.get_logger().info(f"Incoming request\nActivate: {request.activate}")
        response.result = True

        self.__nodeActivate = request.activate
        if self.__nodeActivate:
            try:
                self.cam_subscriber = self.create_subscription(Image,
                                                               '/camera1/image_raw',
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
        self.get_logger().debug('Received an image')
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.detect_floor(self.image_raw.copy())

    def detect_floor(self, img):
        floor_msg = String()
        
        display_results = self.model_display(self.image_raw, verbose=False)[0]

        # annotated_frame = display_results.plot()
        # cv2.imshow("YOLO11 Tracking", annotated_frame)
        # cv2.waitKey(1)

        if len(display_results.boxes) == 0:
            return
        
        # finds first the elevator display
        display_img = self.find_display(img, display_results)
        self.display_publisher.publish(self.bridge.cv2_to_imgmsg(display_img))
        
        # Model was treined with 640x640 with filled edges
        display_img = self.preprocessing_display_img(display_img, size=640)
        # Classification of the elevator display
        floor_results = self.model_floor(display_img, verbose=False)[0]

        floor_filtered = self.filter_floor_detection(floor_results)
        floor_msg.data = floor_filtered
        self.floor_publisher.publish(floor_msg)
    
    def find_display(self, img, results):    
        if len(results.boxes) == 0:
            return
        _ = results.boxes[0].xyxy.tolist()[0] # Get xyxy position of BoundingBox
        int_xyxy = [int(float(x)) for x in _]
        return img[int_xyxy[1]:int_xyxy[3], int_xyxy[0]:int_xyxy[2]].copy() 
    
    def preprocessing_display_img(self, img, size):
        h, w = img.shape[:2]
        
        # Calculate the scale for resize mantain the proportion
        scale = size / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized_img = cv2.resize(img, (new_w, new_h), cv2.INTER_LINEAR)

        d_h, d_w = size - new_h, size - new_w

        # d_h - (d_h // 2) is for odd sizes
        # Ex: d_h = 5, top = 2 bottom = 3
        top, bottom = d_h // 2, d_h - (d_h // 2)
        left, right = d_w // 2, d_w - (d_w // 2)
        # Fill edges with black
        preprossed_img = cv2.copyMakeBorder(resized_img,
                                            top, bottom, left, right,
                                            cv2.BORDER_CONSTANT, value=0)

        return preprossed_img

    def filter_floor_detection(self, results):
        floor = results.names[results.probs.top5[0]]        
        self.last_detections.append(floor)
        last_detections_counter = Counter(self.last_detections)
        mode = last_detections_counter.most_common(1)[0][0] # Get the mode of last detections
        return mode

def main(args=None):
    rclpy.init(args=args)

    floor_detector = FloorDetector()

    rclpy.spin(floor_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()