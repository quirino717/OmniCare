from rclpy.node import Node

class Parameters:
    def __init__(self, node : Node):

        node.declare_parameter('img_topic_name', 'camera1/image_raw')
        self.img_topic_name = node.get_parameter('img_topic_name').get_parameter_value().string_value

        node.declare_parameter('fps_to_save', 3)
        self.fps_to_save = node.get_parameter('fps_to_save').get_parameter_value().integer_value