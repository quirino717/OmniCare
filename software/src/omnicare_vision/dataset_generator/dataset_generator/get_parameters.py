from rclpy.node import Node

class Parameters:
    def __init__(self, node : Node):

        node.declare_parameter('img_topic_name', 'camera1/image_raw')
        self.img_topic_name = node.get_parameter('img_topic_name').get_parameter_value().string_value

        node.declare_parameter('output_frame_fps', 3)
        self.output_frame_fps = node.get_parameter('output_frame_fps').get_parameter_value().integer_value

        node.declare_parameter('output_prefix_filename', 'frame')
        self.output_prefix_filename = node.get_parameter('output_prefix_filename').get_parameter_value().string_value

        node.declare_parameter('output_dir_path', 'frame')
        self.output_dir_path = node.get_parameter('output_dir_path').get_parameter_value().string_value
