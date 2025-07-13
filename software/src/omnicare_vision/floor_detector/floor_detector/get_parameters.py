from rclpy.node import Node

class Parameters:
    def __init__(self, node : Node):

        node.declare_parameter('model_display_name', 'best_display')
        self.model_display_name = node.get_parameter("model_display_name").get_parameter_value().string_value

        node.declare_parameter('model_floor_name', 'best_floor')
        self.model_floor_name = node.get_parameter("model_floor_name").get_parameter_value().string_value

        node.declare_parameter('filter_deque_size', 10)
        self.filter_deque_size = node.get_parameter('filter_deque_size').get_parameter_value().integer_value

        node.declare_parameter('lost_display_timeout_ms', 800)
        self.lost_display_timeout_ms = node.get_parameter('lost_display_timeout_ms').get_parameter_value().integer_value
        
        node.declare_parameter('main_callback_delay_s', 0.1)
        self.main_callback_delay_s = node.get_parameter('main_callback_delay_s').get_parameter_value().double_value
        