from rclpy.node import Node

class Parameters:
    def __init__(self, node : Node):

        node.declare_parameter("usb_port_name", "/dev/usb-user")
        self.usb_port_name = (
            node.get_parameter("usb_port_name")
            .get_parameter_value().string_value
        )

        node.declare_parameter("baud_rate", 115200)
        self.baud_rate = (
            node.get_parameter("baud_rate")
            .get_parameter_value().integer_value
        )
        
        node.declare_parameter('main_callback_delay_s', 0.1)
        self.main_callback_delay_s = (
            node.get_parameter('main_callback_delay_s')
            .get_parameter_value().double_value
        )
        