#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

class TimePublisher(Node):
    def __init__(self):
        super().__init__('time_publisher_pc')
        self.pub = self.create_publisher(Time, '/pc_time', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)  # 1 Hz

    def timer_cb(self):
        t = self.get_clock().now().to_msg()
        self.pub.publish(t)

def main():
    rclpy.init()
    node = TimePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
