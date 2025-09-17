#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

def yaw_to_quat(yaw):
    # retorna x,y,z,w
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))

class PoseFromTF(Node):
    def __init__(self):
        super().__init__('pose_from_tf')

        # Parâmetros configuráveis
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('rate_hz', 10.0)

        self.target = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source = self.get_parameter('source_frame').get_parameter_value().string_value
        self.rate = self.get_parameter('rate_hz').get_parameter_value().double_value

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher de PoseStamped 
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(PoseStamped, '/pose_from_tf', qos)

        self.timer = self.create_timer(1.0 / max(1e-3, self.rate), self._tick)
        self.get_logger().info(f'Publicando /pose_from_tf como {self.target}←{self.source} @ {self.rate:.1f} Hz')

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.target, self.source, rclpy.time.Time())
        except Exception as e:
            # Evita spam
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PoseFromTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
