#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

# Patch para corrigir bibliotecas antigas que usam np.float
if not hasattr(np, 'float'):
    np.float = float

from tf_transformations import euler_from_quaternion

class AMCLOrientationNode(Node):
    def __init__(self):
        super().__init__('amcl_orientation_node')

        # Parâmetros (se quiser mudar nomes dos tópicos via launch/params)
        amcl_topic = self.declare_parameter('amcl_topic', '/amcl_pose').get_parameter_value().string_value

        pub_topic = self.declare_parameter('pub_topic', '/omnicare/navigation/amcl_orientation').get_parameter_value().string_value

        # Publisher: [x, y, z, yaw(rad)]
        self.pub = self.create_publisher(Float64MultiArray, pub_topic, 10)

        # Subscriber do AMCL
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self._amcl_cb,
            10
        )

        self.get_logger().info(
            f'AMCL Orientation node up. Sub: {amcl_topic}  -> Pub: {pub_topic} ([x,y,z,yaw])'
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        # Posição
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Quaternion -> yaw
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])  # radianos

        out = Float64MultiArray()
        out.data = [x, y, z, yaw]
        self.pub.publish(out)

        # (opcional) debug reduzido
        self.get_logger().debug(f"x={x:.3f} y={y:.3f} z={z:.3f} yaw={yaw:.3f} rad")

def main():
    rclpy.init()
    node = AMCLOrientationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
