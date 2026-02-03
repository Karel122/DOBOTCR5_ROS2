#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from math import radians


class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')

        # Ustvari broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)

        # Pripravi sporočilo
        static_tf = TransformStamped()

        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'   # parent frame
        static_tf.child_frame_id = 'user_3'       # child frame

        # Translacija (v metrih)
        static_tf.transform.translation.x = -0.35
        static_tf.transform.translation.y =  0.23
        static_tf.transform.translation.z =  0.53

        # Rotacija (v Euler, nato v kvaternion)
        q = quaternion_from_euler(radians(-90.0), 0.0, 0.0)  # roll, pitch, yaw
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]

        # Objavi transformacijo
        self.broadcaster.sendTransform(static_tf)
        self.get_logger().info('Static transform base_link → user_ published!')


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
