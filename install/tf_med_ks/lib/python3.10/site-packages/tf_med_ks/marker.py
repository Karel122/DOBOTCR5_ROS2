#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PointPublisher(Node):
    def __init__(self):
        super().__init__('point_publisher')
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_point)  # 1 Hz

    def publish_point(self):
        marker = Marker()
        marker.header.frame_id = "kamera"   # or any frame you want
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position of the point (in meters)
        # marker.pose.position.x = -0.072
        # marker.pose.position.y = 0.41
        # marker.pose.position.z = 0.26

        marker.pose.position.x = 0.45
        marker.pose.position.y = 1.06 
        marker.pose.position.z = 0.34
        
        

        # No orientation for a point
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size of the sphere
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Color (RGBA)
        marker.color.a = 1.0  # alpha
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.publisher.publish(marker)
        self.get_logger().info("Published point marker")

def main(args=None):
    rclpy.init(args=args)
    node = PointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
