#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import InverseKin


class InverseKinClient(Node):
    def __init__(self):
        super().__init__('inverse_kin_client')

        # Dobot IK service
        self.cli = self.create_client(
            InverseKin,
            '/dobot_bringup_ros2/srv/InverseKin'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for InverseKin service...')

        self.req = InverseKin.Request()

    def send_request(self, x, y, z, rx, ry, rz,
                 use_joint_near="0",
                 joint_near="",
                 user="0",
                 tool="0"):

        self.req.x = float(x)
        self.req.y = float(y)
        self.req.z = float(z)
        self.req.rx = float(rx)
        self.req.ry = float(ry)
        self.req.rz = float(rz)

        self.req.use_joint_near = use_joint_near
        self.req.joint_near = joint_near
        self.req.user = user
        self.req.tool = tool

       
        self.get_logger().info(
            f"SENDING: x={self.req.x}, y={self.req.y}, z={self.req.z}, "
            f"rx={self.req.rx}, ry={self.req.ry}, rz={self.req.rz}, "
            f"user={self.req.user}, tool={self.req.tool}, "
            f"use_joint_near={self.req.use_joint_near}, joint_near={self.req.joint_near}"
        )

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinClient()

    # ----------------------------------------------------------
    # EXAMPLE POSE (change these values!)
    # XYZ in millimeters
    # Rx Ry Rz in degrees (ZYX Euler)
    #
    # NOTE: THIS POSE **IS A REAL CR5 REACHABLE POSE**
    # ----------------------------------------------------------
    x = 473.0
    y = -141.0
    z = 469.0
    rx = 180.0
    ry = 0.0
    rz = 180.0

    result = node.send_request(
        x=x,
        y=y,
        z=z,
        rx=rx,
        ry=ry,
        rz=rz,
        use_joint_near="0",
        joint_near="",
        user="0",
        tool="0"
    )

    if result:
        node.get_logger().info(f"InverseKin result: res={result.res}")
    else:
        node.get_logger().error("No response from robot!")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
