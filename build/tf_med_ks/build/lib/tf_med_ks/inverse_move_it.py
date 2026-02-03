#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from tf_transformations import quaternion_from_euler
from dobot_msgs_v4.srv import SetFloat
from dobot_msgs_v4.msg import PointArray
import math
import json
from datetime import datetime


# ✔ Joint limits in radians (real CR5)
JOINT_LIMITS = {
    0: (-6.2657, 6.2657),       # joint1 ±359°
    1: (-6.2657, 6.2657),       # joint2 ±359°
    2: (-2.86234, 2.86234),     # joint3 ±164°
    3: (-6.2657, 6.2657),       # joint4 ±359°
    4: (-6.2657, 6.2657),       # joint5 ±359°
    5: (-6.2657, 6.2657)        # joint6 ±359°
}


class IKClient(Node):

    def __init__(self):
        super().__init__('ik_client')

        # MoveIt IK service
        self.cli = self.create_client(GetPositionIK, '/compute_ik')

        # Get transformed TCP points
        self.client_get_tcp = self.create_client(SetFloat, '/poslji_transformirane_tcp_tocke')

        # Provide joint trajectory to external node
        self.server = self.create_service(
            SetFloat,
            '/poslji_tcp_tocke_base_link',
            self.send_points_base_link
        )

        # Wait for services
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for IK service...")

        while not self.client_get_tcp.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /poslji_transformirane_tcp_tocke ...")

        self.req_tcp = SetFloat.Request()
        self.all_joint_solutions_msg = []
        self.all_joint_solutions_deg = []
        self.tcp_points = []


    # ---------------------------------------------------------------------
    # Angle utilities
    # ---------------------------------------------------------------------

    def wrap_angle(self, a):
        """Wraps angle to [-π, π]"""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def clamp(self, idx, angle):
        """Clamp angle to joint limits"""
        jmin, jmax = JOINT_LIMITS[idx]
        return max(jmin, min(jmax, angle))

    def normalize_all(self, joints_rad):
        """Clamp all joints to valid CR5 ranges"""
        return [self.clamp(i, joints_rad[i]) for i in range(6)]

    def smooth_joint_angles(self, prev, target):
        """Avoid unnecessary 360° jumps."""
        delta = target - prev
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return prev + delta

    def adjust_trajectory(self, joint_angles):
        """Ensure smooth angle transitions."""
        adjusted = [joint_angles[0]]
        for i in range(1, len(joint_angles)):
            adjusted.append(self.smooth_joint_angles(adjusted[i - 1], joint_angles[i]))
        return adjusted


    # ---------------------------------------------------------------------
    # Services
    # ---------------------------------------------------------------------

    def send_points_base_link(self, request, response):
        response.data = self.all_joint_solutions_msg
        return response

    def get_tcp_position(self):
        future = self.client_get_tcp.call_async(self.req_tcp)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error("Failed to get TCP position.")
            return

        response = future.result()
        self.tcp_points = response.data
        self.get_logger().info(f"TCP points received: {self.tcp_points}")


    # ---------------------------------------------------------------------
    # IK Computation
    # ---------------------------------------------------------------------

    def send_request(self):

        if not self.tcp_points:
            self.get_logger().warn("No TCP points received.")
            return

        self.all_joint_solutions_msg.clear()
        self.all_joint_solutions_deg.clear()
        json_points = []

        for i, pa in enumerate(self.tcp_points):

            x, y, z, R, P, Y = pa.vrednosti

            # Default RPY if zero
            if R == 0.0 and P == 0.0 and Y == 0.0:
                R, P, Y = 180.0, 0.0, 0.0
                self.get_logger().info(f"Point {i}: defaulting to tool-down RPY.")

            roll = math.radians(R)
            pitch = math.radians(P)
            yaw = math.radians(Y)

            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

            # Build IK request
            req = GetPositionIK.Request()
            req.ik_request.group_name = "cr5_group"
            req.ik_request.ik_link_name = "Link6"
            req.ik_request.robot_state.is_diff = False

            req.ik_request.pose_stamped.header.frame_id = "base_link"
            req.ik_request.pose_stamped.pose.position.x = x
            req.ik_request.pose_stamped.pose.position.y = y
            req.ik_request.pose_stamped.pose.position.z = z

            req.ik_request.pose_stamped.pose.orientation.x = qx
            req.ik_request.pose_stamped.pose.orientation.y = qy
            req.ik_request.pose_stamped.pose.orientation.z = qz
            req.ik_request.pose_stamped.pose.orientation.w = qw

            req.ik_request.timeout.sec = 2
            req.ik_request.avoid_collisions = False

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if not future.result():
                self.get_logger().error(f"IK request failed for point {i}")
                continue

            result = future.result()

            if result.error_code.val != 1:
                self.get_logger().warn(f"IK failed for point {i}, error_code={result.error_code.val}")
                continue

            joints_rad = list(result.solution.joint_state.position)

            if len(joints_rad) < 6:
                self.get_logger().error("IK returned fewer than 6 joints!")
                continue

            # ----------------------------------------------------------
            # 🔥 FIX 1 — First limit enforcement
            # ----------------------------------------------------------
            clamped_rad = self.normalize_all(joints_rad[:6])

            # ----------------------------------------------------------
            # 🔥 FIX 2 — Smooth joint trajectory
            # ----------------------------------------------------------
            adjusted_rad = self.adjust_trajectory(clamped_rad)

            # ----------------------------------------------------------
            # 🔥 FIX 3 — FINAL HARD CLAMP (absolutely required!)
            # ----------------------------------------------------------
            final_rad = self.normalize_all(adjusted_rad)

            # Warn if solver exceeded limits
            for j in range(6):
                if abs(adjusted_rad[j] - final_rad[j]) > 1e-4:
                    self.get_logger().warn(
                        f"Joint {j+1} IK angle {math.degrees(adjusted_rad[j]):.2f}° "
                        f"clamped to {math.degrees(final_rad[j]):.2f}°"
                    )

            # Convert to degrees for JSON + msgs
            final_deg = [math.degrees(a) for a in final_rad]

            pa_out = PointArray()
            pa_out.vrednosti = final_deg
            self.all_joint_solutions_msg.append(pa_out)
            self.all_joint_solutions_deg.append(final_deg)
            json_points.append(final_deg)


        # Save JSON
        file_path = ""

        data = {
            "trajectory_name": "ik_trajectory",
            "metadata": {
                "author": "uporabnik",
                "created": datetime.now().strftime("%Y-%m-%d"),
                "description": "IK solved + normalized (degrees)"
            },
            "points": json_points
        }

        with open(file_path, "w") as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"JSON saved to {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = IKClient()
    node.get_tcp_position()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
