import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_matrix,
    quaternion_multiply,
)

from dobot_msgs_v4.srv import SetFloat
from dobot_msgs_v4.msg import PointArray


# -----------------------------
# CONFIG (set these correctly!)
# -----------------------------
TARGET_FRAME = "base_link"

# IMPORTANT: set this to the actual frame your points are expressed in.
# If you are currently using an optical frame, change this to the NON-optical body frame.
# Examples (RealSense typical):
#   "camera_link" or "camera_depth_frame" (body-ish)
#   "camera_color_optical_frame" (optical)
SOURCE_FRAME = "kamera"

# If incoming rx,ry,rz are DEGREES -> True
# If incoming rx,ry,rz are RADIANS -> False
INPUT_RPY_DEG = True

# If you want output rx,ry,rz in DEGREES -> True
# If you want output rx,ry,rz in RADIANS -> False
OUTPUT_RPY_DEG = True


class TfLookupExample(Node):
    def __init__(self):
        super().__init__("tf_lookup_example")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tcp_points = []
        self.tcp_points_transformed = []
        self.has_transformed = False

        # Timer checks TF and data
        self.timer = self.create_timer(0.5, self.lookup_once)

        # Service client to read TCP points
        self.client_tcp_points = self.create_client(SetFloat, "/tocke_kamera")
        self.get_logger().info("Čakam na strežnik storitve '/tocke_kamera'...")
        while not self.client_tcp_points.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service ni na voljo...")

        future = self.client_tcp_points.call_async(SetFloat.Request())
        future.add_done_callback(self.tcp_points_response)

        # Service server to send transformed points
        self.server_poslji_tcp_points = self.create_service(
            SetFloat,
            "/poslji_transformirane_tcp_tocke",
            self.poslji_tcp_points
        )

    def poslji_tcp_points(self, request, response):
        # IMPORTANT: response.data must be a list of PointArray
        response.data = []
        for point in self.tcp_points_transformed:
            pa = PointArray()
            pa.vrednosti = [float(v) for v in point]
            response.data.append(pa)
        return response

    def tcp_points_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Klic storitve ni uspel: {e}")
            return

        self.tcp_points = [list(pa.vrednosti) for pa in response.data]
        self.get_logger().info(f"Prejete TCP točke (N={len(self.tcp_points)}): {self.tcp_points[:1]} ...")

    def _deg2rad(self, a):
        return a * np.pi / 180.0

    def _rad2deg(self, a):
        return a * 180.0 / np.pi

    def lookup_once(self):
        if not self.tcp_points or self.has_transformed:
            return

        # Get transform TARGET <- SOURCE
        try:
            trans = self.tf_buffer.lookup_transform(
                TARGET_FRAME,
                SOURCE_FRAME,
                Time()
            )
        except Exception:
            self.get_logger().warn("TF še ni na voljo, čakam...")
            return

        t = trans.transform.translation
        r = trans.transform.rotation

        # Build homogeneous transform matrix
        T = quaternion_matrix([r.x, r.y, r.z, r.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        # Rotation quaternion of TF (SOURCE->TARGET)
        q_tf = np.array([r.x, r.y, r.z, r.w], dtype=float)

        self.get_logger().info(f"Using TF: {TARGET_FRAME} <- {SOURCE_FRAME}")
        self.get_logger().info(f"Transformation matrix:\n{T}")

        out = []

        for point in self.tcp_points:
            x, y, z, rx, ry, rz = point

            # ---- position ----
            p = np.array([x, y, z, 1.0], dtype=float)
            p_t = T.dot(p)

            # ---- orientation ----
            # convert input rpy -> quaternion (in SOURCE frame)
            if INPUT_RPY_DEG:
                roll = self._deg2rad(rx)
                pitch = self._deg2rad(ry)
                yaw = self._deg2rad(rz)
            else:
                roll, pitch, yaw = float(rx), float(ry), float(rz)

            q_in = np.array(quaternion_from_euler(roll, pitch, yaw), dtype=float)  # (x,y,z,w)

            # rotate orientation into TARGET: q_out = q_tf ⊗ q_in
            # (because TF is TARGET <- SOURCE)
            q_out = quaternion_multiply(q_tf, q_in)

            # back to euler
            roll2, pitch2, yaw2 = euler_from_quaternion(q_out)

            if OUTPUT_RPY_DEG:
                rx2 = float(self._rad2deg(roll2))
                ry2 = float(self._rad2deg(pitch2))
                rz2 = float(self._rad2deg(yaw2))
            else:
                rx2, ry2, rz2 = float(roll2), float(pitch2), float(yaw2)

            out.append([
                float(p_t[0]),
                float(p_t[1]),
                float(p_t[2]),
                rx2, ry2, rz2
            ])

        self.tcp_points_transformed = out
        self.get_logger().info(f"Transformed TCP points (first): {self.tcp_points_transformed[:1]} ...")

        self.has_transformed = True
        self.timer.cancel()
        self.get_logger().info("Transformacija izvedena. Timer ustavljen.")


def main(args=None):
    rclpy.init(args=args)
    node = TfLookupExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
