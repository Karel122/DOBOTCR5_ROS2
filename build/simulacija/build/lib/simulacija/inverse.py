#!/usr/bin/env python3
"""
safe_ik_full.py (ROS 2 Humble compatible)

- Receives TCP points (x,y,z,R,P,Y) via /poslji_transformirane_tcp_tocke
  (xyz in meters, RPY IN RADIANS)
- For each TCP pose:
   - tries multiple IK branches by varying seeds
   - validates each candidate with MoveIt /check_state_validity
     (joint limits + self-collision + environment collision)
   - validates interpolated joint-space segment prev->candidate (sampling)
   - picks the smoothest valid candidate (closest to previous)
- Adds a floor collision object: plane z = 0 in FRAME_ID.
- Allows collisions between floor and base_link (and optionally Link1) via ACM.
- Saves joint trajectory (deg) + optional inverse dynamics (tau) to JSON
"""

import math
import json
from datetime import datetime
import tempfile

import numpy as np
import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetPositionIK, GetStateValidity
from tf_transformations import quaternion_from_euler

from dobot_msgs_v4.srv import SetFloat
from dobot_msgs_v4.msg import PointArray

from rcl_interfaces.srv import GetParameters

# Planning scene collision objects + ACM
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, AllowedCollisionMatrix, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


# ----------------------------
# CONFIG
# ----------------------------

GROUP_NAME = "cr5_group"
IK_LINK_NAME = "Link6"
FRAME_ID = "base_link"

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

JOINT_LIMITS = {
    0: (-6.2657, 6.2657),       # ±359°
    1: (-6.2657, 6.2657),       # ±359°
    2: (-2.86234, 2.86234),     # ±164° (hard)
    3: (-6.2657, 6.2657),       # ±359°
    4: (-6.2657, 6.2657),       # ±359°
    5: (-6.2657, 6.2657)        # ±359°
}

IK_TIMEOUT_SEC = 2
AVOID_COLLISIONS_IN_IK = True

TRAJ_STEP_RAD = math.radians(2.0)

HOME_SEED = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
EXTRA_SEEDS = [
    [0.0, -1.2,  1.2, 0.0,  1.57, 0.0],
    [0.0,  1.2, -1.2, 0.0, -1.57, 0.0],
    [0.0, -1.2,  1.2, 0.0, -1.57, 0.0],
    [0.0,  1.2, -1.2, 0.0,  1.57, 0.0],
]

MAX_CANDIDATES_PER_POINT = 25
JITTER_SEEDS_N = 12
JITTER_SIGMA = 0.35

DT = 0.1

OUTPUT_JSON_PATH = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/simulacija/config/tocke.json"

DEBUG_IK = True
DEBUG_SEGMENT = False

# NEW: links allowed to be in contact with the floor
FLOOR_ID = "floor_z0"
ALLOW_FLOOR_CONTACT_LINKS = ["base_link", "Link1"]  # add/remove as needed


class SafeIKFull(Node):
    def __init__(self):
        super().__init__("safe_ik_full")

        # Clients
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.tcp_cli = self.create_client(SetFloat, "/poslji_transformirane_tcp_tocke")
        self.apply_scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        # Service server to provide joint points to another node
        self.server = self.create_service(SetFloat, "/poslji_tcp_tocke_base_link", self.send_points_base_link)

        while not self.ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /compute_ik ...")
        while not self.validity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /check_state_validity ...")
        while not self.tcp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /poslji_transformirane_tcp_tocke ...")
        while not self.apply_scene_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /apply_planning_scene ...")

        # Add floor collision once (plane z=0) + ACM allowing base_link contact
        self.add_floor_collision_with_acm(z0=0.0)

        self.req_tcp = SetFloat.Request()
        self.tcp_points = []

        self.qs_rad = []
        self.qs_deg = []
        self.msg_points = []
        self.prev_q = None

        # Pinocchio optional
        self.pin = None
        self.pin_model = None
        self.pin_data = None
        self._try_init_pinocchio_from_robot_description()

    # -----------------------
    # ROS service response
    # -----------------------
    def send_points_base_link(self, request, response):
        response.data = self.msg_points
        return response

    # -----------------------
    # Floor collision + ACM
    # -----------------------
    def add_floor_collision_with_acm(self, z0=0.0):
        """
        Adds a large thin box whose top surface is exactly at z=z0 in FRAME_ID.
        Also updates AllowedCollisionMatrix so base_link (and selected links)
        are allowed to contact the floor without invalidating the state.
        """
        size_x = 10.0
        size_y = 10.0
        thickness = 0.02
        center_z = float(z0 - thickness / 2.0)

        # --- Collision object (floor) ---
        co = CollisionObject()
        co.header.frame_id = FRAME_ID
        co.id = FLOOR_ID
        co.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(size_x), float(size_y), float(thickness)]

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = center_z
        pose.orientation.w = 1.0

        co.primitives.append(prim)
        co.primitive_poses.append(pose)

        # --- Allowed Collision Matrix (ACM) ---
        # We create a *diff* ACM that introduces allowances for:
        #   floor_z0 <-> base_link  (and optional additional links)
        names = [FLOOR_ID] + list(ALLOW_FLOOR_CONTACT_LINKS)

        acm = AllowedCollisionMatrix()
        acm.entry_names = names

        N = len(names)
        matrix = [[False] * N for _ in range(N)]
        idx = {name: i for i, name in enumerate(names)}

        def allow(a, b):
            ia, ib = idx[a], idx[b]
            matrix[ia][ib] = True
            matrix[ib][ia] = True

        for link in ALLOW_FLOOR_CONTACT_LINKS:
            allow(FLOOR_ID, link)

        acm.entry_values = []
        for i in range(N):
            e = AllowedCollisionEntry()
            e.enabled = [bool(x) for x in matrix[i]]
            acm.entry_values.append(e)

        # --- Apply PlanningScene diff ---
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        scene.allowed_collision_matrix = acm

        req = ApplyPlanningScene.Request()
        req.scene = scene

        fut = self.apply_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

        if not fut.result() or not fut.result().success:
            self.get_logger().warn("Failed to apply planning scene (floor + ACM).")
            return

        self.get_logger().info(
            f"Added floor collision at z={z0} (id='{FLOOR_ID}') and allowed contact for links: {ALLOW_FLOOR_CONTACT_LINKS}"
        )

    # -----------------------
    # Utility helpers
    # -----------------------
    def wrap_angle_pi(self, a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    def within_limits(self, q):
        for j in range(6):
            jmin, jmax = JOINT_LIMITS[j]
            if q[j] < jmin or q[j] > jmax:
                return False, j
        return True, None

    # Input R,P,Y are in RADIANS
    def rpy_rad_to_quat(self, R, P, Y):
        qx, qy, qz, qw = quaternion_from_euler(R, P, Y)  # axes='sxyz'
        return float(qx), float(qy), float(qz), float(qw)

    def contacts_summary(self, contacts, max_items=10):
        if contacts is None:
            return []
        if hasattr(contacts, "keys"):
            return list(contacts.keys())[:max_items]

        out = []
        try:
            for c in contacts[:max_items]:
                b1 = getattr(c, "contact_body_1", None)
                b2 = getattr(c, "contact_body_2", None)
                if b1 is not None and b2 is not None:
                    out.append(f"{b1} <-> {b2}")
                else:
                    out.append(str(c))
        except Exception:
            out = [str(contacts)]
        return out

    def fmt_q(self, q, deg=False, nd=2):
        if q is None:
            return "None"
        if deg:
            return "[" + ", ".join(f"{math.degrees(v):.{nd}f}" for v in q) + "]"
        return "[" + ", ".join(f"{v:.{nd}f}" for v in q) + "]"

    # -----------------------
    # MoveIt checks
    # -----------------------
    def is_state_valid(self, q_rad):
        req = GetStateValidity.Request()
        req.group_name = GROUP_NAME
        req.robot_state.joint_state.name = JOINT_NAMES
        req.robot_state.joint_state.position = [float(x) for x in q_rad]

        fut = self.validity_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result():
            return False, []
        res = fut.result()
        return bool(res.valid), res.contacts

    def call_compute_ik(self, x, y, z, qx, qy, qz, qw, seed):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = IK_LINK_NAME

        req.ik_request.pose_stamped.header.frame_id = FRAME_ID
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        req.ik_request.pose_stamped.pose.orientation.x = float(qx)
        req.ik_request.pose_stamped.pose.orientation.y = float(qy)
        req.ik_request.pose_stamped.pose.orientation.z = float(qz)
        req.ik_request.pose_stamped.pose.orientation.w = float(qw)

        req.ik_request.timeout.sec = IK_TIMEOUT_SEC
        req.ik_request.avoid_collisions = bool(AVOID_COLLISIONS_IN_IK)

        req.ik_request.robot_state.is_diff = True
        req.ik_request.robot_state.joint_state.name = JOINT_NAMES
        req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]

        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result():
            return None

        res = fut.result()
        if res.error_code.val != 1:
            return None

        q = list(res.solution.joint_state.position)[:6]
        q = [float(v) for v in q]
        q = [self.wrap_angle_pi(v) for v in q]
        return q

    # -----------------------
    # Seed generation
    # -----------------------
    def generate_jitter_seeds(self, base_seed, n=12, sigma=0.35):
        seeds = []
        base = np.array(base_seed, dtype=float)
        for _ in range(n):
            s = base + np.random.normal(0.0, sigma, size=6)
            s = np.array([self.wrap_angle_pi(x) for x in s])
            seeds.append(s.tolist())
        return seeds

    def add_branch_flip_seeds(self, q):
        flips = []
        if q is None:
            return flips
        q = np.array(q, dtype=float)

        # Wrist flip (common heuristic)
        q1 = q.copy()
        q1[4] = self.wrap_angle_pi(q1[4] + math.pi)
        q1[5] = self.wrap_angle_pi(q1[5] + math.pi)
        flips.append(q1.tolist())

        # Rough elbow flip heuristic
        q2 = q.copy()
        q2[1] = self.wrap_angle_pi(q2[1] + math.pi)
        q2[2] = self.wrap_angle_pi(q2[2] - math.pi)
        flips.append(q2.tolist())

        return flips

    def generate_seeds(self):
        seeds = []
        if self.prev_q is not None:
            seeds.append(self.prev_q)
            seeds.extend(self.add_branch_flip_seeds(self.prev_q))
            seeds.extend(self.generate_jitter_seeds(self.prev_q, n=JITTER_SEEDS_N, sigma=JITTER_SIGMA))

        seeds.append(HOME_SEED)
        seeds.extend(EXTRA_SEEDS)
        seeds.extend(self.generate_jitter_seeds(HOME_SEED, n=6, sigma=0.6))
        return seeds

    def q_distance(self, q1, q2):
        a = np.array(q1, dtype=float)
        b = np.array(q2, dtype=float)
        return float(np.linalg.norm(a - b))

    # -----------------------
    # Trajectory segment sampling collision check
    # -----------------------
    def _interp_segment(self, q0, q1, step_rad=TRAJ_STEP_RAD):
        q0 = np.array(q0, dtype=float)
        q1 = np.array(q1, dtype=float)
        max_delta = float(np.max(np.abs(q1 - q0)))
        n = max(2, int(math.ceil(max_delta / step_rad)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            yield (1.0 - t) * q0 + t * q1

    def segment_is_valid(self, q0, q1, step_rad=TRAJ_STEP_RAD):
        for k, q in enumerate(self._interp_segment(q0, q1, step_rad)):
            valid, contacts = self.is_state_valid(q)
            if not valid:
                info = {
                    "sample": k,
                    "q": [float(x) for x in q],
                    "contacts": self.contacts_summary(contacts, max_items=10),
                }
                if DEBUG_SEGMENT:
                    self.get_logger().warn(f"Segment collision sample {k}: contacts={info['contacts']}")
                return False, info
        return True, {"ok": True}

    # -----------------------
    # Core: find best valid IK
    # -----------------------
    def find_best_valid_ik(self, x, y, z, R, P, Y, point_index=None):
        qx, qy, qz, qw = self.rpy_rad_to_quat(R, P, Y)
        seeds = self.generate_seeds()
        candidates = []

        if DEBUG_IK:
            self.get_logger().info(
                f"[Point {point_index}] TCP: x={x:.4f}, y={y:.4f}, z={z:.4f}, "
                f"RPYrad=({R:.3f},{P:.3f},{Y:.3f}) | "
                f"RPYdeg=({math.degrees(R):.2f},{math.degrees(P):.2f},{math.degrees(Y):.2f}), "
                f"seeds={len(seeds)}"
            )

        for si, seed in enumerate(seeds):
            q = self.call_compute_ik(x, y, z, qx, qy, qz, qw, seed)
            if q is None:
                continue

            ok_limits, _ = self.within_limits(q)
            if not ok_limits:
                continue

            valid, _ = self.is_state_valid(q)
            if not valid:
                continue

            if self.prev_q is not None:
                seg_ok, _ = self.segment_is_valid(self.prev_q, q)
                if not seg_ok:
                    continue

            candidates.append(q)
            if len(candidates) >= MAX_CANDIDATES_PER_POINT:
                break

        if not candidates:
            return None

        if self.prev_q is None:
            return candidates[0]

        candidates.sort(key=lambda qq: self.q_distance(self.prev_q, qq))
        return candidates[0]

    # -----------------------
    # Receive TCP points
    # -----------------------
    def get_tcp_points(self):
        fut = self.tcp_cli.call_async(self.req_tcp)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result():
            self.get_logger().error("Failed to get TCP points.")
            return False

        res = fut.result()
        self.tcp_points = res.data
        self.get_logger().info(f"TCP points received: {self.tcp_points}")
        return True

    # -----------------------
    # Build safe trajectory
    # -----------------------
    def build_safe_trajectory(self):
        self.qs_rad.clear()
        self.qs_deg.clear()
        self.msg_points.clear()
        self.prev_q = None

        kept = 0
        skipped = 0

        for i, pa in enumerate(self.tcp_points):
            x, y, z, R, P, Y = pa.vrednosti

            # unit guard: if values look like mm, convert
            if abs(x) > 2.0 or abs(y) > 2.0 or abs(z) > 2.0:
                self.get_logger().warn(f"Point {i}: looks like mm -> converting to meters.")
                x, y, z = x / 1000.0, y / 1000.0, z / 1000.0

            q_best = self.find_best_valid_ik(x, y, z, R, P, Y, point_index=i)
            if q_best is None:
                self.get_logger().warn(f"Point {i}: no valid IK branch found. Skipping.")
                skipped += 1
                continue

            self.prev_q = q_best
            self.qs_rad.append(q_best)

            q_deg = [math.degrees(v) for v in q_best]
            self.qs_deg.append(q_deg)

            msg = PointArray()
            msg.vrednosti = q_deg
            self.msg_points.append(msg)

            kept += 1

        self.get_logger().info(f"Trajectory built: kept={kept}, skipped={skipped}")
        return kept > 0

    def verify_full_trajectory(self):
        if len(self.qs_rad) < 2:
            return True
        for i in range(len(self.qs_rad) - 1):
            ok, info = self.segment_is_valid(self.qs_rad[i], self.qs_rad[i + 1])
            if not ok:
                self.get_logger().warn(f"Trajectory invalid on segment {i}->{i+1}: {info}")
                return False
        return True

    # -----------------------
    # Pinocchio inverse dynamics (unchanged)
    # -----------------------
    def _get_robot_description_xml(self, target_node: str):
        srv_name = f"{target_node}/get_parameters" if target_node.startswith("/") else f"/{target_node}/get_parameters"
        cli = self.create_client(GetParameters, srv_name)
        if not cli.wait_for_service(timeout_sec=2.0):
            return None

        req = GetParameters.Request()
        req.names = ["robot_description"]

        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.result():
            return None

        values = fut.result().values
        if not values:
            return None

        xml = values[0].string_value
        return xml if xml else None

    def _try_init_pinocchio_from_robot_description(self):
        try:
            import pinocchio as pin
            self.pin = pin
        except Exception as e:
            self.get_logger().warn(f"pinocchio not available -> inverse dynamics disabled. ({e})")
            return

        urdf_xml = self._get_robot_description_xml("/move_group")
        if urdf_xml:
            self.get_logger().info("Loaded robot_description from /move_group")
        else:
            urdf_xml = self._get_robot_description_xml("/robot_state_publisher")
            if urdf_xml:
                self.get_logger().info("Loaded robot_description from /robot_state_publisher")

        if not urdf_xml:
            self.get_logger().warn("robot_description not found -> inverse dynamics disabled.")
            return

        try:
            with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
                f.write(urdf_xml)
                urdf_path = f.name

            self.pin_model = self.pin.buildModelFromUrdf(urdf_path)
            self.pin_data = self.pin_model.createData()
            self.get_logger().info(f"pinocchio loaded URDF from temp file: {urdf_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize pinocchio model -> inverse dynamics disabled. ({e})")
            self.pin_model = None
            self.pin_data = None

    def estimate_qd_qdd(self, qs, dt=DT):
        qs = np.array(qs, dtype=float)
        n = qs.shape[0]
        qd = np.zeros_like(qs)
        qdd = np.zeros_like(qs)

        if n >= 2:
            qd[1:] = (qs[1:] - qs[:-1]) / dt
            qd[0] = qd[1]
        if n >= 3:
            qdd[2:] = (qd[2:] - qd[1:-1]) / dt
            qdd[0] = qdd[2]
            qdd[1] = qdd[2]
        else:
            qdd[:] = 0.0

        return qd.tolist(), qdd.tolist()

    def compute_inverse_dynamics(self):
        if self.pin is None or self.pin_model is None or self.pin_data is None:
            return None

        qd, qdd = self.estimate_qd_qdd(self.qs_rad, dt=DT)
        taus = []

        for i in range(len(self.qs_rad)):
            q = np.array(self.qs_rad[i], dtype=float)
            v = np.array(qd[i], dtype=float)
            a = np.array(qdd[i], dtype=float)

            tau = self.pin.rnea(self.pin_model, self.pin_data, q, v, a)
            taus.append([float(x) for x in tau])

        return taus

    # -----------------------
    # Save JSON
    # -----------------------
    def save_json(self, taus=None):
        out = {
            "trajectory_name": "safe_ik_trajectory_with_floor_acm",
            "metadata": {
                "created": datetime.now().strftime("%Y-%m-%d"),
                "description": (
                    "TCP->IK with many seeds. Validated with MoveIt state validity "
                    "(limits + collisions incl. floor z=0). Floor contact is allowed for "
                    f"links {ALLOW_FLOOR_CONTACT_LINKS} via ACM."
                ),
                "group_name": GROUP_NAME,
                "ik_link_name": IK_LINK_NAME,
                "frame_id": FRAME_ID,
                "traj_sampling_step_deg": float(math.degrees(TRAJ_STEP_RAD)),
                "dt_assumed_s": DT,
                "floor_collision": {"frame": FRAME_ID, "z0": 0.0, "id": FLOOR_ID, "allowed_links": ALLOW_FLOOR_CONTACT_LINKS},
                "input_units": {"xyz": "meters", "rpy": "radians"},
            },
            "points": self.qs_deg,
            "tau_Nm": taus,
        }

        with open(OUTPUT_JSON_PATH, "w") as f:
            json.dump(out, f, indent=2)

        self.get_logger().info(f"JSON saved to {OUTPUT_JSON_PATH}")


def main(args=None):
    rclpy.init(args=args)
    node = SafeIKFull()

    if not node.get_tcp_points():
        node.destroy_node()
        rclpy.shutdown()
        return

    if not node.build_safe_trajectory():
        node.get_logger().error("No valid trajectory points. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.verify_full_trajectory()

    taus = node.compute_inverse_dynamics()
    if taus is None:
        node.get_logger().warn("Inverse dynamics not computed (pinocchio missing or model load failed).")
    else:
        node.get_logger().info("Inverse dynamics computed (tau).")

    node.save_json(taus=taus)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


