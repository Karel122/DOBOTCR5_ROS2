import rclpy
from rclpy.node import Node
import json
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import DynamicJointState


def shortest_angle_deg(q_current: float, q_target: float) -> float:
    """
    Vrne cilj, ki predstavlja isti fizikalni položaj (mod 360),
    vendar z najmanjšim zasukom glede na q_current.
    """
    delta = q_target - q_current
    delta = (delta + 180.0) % 360.0 - 180.0   # delta v (-180, 180]
    return q_current + delta


def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


def deg2rad(x: float) -> float:
    return x * math.pi / 180.0


class JointTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('node')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/cr5_group_controller/joint_trajectory',
            10
        )

        # --- SUBSCRIBE: /dynamic_joint_states (pravi feedback) ---
        # hranimo v STOPINJAH, ker shortest_angle_deg dela v stopinjah
        self.current_positions_deg = {}   # dict: joint_name -> position(deg)
        self.got_state = False

        self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_js_cb,
            10
        )

        # --- PREBERI JSON TOČKE (pričakujemo stopinje) ---
        file_path = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/final_pkg/config/trajektorija_posnetek.json"
        with open(file_path, "r") as f:
            data = json.load(f)

        self.points = data.get("points", [])
        if not self.points:
            raise RuntimeError("tocke.json nima 'points' ali je prazen.")

        self.get_logger().info(f"Prebranih točk: {len(self.points)}")

        # Joint order, ki ga boš pošiljal kontrolerju
        self.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']

        # indeks naslednje točke iz JSON
        self.idx = 0

        # publish 1 point per tick
        self.timer = self.create_timer(1.0, self.publish_next_point)

    def dynamic_js_cb(self, msg: DynamicJointState):
        """
        Prebere position iz interface_values.
        V ROS2 je position skoraj vedno v RADIANIH -> pretvorimo v stopinje.
        """
        current_deg = {}

        for name, iface in zip(msg.joint_names, msg.interface_values):
            if "position" in iface.interface_names:
                pos_idx = iface.interface_names.index("position")
                pos_rad = float(iface.values[pos_idx])
                current_deg[name] = rad2deg(pos_rad)

        if current_deg:
            self.current_positions_deg = current_deg
            self.got_state = True

    def publish_next_point(self):
        if not self.got_state:
            self.get_logger().warn("Še nimam /dynamic_joint_states. Ne pošiljam trajektorije.")
            return

        if self.idx >= len(self.points):
            self.get_logger().info("Vse točke poslane. Ustavljam timer.")
            self.timer.cancel()
            return

        # JSON: stopinje
        target_raw_deg = [float(x) for x in self.points[self.idx]]

        # --- shortest path v STOPINJAH ---
        target_opt_deg = []
        missing = []
        for j_name, q_tgt_deg in zip(self.joint_names, target_raw_deg):
            if j_name not in self.current_positions_deg:
                missing.append(j_name)
                continue
            q_cur_deg = self.current_positions_deg[j_name]
            target_opt_deg.append(shortest_angle_deg(q_cur_deg, q_tgt_deg))

        if missing:
            self.get_logger().error(f"Manjkajo jointi v /dynamic_joint_states: {missing}. Ne pošiljam.")
            return

        # --- pretvori v RADIANE tik pred pošiljanjem ---
        target_opt_rad = [deg2rad(x) for x in target_opt_deg]

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_opt_rad
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Poslana točka {self.idx+1}/{len(self.points)} | raw_deg={target_raw_deg} | opt_deg={target_opt_deg} | opt_rad={target_opt_rad}"
        )

        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
