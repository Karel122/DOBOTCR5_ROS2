import rclpy
from rclpy.node import Node
import json
import numpy as np
from sensor_msgs.msg import JointState
from dobot_msgs_v4.srv import SetFloat
import time
from dobot_msgs_v4.msg import PointArray


class MinimalService(Node):

    def __init__(self):
        super().__init__('preveri_json')
        self.degs_ready = False  # počakamo na prve joint state podatke

        # ---- SUBSCRIBE NA REALNE JOINT STATE ----
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states_robot',
            self.listener_callback,
            10)



        # ---- SERVICE ZA VRNITEV KONČNIH TOČK ----
        self.srv = self.create_service(
            SetFloat,
            '/nalozi_tocke',
            self.check_json_callback)

        # ---- PREBERI JSON TAKOJ OB ZAGONU ----
        file_path = "/home/lab3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/final_pkg/config/ik_trajectory.json"

        with open(file_path, "r") as f:
            data = json.load(f)

        # JSON vsebuje seznam točk
        self.waypoints_raw = data["points"]
        #print(self.waypoints_raw)
        self.get_logger().info(f"Naloženih {len(self.waypoints_raw)} točk iz JSON.")

        # ---- POČAKAJ NA PRVE JOINT STATE ----
        while not self.degs_ready:
            self.get_logger().warn("Čakam na /joint_states_robot ...")
            rclpy.spin_once(self, timeout_sec=0.1)

        # ---- ZAŽENI OBDELAVO ----
        self.process_waypoints()


    def listener_callback(self, msg):
        self.degs = [np.degrees(angle) for angle in msg.position]
        self.degs_ready = True


    def process_waypoints(self):
        
        # pretvori v numpy
        first = np.array(self.degs, dtype=float)       # trenutni realni sklepi robota
        #last = np.array(self.waypoints_raw[-1], dtype=float)       # zadnji na seznamu self.waypoints_raw
        last = np.array(self.waypoints_raw[0], dtype=float)
        razlika = np.abs(first - last)

        #self.get_logger().info(f"Razlike first-last: {razlika}")

        waypoints = self.waypoints_raw.copy()

        # ---- KORAK 1: PREVERI START -> PRVA JSON TOČKA ----
        final_waypoints = []
        if np.all(razlika < 50):
            #waypoints.insert(0, self.degs)
            pass
        else:
            max_diff = np.max(razlika)
            faktor = int(np.floor(max_diff / 50.0))

            new_points = []
            for i in range(1, faktor + 1):
                alpha = i / (faktor + 1)
                interpolated = (1 - alpha) * last + alpha * first
                new_points.append(interpolated.tolist())

            for p in new_points:
                final_waypoints.insert(0, p)

            final_waypoints.insert(0, self.degs)
        #print(final_waypoints)
        # ---- KORAK 2: PREVERI VSE SOSSEDNJE PAR --- 
        
        for i in range(len(waypoints) - 1):
            current = np.array(waypoints[i], dtype=float)
            next = np.array(waypoints[i + 1], dtype=float)

            final_waypoints.append(current.tolist())

            razlika = np.abs(next - current)
            max_diff = np.max(razlika)

            if max_diff > 50:
                faktor = int(np.floor(max_diff / 50.0))
                self.get_logger().info(
                    f"Razlika med {i} in {i+1} = {max_diff:.2f}, dodajam {faktor} vmesnih točk.")

                for j in range(1, faktor + 1):
                    alpha = j / (faktor + 1)
                    interpolated = (1 - alpha) * current + alpha * next
                    final_waypoints.append(interpolated.tolist())

        final_waypoints.append(waypoints[-1])
        self.waypoints = final_waypoints
        print(f"tocke:{self.waypoints}")

        # ---- PRETVORI V PointArray ZA SERVICE ----
        self.konce_vrednosti = []
        for wp in self.waypoints:
            pa = PointArray()
            pa.vrednosti = [float(x) for x in wp]
            self.konce_vrednosti.append(pa)
        #self.get_logger().info(f"konce_vrednosti{self.konce_vrednosti}")
        
        #self.get_logger().info(f"Končno število točk: {len(self.waypoints)}")


    def check_json_callback(self, request, response):
        #self.get_logger().info(f"poslane točke: {self.konce_vrednosti}")
        response.data = self.konce_vrednosti
        #self.get_logger().info(f"Poslal {len(response.data)} točk.")

        self.get_logger().info(f"poslane točke: {response.data}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
