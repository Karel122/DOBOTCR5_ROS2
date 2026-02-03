#!/usr/bin/env python3
import rclpy
import json
import numpy as np
from rclpy.node import Node
from dobot_msgs_v4.srv import ServoJ, GetAngle


class ServoJClient(Node):
    def __init__(self):
        super().__init__('skupni_node')

        # ---- 1) Ustvari kliente ----
        self.cli_move = self.create_client(ServoJ, '/dobot_bringup_ros2/srv/ServoJ')
        self.cli_get = self.create_client(GetAngle, '/dobot_bringup_ros2/srv/GetAngle')

        while not self.cli_get.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Čakam na GetAngle service...')
        while not self.cli_move.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Čakam na ServoJ service...')

        # ---- 2) Naloži JSON waypoints (NIČ VEČ SERVICE!) ----
        self.waypoints = self.load_waypoints_from_json(
            "/home/lab3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/final_pkg/config/trajektorija_posnetek.json"
        )

        self.get_logger().info(f"Naloženi waypoints iz JSON: {self.waypoints}")

        # ---- 3) Nastavitve ----
        self.index = 0
        self.max_hitrost = 20.0
        self.timer = self.create_timer(3.0, self.run_step)


    # =============================
    #  FUNKCIJA ZA BRANJE JSON-a
    # =============================
    def load_waypoints_from_json(self, json_path):
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                return data["points"]    # pričakuješ 2D tabelo
        except Exception as e:
            self.get_logger().error(f"Napaka pri branju JSON datoteke: {e}")
            return []


    # =============================
    #  PERIODIČNI KORAK
    # =============================
    def run_step(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info('Trajektorija je zaključena.')
            self.timer.cancel()
            return

        get_request = GetAngle.Request()
        future = self.cli_get.call_async(get_request)
        future.add_done_callback(self.after_get)


    # =============================
    #  CALLBACK PO GETAngle
    # =============================
    def after_get(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        # pretvori response.robot_return -> list(float)
        nova = response.robot_return.strip('{}').split(',')
        self.nova_pozicija = [float(x) for x in nova]

        # ciljna pozicija
        goal = self.waypoints[self.index]

        # izračun razlike
        razlika = np.array(goal) - np.array(self.nova_pozicija)
        max_zasuk = np.max(np.abs(razlika))
        cas_rotacije = round(max_zasuk / self.max_hitrost, 2)

        self.get_logger().info(f"Max zasuk: {max_zasuk}")
        self.get_logger().info(f"Čas zasuka: {cas_rotacije} s")

        # če prevelik gib za 3s → preskoči točko
        if cas_rotacije > 3.0:
            self.get_logger().info("Prevelik gib – preskakujem točko.")
            self.index += 1
            return

        # ----  ServoJ pošlji ----
        move_request = ServoJ.Request()
        move_request.a, move_request.b, move_request.c, move_request.d, move_request.e, move_request.f = goal

        move_request.param_value = [f"t={cas_rotacije}"]

        self.cli_move.call_async(move_request)
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = ServoJClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
