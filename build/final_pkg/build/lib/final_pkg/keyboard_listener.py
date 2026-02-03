import rclpy
from rclpy.node import Node
import sys
import termios
import tty
import threading
from sensor_msgs.msg import JointState
from dobot_msgs_v4.srv import StopDrag, StartDrag
import json
import os
from datetime import date
import math
import numpy as np


def get_key():
    """Prebere en znak s tipkovnice (brez Enter)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class KeyboardListener(Node):

    def __init__(self):
        super().__init__('keyboard_listener')

        self.cli_start = self.create_client(StartDrag, '/dobot_bringup_ros2/srv/StartDrag')
        self.cli_stop = self.create_client(StopDrag, '/dobot_bringup_ros2/srv/StopDrag')

        start_drag = StartDrag.Request()
        future = self.cli_start.call_async(start_drag)



        # Subscribaj na joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states_robot',
            self.listener_callback,
            10)

        # Shranimo zadnje prejeto sporočilo
        self.last_point = None
        self.last_point_space = None

        # JSON datoteka
        self.file_path = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/final_pkg/config/trajektorija_posnetek.json"

        # Če datoteka še ne obstaja, jo inicializiraj s strukturo
        if not os.path.exists(self.file_path):
            data = {
                "trajectory_name": "primer_poti",
                "metadata": {
                    "author": "uporabnik",
                    "created": str(date.today()),   # današnji datum
                    "description": "Testna pot za robota"
                },
                "points": []
            }
            with open(self.file_path, "w") as f:
                json.dump(data, f, indent=2, separators=(',', ': '))

        # Zaženemo tipkovnični thread
        self.running = True
        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()

    def listener_callback(self, msg):
        # Shranimo zadnjo točko
        self.last_point_space = list(msg.position)

    def keyboard_loop(self):
        self.get_logger().info("Pritisni SPACE za shranjevanje (q za izhod)")

        while self.running:
            key = get_key()
            if key == " ":
                self.space_pressed()
            elif key == "q":
                stop_drag = StopDrag.Request()
                future = self.cli_stop.call_async(stop_drag)
                self.get_logger().info("Prekinjam node...")
                rclpy.shutdown()
                break

    def space_pressed(self):
        if self.last_point_space is None:
            self.get_logger().warn("Ni podatkov za shranjevanje (še ni prišlo sporočilo)!")
            return

        # Preberi JSON datoteko ali jo inicializiraj
        if not os.path.exists(self.file_path) or os.path.getsize(self.file_path) == 0:
            data = {
                "trajectory_name": "primer_poti",
                "metadata": {
                    "author": "uporabnik",
                    "created": str(date.today()),
                    "description": "Testna pot za robota"
                },
                "points": []
            }
        else:
            with open(self.file_path, "r") as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    data = {
                        "trajectory_name": "primer_poti",
                        "metadata": {
                            "author": "uporabnik",
                            "created": str(date.today()),
                            "description": "Testna pot za robota"
                        },
                        "points": []
                    }

        # Pretvori radiane v stopinje
        point_degrees = [math.degrees(x) for x in self.last_point_space]
        
        # Dodaj novo točko (1D float list) v skupni seznam
        self.get_logger().info(f"Zadnja točka (v radianih): {self.last_point}")

        if self.last_point is not None:
            point_degrees = np.array(point_degrees)
            

            zasuk = point_degrees - self.last_point

            faktor = (max(abs(zasuk)) / 50)
            self.get_logger().info(f"Faktor: {faktor}")
            if int(np.floor(faktor)) > 0:
                for i in range(int(np.floor(faktor))):
                    point_degrees_copy = self.last_point + zasuk / faktor * (i + 1)
                    self.get_logger().info(f"Vmesna točka {i}")
                    data["points"].append(list(point_degrees_copy))
        #else:
           # data["points"].append([0.0]*6)    

        self.last_point= point_degrees
        data["points"].append(list(point_degrees))

        # Shrani nazaj v datoteko
        with open(self.file_path, "w") as f:
            json.dump(data, f, indent=2, separators=(',', ': '))

        self.get_logger().info(f"Točka (v stopinjah) shranjena v {self.file_path}: {point_degrees}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
