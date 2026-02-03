import json
import rclpy
from rclpy.node import Node 

# Uvozi definiran service in message
from dobot_msgs_v4.srv import SetFloat
from dobot_msgs_v4.msg import PointArray


class ReadFromJson(Node):

    def __init__(self):
        super().__init__('read_from_json')

        # Ustvari service
        self.srv = self.create_service(
            SetFloat,          # service tip (v response ima PointArray[])
            '/nalozi_tocke',    # ime servisa
            self.load_points   # callback
        )

    def load_points(self, request, response):
        # Pot do JSON datoteke
        #file_path = "/home/lab3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/final_pkg/config/trajektorija_1.json"
        file_path = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/simulacija/config/tocke.json"

        # Odpri in preberi datoteko
        with open(file_path, "r") as f:
            data = json.load(f)

        # Preberi samo "points" (2D seznam)
        points = data.get("points", [])
        self.get_logger().info(f"Prebrane točke: {points}")
        self.get_logger().info(f"Tip podatkov: {type(points)}")

        # Napolni response s PointArray objekti
        response.data = []  # seznam PointArray
        for i, point in enumerate(points, start=1):
            pa = PointArray()
            pa.vrednosti = [float(x) for x in point]  # vsaka vrstica je float64[]
            response.data.append(pa)



            # self.get_logger().info(f"Točka {i}: {pa.vrednosti}")
            # self.get_logger().info(f"Tip točke {i}: {type(response.data)}")
        #self.get_logger().info(f"Prikaz:{response.data}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ReadFromJson()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
