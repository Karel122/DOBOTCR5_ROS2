import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import CheckJson
import json
import os
import numpy as np
from sensor_msgs.msg import JointState
from dobot_msgs_v4.srv import SetFloat
import time
from dobot_msgs_v4.msg import PointArray



class MinimalService(Node):

    def __init__(self):
        super().__init__('preveri_json')


        self.subscription = self.create_subscription(
           JointState,
           '/joint_states_robot',
           self.listener_callback,
           10)
        

       # self.konce_vrednosti = []

        


        
        

        # self.srv = self.create_service(
        #     SetFloat,          # service tip (v response ima PointArray[])
        #     'nalozi_tocke',    # ime servisa -> na novo definiran service - še ni!!!!
        #     self.load_points   # callback
        # )

    

    
        self.srv = self.create_service(SetFloat, '/nalozi_tocke_node_first', self.check_json_callback)



        
       

        time.sleep(1)

        self.cli_json = self.create_client(SetFloat, '/nalozi_tocke')

        get_json_request = SetFloat.Request()
        future = self.cli_json.call_async(get_json_request)
        future.add_done_callback(self.after_json_callback)
        # self.msg.position = None

    def listener_callback(self, msg):
        self.msg = msg
        self.degs = [np.degrees(angle) for angle in msg.position]


    #nalozie_tocke_node_first
    def check_json_callback(self, request, response):
        
        response.data  = self.konce_vrednosti
        self.get_logger().info(f"response: {response.data}")
        return response

    

    def after_json_callback(self, future):

        if not hasattr(self, "degs"):
            self.get_logger().warn("JointState še ni prispel — čakam...")
            # počakamo 100ms in kličemo samega sebe ponovno
            time.sleep(0.1)
            self.after_json_callback(future)
            return


        response = future.result()
        self.waypoints = [list(pa.vrednosti) for pa in response.data]

        # 1) PREVERI first -> last
        first = np.array(np.array(self.degs), dtype=float)
        last = np.array(self.waypoints[-1], dtype=float)

        razlika = np.abs(first - last)
        self.get_logger().info(f"Razlike first-last: {razlika}")

        if np.all(razlika < 50):
            self.waypoints.insert(0, self.degs)
        else:
            max_diff = np.max(razlika)
            faktor = int(np.floor(max_diff / 50.0))
            self.get_logger().info(f"Največja razlika first-last = {max_diff:.2f}, dodali bomo {faktor} vmesnih točk.")

            new_points = []
            for i in range(1, faktor+1):
                alpha = i / (faktor+1)
                interpolated = (1 - alpha) * last + alpha * first
                new_points.append(interpolated.tolist())

            for p in reversed(new_points):
                self.waypoints.insert(0, p)

            #### 

            self.waypoints.insert(0, self.degs)

        # 2) PREVERI vse sosede v waypoints
        new_waypoints = []
        for i in range(len(self.waypoints)-1):
            current = np.array(self.waypoints[i], dtype=float)
            nxt = np.array(self.waypoints[i+1], dtype=float)

            new_waypoints.append(current.tolist())

            razlika = np.abs(nxt - current)
            max_diff = np.max(razlika)

            if max_diff > 50:
                faktor = int(np.floor(max_diff / 50.0))
                self.get_logger().info(
                    f"Razlika med točko {i} in {i+1} = {max_diff:.2f}, dodali bomo {faktor} vmesnih točk."
                )

                for j in range(1, faktor+1):
                    alpha = j / (faktor+1)
                    interpolated = (1 - alpha) * current + alpha * nxt
                    new_waypoints.append(interpolated.tolist())

        new_waypoints.append(self.waypoints[-1])
        self.waypoints = new_waypoints

        # pripravi response
        response.data = []
        for point in self.waypoints:
            pa = PointArray()
            pa.vrednosti = [float(x) for x in point]
            response.data.append(pa)

        self.konce_vrednosti = response.data
        self.get_logger().info(f"Končno število točk: {len(self.waypoints)}")
        return response



def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
