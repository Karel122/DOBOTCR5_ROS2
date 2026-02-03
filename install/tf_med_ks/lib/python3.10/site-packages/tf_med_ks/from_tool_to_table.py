#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import Transform, SetStaticTF
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage 
from tf2_ros import Buffer, TransformListener
import numpy as np
from rclpy.time import Time
from tf_transformations import quaternion_from_euler, quaternion_matrix



class TransformServer(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.last_T = None

        self.points = [
                [
                    0.3, 0.2, 0.4, 0, 0, 0
                ]
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.lookup_transform)

        

    def lookup_transform(self):
        try:
           
            transform = self.tf_buffer.lookup_transform( ##iz child v parent frame
                'base_link',    # parent frame
                'kamera',       # child frame
                Time()          # zadnji znani čas
            )

            t = transform.transform.translation
            r = transform.transform.rotation
            q = [r.x, r.y, r.z, r.w]
            T = quaternion_matrix(q)
            T[0, 3] = t.x
            T[1, 3] = t.y
            T[2, 3] = t.z
            self.last_T = T

            # self.get_logger().info(
            #     f"\nTransformacija base_link <- user_2:\n"
            #     f"Translation: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}\n"
            #     f"Rotation (quat): x={r.x:.6f}, y={r.y:.6f}, z={r.z:.6f}, w={r.w:.6f}"
            # )


        except Exception as e:
            self.get_logger().warn(f"Transform ni na voljo: {e}")
            if self.last_T is None:
                return  
            T = self.last_T  



        for i in range(len(self.points)):
            x, y, z, roll, pitch, yaw = self.points[i]

            q_pose = quaternion_from_euler(roll, pitch, yaw)
            T_pose = quaternion_matrix(q_pose)
            T_pose[0:3, 3] = [x, y, z]

            #self.get_logger().info(f"Pose {i} Transformation Matrix:\n{T_pose}")

            T_result = T @ T_pose  

            self.get_logger().info(
            f"\nPose {i} transformed to base_link:\n"
            f"{np.array_str(T_result, precision=3, suppress_small=True)}"
        )

            

             
   
def main(args=None):
    rclpy.init(args=args)
    node = TransformServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
