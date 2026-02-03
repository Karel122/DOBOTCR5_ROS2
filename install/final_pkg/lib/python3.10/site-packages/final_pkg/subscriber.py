import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json



class Joint_States_Subscriber(Node):

    def __init__(self):
        super().__init__('joint_states_subscriber')
        self.subscription = self.create_subscription(
           JointState,
            '/joint_states_robot',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        return    

def main(args=None):
    rclpy.init(args=args)
    node = Joint_States_Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
