#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import SendFloat


class MizaClient(Node):
    def __init__(self):
        super().__init__('miza_client')
        self.client = self.create_client(SendFloat, 'send_floats')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for send_floats service...')

        self.request = SendFloat.Request()
        self.request.hitrost = 500.0
        self.request.zasuk = 90.0
        
        self.future = self.client.call_async(self.request)
        self.timer = self.create_timer(0.5, self.check_response)

    def check_response(self):
        if self.future.done():
            try:
                response = self.future.result()
                self.get_logger().info(f'Service response: success={response.success}')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
            finally:
                self.timer.cancel()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MizaClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
