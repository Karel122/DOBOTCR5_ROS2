import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import GetAngle, SetFloat

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client_node')

        # Create a client for the service
        self.cli = self.create_client(SetFloat, '/nalozi_tocke')  # Replace with actual service name

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to be available...')

        # Create a request message
        self.req = SetFloat.Request()  # Replace with actual service request message

        # Optionally, set request parameters
        #self.req.some_field = 'value'  # Modify based on service parameters

        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)

        # Add a callback when the service call completes
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            # Get the result from the future
            response = future.result()
            self.get_logger().info(f"Service call successful: {response}")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()
    rclpy.spin(node)

    # Shutdown ROS 2 when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
