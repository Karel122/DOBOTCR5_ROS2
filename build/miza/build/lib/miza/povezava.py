import rclpy
from rclpy.node import Node



def table_commands_dict():

    dict_of_commands ={"CW":"M22X;",

                       "ACW":"M23X;",

                        "Stop":"M26X;",

                        "EStop":"M27X;",

                        "ClearC":"M29X;",

                        "Status":"M84X;"}

    return dict_of_commands

# List all available HID devices

def list_hid_devices():

    import hid

    devices = hid.enumerate()

    #for device in devices:

        #print(f"Vendor ID: {device['vendor_id']}, "

              #f"Product ID: {device['product_id']}, "

              #f"Product: {device['product_string']}, "

              #f"Manufacturer: {device['manufacturer_string']}")

    return devices

 

# Function for any command

def command_to_table(vendor_id, product_id, command):

    import hid

    try:

        # Open the device by vendor and product ID

        device = hid.Device(vendor_id, product_id)

        #device.open(vendor_id, product_id)

 

        #print(f"Opened HID device with Vendor ID {vendor_id} and Product ID {product_id}")

 

        # Writing data to the device

        # (Note: The format of the data depends on the device)

        #4D,38,34,58,3B

        data_to_write = 32*[0x0]

        data_to_write = chr(0x01) + command + chr(0) * max(31 - len(command), 0)

        #print(chr(0x01).encode())

        device.write(data_to_write.encode())

        print(f"Wrote data: {data_to_write}")

 

        # Reading data from the device

        response = device.read(32)  # Reading up to 32 bytes

        response_str = ''.join(chr(n) for n in response)

        response_str = response_str.split('\x00',1)[0]

        #print(f"Read data: {response_str}")

 

        # Close the device

        device.close()

        return response_str

 

    except Exception as e:

        print(f"Error interacting with the device: {e}")

        return e

 

# Check status

def moving(vendor_id, product_id):

    import time

    res = command_to_table(vendor_id, product_id, "M84X;")

    print(res, time.time())
    print(res)

    if res[2] == "1":

        odg = False

    else:

        odg = True

    return(odg)

 

def wait_to_avaliable(vendor_id, product_id):

    import time

    while moving(vendor_id, product_id):

        time.sleep(0.2)

 

# Function for specific move commadns

def rot_for(vendor_id, product_id, degree, speed):

    if moving(vendor_id, product_id):

        wait_to_avaliable(vendor_id, product_id)

   

    command = f"G91G01X{degree}F{speed};"

    print(command_to_table(vendor_id, product_id, command))

 

    if moving(vendor_id, product_id):

        wait_to_avaliable(vendor_id, product_id)

   

    print(command_to_table(vendor_id, product_id, "M84X;"))
from dobot_msgs_v4.srv import SendFloat

class MizaNode(Node):
    def __init__(self):
        super().__init__('MizaNode')
        self.get_logger().info('MizaNode started!')
        self.vendor_id = 0x04D8
        self.product_id = 0x0404

        self.srv = self.create_service(SendFloat, 'send_floats', self.callback)
        self.get_logger().info('Service server ready: send_floats')


    def callback(self, request, response):
        self.get_logger().info(f"Received request: zasuk={request.zasuk}, hitrost={request.hitrost}")

        #self.timer = self.create_timer(2.0, self.check_status)

    #def check_status(self):
        rezultat = moving(self.vendor_id, self.product_id)
        self.get_logger().info(f"status:{rezultat}")
        rot_for(self.vendor_id, self.product_id, 90, 500)


def main(args=None):
    rclpy.init(args=args)
    node = MizaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    




if __name__ == '__main__':
    main()