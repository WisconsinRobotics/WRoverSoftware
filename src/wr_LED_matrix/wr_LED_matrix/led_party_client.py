import random
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from custom_msgs_srvs.srv import LED
import time

class LEDClientParty(Node):

    def __init__(self):
        super().__init__('led_party_client')
        self.led_cli = self.create_client(LED, 'change_LED')
        self.led_req = LED.Request()
        while not self.led_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        self.led_req.red = random.randint(0, 255)
        self.led_req.green = random.randint(0, 255)
        self.led_req.blue = random.randint(0, 255)
        return self.led_cli.call_async(self.led_req)


def main():
    rclpy.init()
    minimal_client = LEDClientParty()

    try:
        while rclpy.ok():
            future = minimal_client.send_request()
            rclpy.spin_until_future_complete(minimal_client, future)
            time.sleep(0.1)  # wait 0.1 seconds before next request (adjust as needed)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()