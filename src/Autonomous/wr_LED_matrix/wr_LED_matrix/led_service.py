from custom_msgs_srvs.srv import LED

import rclpy
from rclpy.node import Node
import serial


class LEDService(Node):

    def __init__(self):
        super().__init__('led_service')

        port = "/dev/ttyACM0"
        baud = 9600
        self.declare_parameter('led_mode', "mock")

        try:
            self.led_mode = self.get_parameter('led_mode').get_parameter_value().string_value
            if self.led_mode == "real":
                self.s = serial.Serial(port, baud, timeout=1)
                self.get_logger().info("Waiting for Arduino magic word...")

                # Wait until enough bytes are available
                while self.s.in_waiting < 8:
                    #self.get_logger().info("input bits: " + str(self.s.in_waiting))
                    rclpy.spin_once(self, timeout_sec=0.1)

                magic_word = self.s.read(8)
                if magic_word != bytes([0x31, 0x41, 0x59, 0x26, 0xDE, 0xAD, 0xBE, 0xEF]):
                    self.get_logger().error(
                        f"Received incorrect magic word ({magic_word}) on startup, exiting..."
                    )
                    exit(1)

                self.get_logger().info("Magic word OK. Serial ready. Starting ROS service.")
            self.srv = self.create_service(LED, 'change_LED', self.change_LED)

        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            exit(1)

    def change_LED(self, request, response):
        self.get_logger().info(f'Received color: R={request.red}, G={request.green}, B={request.blue}')
        if self.led_mode == "real":
            crc = request.red ^ request.green ^ request.blue
            packet = bytearray([request.red, request.green, request.blue, crc])
            self.s.write(packet)
        return response
    def turn_off_LED(self):
        packet = bytearray([0, 0, 0, 0])
        self.s.write(packet)


def main():
    rclpy.init()
    node = LEDService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.turn_off_LED()
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if hasattr(node, 's') and node.s.is_open:
            node.s.close()


if __name__ == '__main__':
    main()
