import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, Bool
from sensor_msgs.msg import NavSatFix

CMD_RATE = 10

class DriveControler(Node):
    def __init__(self):
        super().__init__("drive_control")
        ## Start publishing data (very simple)
        self.drive_control_publisher = self.create_publisher(Bool, 'drive_control', 5)
        self.to_drive = True
        self.create_timer(1.0 / CMD_RATE, self.publisher_callback)

        ## Start user input loop so that we can control what we publish
        thread = threading.Thread(target=self.user_input_loop)
        thread.daemon = True
        thread.start()

    def user_input_loop(self):
        while rclpy.ok():
            try:
                console_msg = None
                if self.to_drive:
                    console_msg = "To stop autonomous drive type FALSE: "
                else:
                    console_msg = "To start autonomous drive type TRUE: "

                user_input = input(console_msg)
                if user_input == "FALSE" and self.to_drive:
                    self.to_drive = False
                elif user_input == "TRUE" and (not self.to_drive):
                    self.to_drive = True
                else:
                    print("Type valid command")
            except IOError:
                break

    def publisher_callback(self):
        msg = Bool()
        msg.data = self.to_drive
        self.drive_control_publisher.publish(msg)


def main():
    rclpy.init()
    node = DriveControler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
