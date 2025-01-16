import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SwerveSubscriber(Node):

    def __init__(self):
        super().__init__('swerve_motor')
        self.subscription = self.create_subscription(
            String,
            'swerve',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'can_msg', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        # TODO: Just buttons for now, but this will change to joysticks later
        button_log = msg.data.split(' ')
        button = int(button_log[0])
        pressed = int(button_log[1])
        if pressed and button == 2:
            can_msg = String()
            can_msg.data = "41 CAN_PACKET_SET_CURRENT 3 int"
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    swerve_subscriber = SwerveSubscriber()

    rclpy.spin(swerve_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
