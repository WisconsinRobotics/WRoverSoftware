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
        data = float(msg.data)
        # TODO: Make this more abstract for actual control
        if data == 4.0:
            can_msg = String()
            can_msg.data = "41 CAN_PACKET_SET_CURRENT 4 int"
            self.publisher_.publish(can_msg)


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
