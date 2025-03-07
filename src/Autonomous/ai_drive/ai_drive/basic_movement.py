import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AI_Drive(Node):

    def __init__(self):
        super().__init__('ai_drive')

        #Publisher to robot movement
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        #Subscriber from lidar data
        #self.subscription = self.create_subscription(
        #    LaserScan,
        #    '/scan',
        #    self.listener_callback,
        #    10)
        #self.subscription  # prevent unused variable warning

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(1)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.linear.x)
        self.i += 1

    def listener_callback(self):
        self.get_logger().info('Subscribing to data')


def main(args=None):
    rclpy.init(args=args)

    node = AI_Drive()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()