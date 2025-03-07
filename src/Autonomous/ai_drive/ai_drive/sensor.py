import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AI_Drive(Node):

    def __init__(self):
        super().__init__('ai_drive')

        # Publisher to robot movement
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        # Subscriber for LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        # Timer for movement
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.obstacle_detected = False  # Track if an obstacle is detected

    def listener_callback(self, msg: LaserScan):  # <-- Fix: Added `msg` parameter
        """Callback function for LiDAR data"""
        min_distance = min(msg.ranges)  # Get closest obstacle distance

        # Define a safety threshold (e.g., stop if an object is closer than 0.5m)
        if min_distance < 0.5:
            self.obstacle_detected = True
            self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m! Stopping.")
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        """Publishes movement commands"""
        msg = Twist()

        if self.obstacle_detected:
            msg.linear.x = 0.0  # Stop the robot
        else:
            msg.linear.x = 1.0  # Move forward

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity: {msg.linear.x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = AI_Drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
