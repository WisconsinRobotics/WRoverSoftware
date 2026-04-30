#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Test node started.")

    def timer_callback(self):
        self.get_logger().info("Hello from ROS2 Python node!")


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
