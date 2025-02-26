import rclpy
from rclpy.node import Node
import requests
import time
import random

from sensor_msgs.msg import NavSatFix

server_url = "http://127.0.0.1:5000/update_location"

class GpsSubscriber(Node):

    def __init__(self):
        super().__init__('display_gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'fix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('Latitude: %s' % msg.latitude)
        #self.get_logger().info('Longitude: %s' % msg.longitude)
        """Callback function that sends GPS data to the server whenever a new message arrives."""
        location = {"lat": msg.latitude, "lon": msg.longitude}

        try:
            response = requests.post(server_url, json=location)
            if response.status_code == 200:
                self.get_logger().info(f"Updated location: {location}")
            else:
                self.get_logger().warn(f"Error updating location: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send data: {e}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GpsSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
