import rclpy
from rclpy.node import Node
import pygame

# NOTE: This might cause problems if called multiple times
pygame.init()

from std_msgs.msg import String

class XboxPublisher(Node):

    def __init__(self):
        super().__init__('xbox_publisher')
        self.publisher_ = self.create_publisher(String, 'xbox', 10)
        # NOTE: This might need to be tuned
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joysticks = {}

    def timer_callback(self):
        # Message will be formatted as "{button} {value}"
        msg = String()

        # TODO: add stick capability
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"{event.button} pressed.")
                msg.data = f"{event.button} 1"
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)


            if event.type == pygame.JOYBUTTONUP:
                print(f"{event.button} released.")
                msg.data = f"{event.button} 0"
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)


            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                self.joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del self.joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()

        # For each joystick:
        for joystick in self.joysticks.values():
            jid = joystick.get_instance_id()

def main(args=None):
    rclpy.init(args=args)

    xbox_publisher = XboxPublisher()

    rclpy.spin(xbox_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    xbox_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
