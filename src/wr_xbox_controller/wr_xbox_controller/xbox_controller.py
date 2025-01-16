import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import String

# NOTE: This might cause problems if called multiple times
pygame.init()


class XboxPublisher(Node):

    def __init__(self):
        super().__init__('xbox_publisher')
        self.publisher_ = self.create_publisher(String, 'xbox', 10)
        # NOTE: This might need to be tuned
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joysticks = {}

    def timer_callback(self):
        # No button capability, but doesn't sound like we need it. 
        for event in pygame.event.get():
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

        # TODO: Currently movement is binary. Could make it marginal for finer control
        # Target joystick 0 for swerve
        sticks = list(self.joysticks.values())
        if sticks:
            swerve_stick = sticks[0]
            # Axis 1 is left stick
            sstick_axis = swerve_stick.get_axis(1)
            # -1 means up, 1 means down
            if sstick_axis <= -0.5:
                swerve_command = String()
                swerve_command.data = ""
                # Command is "value"
                self.publisher_.publish("1.0")
            elif sstick_axis >= 0.5:
                swerve_command = String()
                swerve_command.data = ""
                # Command is "value"
                self.publisher_.publish("-1.0")


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
