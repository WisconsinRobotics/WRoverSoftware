import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

# NOTE: This might cause problems if called multiple times
pygame.init()


class XboxPublisher(Node):

    def __init__(self):
        super().__init__('arm_xbox_publisher')
        self.arm_publisher = self.create_publisher(Float32MultiArray, 'joy', 10)
        # NOTE: This might need to be tuned
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joysticks = {}
        self.AXIS_BOUNDARY = 0.1

        self.buttons_publisher_ = self.create_publisher(Int16MultiArray, 'buttons', 2)
        self.buttons=[0,0,0,0] #Up, Down, Left, Right

    def timer_callback(self):
        #We have button capability, yippee. 
        running = True
        print("AAAAAAAAA")
        #self.get_logger().debug("BBBBBBBBBB")
        while running:
            print(len(self.joysticks))
            if len(self.joysticks) > 0:
                # Index 0 is left stick x-axis, 1 is left stick y-axis, 3 is right stick x-axis, 2 is right stick y-axis
                motion = [self.joysticks[0].get_axis(2),-self.joysticks[0].get_axis(1),-self.joysticks[0].get_axis(4)]
                # Ignore jitter in sticks
                for i in range(3):
                    if abs(motion[i]) < self.AXIS_BOUNDARY:
                        motion[i] = 0.0
                print(motion)
                # Publish to topic swerve
                swerve_command = Float32MultiArray()
                swerve_command.data = motion
                self.arm_publisher.publish(swerve_command)
            
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.JOYHATMOTION:
                    if event.value[1] == 1:  # D-Pad Up
                        self.buttons[0] = 1
                    else:
                        self.buttons[0] = 0
                    if event.value[1] == -1:  # D-Pad Down
                        self.buttons[1] = 1
                    else:
                        self.buttons[1] = 0
                    if event.value[0] == -1:  # D-Pad Left
                        self.buttons[2] = 1
                    else:
                        self.buttons[2] = 0
                    if event.value[0] == 1:  # D-Pad Right
                        self.buttons[3] = 1
                    else:
                        self.buttons[3] = 0  # Reset to False
                print(self.buttons)
                buttons_command = Int16MultiArray()
                buttons_command.data = self.buttons
                self.buttons_publisher_.publish(buttons_command)
            
                        

                # Handle hotplugging
                if event.type == pygame.JOYDEVICEADDED:
                    # This event will be generated when the program starts for every
                    # joystick, filling up the list without needing to create them manually.
                    self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
                    print(f"{len(self.joysticks)} Joysticks connected")
                    print(f"There are {self.joysticks[0].get_numaxes()} axes")
                    print(self.joysticks)

                if event.type == pygame.JOYDEVICEREMOVED:
                    swerve_command = Float32MultiArray()
                    motion = [0.0,0.0,0.0]
                    swerve_command.data = motion
                    self.swerve_publisher_.publish(swerve_command)
                    self.joysticks = {}
                    print(f"Joystick {event.instance_id} disconnected")

                #if event.type == pygame.JOYAXISMOTION:
                #    # Index 0 is left stick x-axis, 1 is left stick y-axis, 2 is right stick x-axis
                #    motion = [self.joysticks[0].get_axis(0),self.joysticks[0].get_axis(1),self.joysticks[0].get_axis(3)]
                #    # Ignore jitter in sticks
                #    for i in range(3):
                #        if abs(motion[i]) < self.AXIS_BOUNDARY:
                #            motion[i] = 0.0
                #    print(motion)
                #    # Publish to topic swerve
                #    swerve_command = Float32MultiArray()
                #    swerve_command.data = motion
                #    self.swerve_publisher_.publish(swerve_command)

        pygame.quit()
def main(args=None):
    rclpy.init(args=args)

    old_xbox_publisher = XboxPublisher()

    rclpy.spin(old_xbox_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    old_xbox_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
