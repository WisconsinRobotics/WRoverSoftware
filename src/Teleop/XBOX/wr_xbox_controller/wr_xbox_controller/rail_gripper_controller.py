import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

# NOTE: This might cause problems if called multiple times
pygame.init()

CONTROLLER = 1 # Choose controller for the arm TODO 

class XboxPublisher(Node):

    def __init__(self):
        super().__init__('arm_xbox_publisher')
        self.arm_publisher = self.create_publisher(Float32MultiArray, 'rail', 10)
        # NOTE: This might need to be tuned
        timer_period = .01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joysticks = {}
        self.AXIS_BOUNDARY = 0.1
        self.motion_command = Float32MultiArray()
        self.motion = [0.0, 0.0]

        self.buttons_publisher_ = self.create_publisher(Int16MultiArray, 'buttons', 2)
        self.buttons=[0,0,0,0,0,0] #Up, Down, Left, Right, A, B


    def timer_callback(self):
        #We have button capability, yippee. 
        #print(len(self.joysticks))
        #print(self.joysticks[0])
        if len(self.joysticks) > 0:
            self.motion = [self.joysticks[0].get_axis(2), #Left trigger
                      self.joysticks[0].get_axis(5) ] #Right trigger

        self.motion_command.data = self.motion
        self.arm_publisher.publish(self.motion_command)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.joy == CONTROLLER:
                    if event.button == 0:  # A button
                        self.buttons[4] = 1
                    elif event.button == 1:  # B button
                        self.buttons[5] = 1
                    elif event.button == 2: # X Button
                        self.get_logger().info("Pressed ARM controller (ARM)")

            elif event.type == pygame.JOYBUTTONUP:
                if event.joy == CONTROLLER:
                    if event.button == 0:  # A button
                        self.buttons[4] = 0
                    elif event.button == 1:  # B button
                        self.buttons[5] = 0
            if event.type == pygame.JOYHATMOTION:
                if event.joy == CONTROLLER:
                    if event.value[0] == -1:  # D-Pad Left
                        self.buttons[2] = 1
                    else:
                        self.buttons[2] = 0
                    if event.value[0] == 1:  # D-Pad Right
                        self.buttons[3] = 1
                    else:
                        self.buttons[3] = 0  # Reset to False
            #print(self.buttons)
            buttons_command = Int16MultiArray()
            buttons_command.data = self.buttons  
            #print(buttons_command)
            self.buttons_publisher_.publish(buttons_command)    
                  
    
            if event.type == pygame.QUIT:
                running = False
            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
                print(f"{len(self.joysticks)} Joysticks connected")
                #print(f"Joystick {event.instance_id} connected")
                #print(f"There are {self.joysticks[0].get_numaxes()} axes")
                #print(self.joysticks)

            if event.type == pygame.JOYDEVICEREMOVED:
                swerve_command = Float32MultiArray()
                motion = [0.0,0.0,0.0]
                swerve_command.data = motion
                self.arm_publisher.publish(swerve_command)
                self.joysticks = {}
                print(f"Joystick {event.instance_id} disconnected")

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
