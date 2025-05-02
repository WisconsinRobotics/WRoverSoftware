import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

from relaxed_ik_ros2.msg import  EEVelGoals
from geometry_msgs.msg import  Twist


CONTROLLER = 1
# NOTE: This might cause problems if called multiple times
#RUN IK WITH VELOCITY CONTROL
pygame.init()

class XboxPublisher(Node):

    def __init__(self):
        super().__init__('arm_xbox_publisher')
        # NOTE: This might need to be tuned
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joysticks = {}
        self.AXIS_BOUNDARY = 0.1

        self.buttons=[0,0,0,0,0,0] #Up, Down, Left, Right, A, B

        self.ee_vel_goals_pub = self.create_publisher(EEVelGoals, 'relaxed_ik/ee_vel_goals', 5)
        self.linear = [0.0,0.0,0.0]
        self.angular = [0.0,0.0,0.0]

    def timer_callback(self):
        
        #self.get_logger().debug("BBBBBBBBBB")
        if (len(self.joysticks) >CONTROLLER):#TODO:CHANGE THIS IS FOR TESTING
            
            # Index 0 is left stick x-axis, 1 is left stick y-axis, 3 is right stick x-axis, 2 is right stick y-axis
            motion = [self.joysticks[CONTROLLER].get_axis(2),-self.joysticks[CONTROLLER].get_axis(1),-self.joysticks[CONTROLLER].get_axis(4)]
            print(motion)
            # Ignore jitter in sticks
            for i in range(3):
                if abs(motion[i]) < self.AXIS_BOUNDARY:
                    motion[i] = 0.0
    
            self.linear[0] = -motion[1]/15000 #Moving UP and DOWN
            # No Y movement (linear[1])
            self.linear[2] = motion[2] /15000 #Moving side to side
            
            #NO angular rotation for simulation
            self.angular[0] = 0.0

            if self.buttons[0] == 1: #UP D_PAD
                self.angular[1] = 8.0/10000.0 #Rotate end effector up
            elif self.buttons[1] == 1: #DOWN D_PAD
                self.angular[1] = -8.0/10000.0 #Rotate end effector down
            else:
                self.angular[1] = 0.0

            #The actual turning of the end effector will be controlled by the 
            #               rail_gripper_controller as it is not IK dependent
            if self.buttons[2] == 1:
                self.angular[0] = 8/10000.0 #Rotate end effector left
            elif self.buttons[3] == 1:
                self.angular[0] = -8.0/10000.0 #Rotate end effector right
            else:
                self.angular[0] = 0.0
            #self.get_logger().info("Publishing")
            
            msg = EEVelGoals()
            for i in range(1):
                twist = Twist()
                twist.linear.x = self.linear[0]
                twist.linear.y = self.linear[1]
                twist.linear.z = self.linear[2]

                twist.angular.x = self.angular[0]
                twist.angular.y = self.angular[1]
                twist.angular.z = self.angular[2]

                tolerance = Twist()
                tolerance.linear.x = 0.0
                tolerance.linear.y = 0.0
                tolerance.linear.z = 0.0
                tolerance.angular.x = 0.0
                tolerance.angular.y = 0.0
                tolerance.angular.z = 0.0

                msg.ee_vels.append(twist)
                msg.tolerances.append(tolerance)
            #self.get_logger().info("Publishing: " + str(self.linear[0]))
            self.ee_vel_goals_pub.publish(msg)        
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.JOYHATMOTION:
                if event.joy ==CONTROLLER:
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

def main(args=None):
    rclpy.init(args=args)

    arm_rail_xbox = XboxPublisher()

    rclpy.spin(arm_rail_xbox)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_rail_xbox.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
