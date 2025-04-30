import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math


#WRIST_SPEED_VALUE = .2 #As we are publishing 100 times per second. It moves 10% of the way per second.
#GRIPPER_SPEED_VALUE = .2
class ArmLogic(Node):

    def __init__(self):
        super().__init__('arm_logic')
        self.subscription_joy = self.create_subscription(
            Float32MultiArray,
            'swerve',
            self.listener_callback_joy,
            10)
        
        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons',
            self.listener_callback_buttons,
            10)

        self.sci_pub_carousel = self.create_publisher(Float64, 'sci_carousel', 10)
        self.sci_pub_auger = self.create_publisher(Float64, 'sci_auger', 10)
        self.sci_pub_insert = self.create_publisher(Float64, 'sci_insert', 10)
        self.sci_pub_chute = self.create_publisher(Float64, 'sci_chute', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.D_PAD = [0,0,0,0] #Array to keep track of which buttons are pressed
 

        #Define messages beforehand
        self.msg_carousel = Float64()
        self.msg_carousel.data = 0.0

        self.msg_auger = Float64()
        self.msg_auger.data = 0.0

        self.msg_insert = Float64()
        self.msg_insert.data = -0.6

        self.msg_chute = Float64()
        self.msg_chute.data = 1.0

    
    #Put publishers in timer to limit rate of publishing
    def timer_callback(self):
        self.sci_pub_auger.publish(self.msg_auger)
        self.sci_pub_carousel.publish(self.msg_carousel)
        self.sci_pub_insert.publish(self.msg_insert)

    def listener_callback_joy(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        motion = msg.data

        #Expecting (left trigger, rigt trigger)
        carousel_speed = (motion[3]-motion[2])*0.2
        
        #Publishing
        self.msg_carousel.data = carousel_speed

    def listener_callback_buttons(self, msg):
        buttons = msg.data
        
        #Expecting D-Pad
        self.D_PAD = [buttons[0], buttons[1], buttons[2], buttons[3]] # up, down, left, right

        # Insertion controlled by up/down
        self.msg_insert.data = (int(buttons[3])-int(buttons[2]))*0.9

        # Chute controlled by left/right
        if buttons[3]: # eject
            self.msg_chute.data = 0.0
        if buttons[2]: # retain
            self.msg_chute.data = 1.0
        
        #Expecting A and B buttons
        self.msg_auger.data = buttons[4]*0.7 # auger spin when A
        


def main(args=None):
    rclpy.init(args=args)
    swerve_subscriber = ArmLogic()
    print("Starting ros2 arm_logic node")
    rclpy.spin(swerve_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


