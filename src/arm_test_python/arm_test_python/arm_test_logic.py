import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math


WRIST_SPEED_VALUE = .2
GRIPPER_SPEED_VALUE = .2
class ArmLogic(Node):

    def __init__(self):
        super().__init__('arm_logic')
        self.subscription_joy = self.create_subscription(
            Float32MultiArray,
            'joy',
            self.listener_callback_joy,
            10)
        
        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons',
            self.listener_callback_buttons,
            10)

        self.arm_publisher_base = self.create_publisher(Float64, 'arm_base', 10)
        self.arm_publisher_wrist_left = self.create_publisher(Float64, 'arm_wrist_left', 10)
        self.arm_publisher_wrist_right = self.create_publisher(Float64, 'arm_wrist_right', 10)
        self.arm_publisher_gripper = self.create_publisher(Float64, 'arm_gripper', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Define messages beforehand
        self.msg_linear_rail = Float64()
        self.msg_linear_rail.data = 0.0

        self.msg_wrist_left = Float64()
        self.msg_wrist_left.data = 0.0

        self.msg_wrist_right = Float64()
        self.msg_wrist_right.data = 0.0

        self.msg_gripper = Float64()
        self.msg_gripper.data = 0.0
    
    #Put publishers in timer to limit rate of publishing
    def timer_callback(self):
        self.arm_publisher_base.publish(self.msg_linear_rail)
        self.arm_publisher_wrist_left.publish(self.msg_wrist_left)
        self.arm_publisher_wrist_right.publish(self.msg_wrist_right)
        self.arm_publisher_gripper.publish(self.msg_gripper)


    def get_linear_rail_speed(self, left, right) -> Float64:
        #Reverse if right is positive
        #Converting -1 -> 1 range of triggers to 0->1
        return ((left+1)/2 - (right+1)/2)
    
    def get_wrist_speeds(self, up, down, left, right) -> Float32MultiArray:
        #Assume left is forward
        wrist_speeds = [0,0]
        if up == 1:
            wrist_speeds = [WRIST_SPEED_VALUE, -WRIST_SPEED_VALUE]
            return wrist_speeds
        elif down == 1:
            wrist_speeds = [-WRIST_SPEED_VALUE, WRIST_SPEED_VALUE]
            return wrist_speeds
        elif left == 1:
            wrist_speeds = [WRIST_SPEED_VALUE, WRIST_SPEED_VALUE]
            return wrist_speeds
        elif right == 1:
            wrist_speeds = [-WRIST_SPEED_VALUE, -WRIST_SPEED_VALUE]
            return wrist_speeds
        else:
            return wrist_speeds

    
    def get_gripper_speed(self, a, b) -> float:
        if a == 1:
            return GRIPPER_SPEED_VALUE
        elif b == 1:
            return -GRIPPER_SPEED_VALUE
        else:
            return 0

    def listener_callback_joy(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        motion = msg.data

        #Expecting (left trigger, rigt trigger)
        linear_rail_speed = self.get_linear_rail_speed(motion[2], motion[3])
        
        #Publishing
        self.msg_linear_rail.data = linear_rail_speed

    def listener_callback_buttons(self, msg):
        buttons = msg.data
        
        #Expecting D-Pad
        wrist_speeds = self.get_wrist_speeds(buttons[0], buttons[1], buttons[2], buttons[3])
        #Expecting A and B buttonrs
        gripper_speed = self.get_gripper_speed(buttons[4], buttons[5])
        
        #Publishing
        self.msg_wrist_left.data = float(wrist_speeds[0])
        self.msg_wrist_right.data = float(wrist_speeds[1])
        self.msg_gripper.data = float(gripper_speed)
    


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


