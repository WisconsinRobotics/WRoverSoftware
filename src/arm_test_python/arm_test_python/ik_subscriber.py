import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from custom_msgs_srvs.msg import GripperPosition
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math
from sensor_msgs.msg import JointState

#TODO: FIX IK for wrist

GRIPPER_SPEED_VALUE = .25
WRIST_SPEED_VALUE = .16
TURN_SPEED = 0.3 #TODO set this properly
MAX_ANGLE = 140

class IKSubscriber(Node):

    def __init__(self):
        super().__init__('arm_logic')
        self.subscription_joint_solutions = self.create_subscription(
            JointState,
            '/relaxed_ik/joint_angle_solutions',
            self.listener_callback,
            10)
        

        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons_arm',
            self.listener_callback_buttons,
            10)
            
        self.arm_position_publisher = self.create_publisher(Float32MultiArray, 'arm_angles', 10)

        self.arm_publisher_wrist_left = self.create_publisher(GripperPosition, 'arm_wrist_left', 10)
        self.arm_publisher_wrist_right = self.create_publisher(GripperPosition, 'arm_wrist_right', 10)
    
        self.arm_publisher_gripper = self.create_publisher(Float64, 'arm_gripper', 10)

        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        timer_period = 0.005  # TODO: seconds
        self.timer_rotation = self.create_timer(timer_period, self.change_rotation)

        
        self.kohler_shift = 130
        self.arm_angles = [0.0, 0.0, 50.0 + self.kohler_shift]

        #Define messages beforehand
        self.msg_linear_rail = Float64()
        self.msg_linear_rail.data = 0.0

        self.msg_wrist = GripperPosition()
        self.msg_wrist.left_position = 180.0
        self.msg_wrist.right_position = 180.0
        self.add_left_EE = 0.0
        self.add_right_EE = 0.0
        self.absolute_left_EE = 0.0
        self.absolute_right_EE = 0.0

        self.msg_gripper = Float64()
        self.msg_gripper.data = 0.0

        self.angle_change = 0.0
        self.absolute_angle = 0.0


    def listener_callback(self, data):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #print(data.position)
        self.processPositions(data.position)
    

        
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.arm_angles
        #print(msg)
        self.arm_position_publisher.publish(msg)
        #print("Left Position: " + str(float(self.arm_angles[2] + self.absolute_left_EE)))
        #self.msg_wrist.left_position = float(-self.arm_angles[2] + self.absolute_left_EE - self.absolute_angle)
        #self.msg_wrist.right_position = float(-self.arm_angles[2] + self.absolute_right_EE - self.absolute_angle)
        
        #Wrist Simple TODO
        self.msg_wrist.left_position = float(self.absolute_left_EE - self.absolute_angle + 50+ self.kohler_shift)
        self.msg_wrist.right_position = float(self.absolute_right_EE - self.absolute_angle + 50+self.kohler_shift)

        #self.get_logger().info('Left Position: "%s"' % self.msg_wrist.left_position)
        #self.get_logger().info('Right Position: "%s"' % self.msg_wrist.right_position)

        self.arm_publisher_wrist_left.publish(self.msg_wrist)
        self.arm_publisher_wrist_right.publish(self.msg_wrist)
        #print("Msg Gripper: " + str(self.msg_gripper))
        self.arm_publisher_gripper.publish(self.msg_gripper)
        

    def change_rotation(self):
        self.absolute_left_EE += self.add_left_EE
        self.absolute_right_EE += self.add_right_EE
        #Making sure it doesn't go past limit but also that it can go back
        if self.absolute_angle <= MAX_ANGLE and self.angle_change == WRIST_SPEED_VALUE:
            self.absolute_angle += self.angle_change
        if self.absolute_angle >= -MAX_ANGLE and self.angle_change == -WRIST_SPEED_VALUE:
            self.absolute_angle += self.angle_change


    def processPositions(self, arm_positions):
        #Shoulder
        self.arm_angles[0] = arm_positions[0] *(-105.0 / (math.pi/2))

        #Elbow
        self.arm_angles[1] = -(arm_positions[1]) *(105.0 / (math.pi/2))
        #self.get_logger().info('I heard: "%s"' % arm_positions[2])


        #End Effector up and down
        self.arm_angles[2] = (arm_positions[2]* (120.0 / (math.pi/2))) + 50 + self.kohler_shift
    
    def listener_callback_buttons(self, msg):
        buttons = msg.data
        #Expecting D-Pad
        self.D_PAD = [buttons[0], buttons[1], buttons[2], buttons[3]] # up, down, left, right
        
        #Turn gripper side to side
        self.set_turning_speed(self.D_PAD[2], self.D_PAD[3])

        #Move up and down gripper
        self.update_angle(self.D_PAD[0], self.D_PAD[1])

        #Expecting A and B buttons
        gripper_speed = self.get_gripper_speed(buttons[4], buttons[5])
        self.msg_gripper.data = float(gripper_speed)

    def set_turning_speed(self, left_turning, right_turning):
        #print(f"Left turning: {left_turning} Right turning: {right_turning}")
        if left_turning == 1:
            self.add_left_EE = TURN_SPEED
            self.add_right_EE = -TURN_SPEED
        elif right_turning == 1:
            self.add_left_EE = -TURN_SPEED
            self.add_right_EE = TURN_SPEED
        else:
            self.add_left_EE = 0.0
            self.add_right_EE = 0.0

    def update_angle(self, up, down):
        if up == 1:
            if self.absolute_angle <= MAX_ANGLE:
                self.angle_change = WRIST_SPEED_VALUE
            else:
                self.angle_change = 0.0
        elif down == 1:
            if self.absolute_angle >= -MAX_ANGLE:  
                self.angle_change = -WRIST_SPEED_VALUE
            else:
                self.angle_change = 0.0
        else:
            self.angle_change = 0.0


    def get_gripper_speed(self, a, b) -> float:
        if a == 1:
            return GRIPPER_SPEED_VALUE
        elif b == 1:
            return -GRIPPER_SPEED_VALUE
        else:
            return 0


def main(args=None):
    rclpy.init(args=args)
    ik_subscriber = IKSubscriber()
    print("Starting ros2 arm_logic node")
    rclpy.spin(ik_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ik_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


