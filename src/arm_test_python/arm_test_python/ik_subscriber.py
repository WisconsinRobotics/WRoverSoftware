import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from custom_msgs_srvs.msg import GripperPosition
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math
from sensor_msgs.msg import JointState



GRIPPER_SPEED_VALUE = .25
TURN_SPEED = 1 #TODO set this properly

class IKSubscriber(Node):

    def __init__(self):
        super().__init__('arm_logic')
        self.subscription_joint_solutions = self.create_subscription(
            JointState,
            '/relaxed_ik/joint_angle_solutions',
            self.listener_callback,
            10)
        
        self.subscription_joy = self.create_subscription(
            Float32MultiArray,
            'rail',
            self.listener_callback_joy,
            10)

        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons',
            self.listener_callback_buttons,
            10)
            
        self.arm_position_publisher = self.create_publisher(Float32MultiArray, 'arm_angles', 10)

        self.arm_publisher_wrist_left = self.create_publisher(GripperPosition, 'arm_wrist_left', 10)
        self.arm_publisher_wrist_right = self.create_publisher(GripperPosition, 'arm_wrist_right', 10)
    
        self.arm_publisher_gripper = self.create_publisher(Float64, 'arm_gripper', 10)

        
        self.buttons_publisher_ = self.create_publisher(Int16MultiArray, 'buttons', 2)
        self.buttons=[0,0,0,0] #Up, Down, Left, Right

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        timer_period = 0.05  # seconds
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

        self.arm_publisher_base = self.create_publisher(Float64, 'arm_base', 10)


    def listener_callback(self, data):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #print(data.position)
        self.processPositions(data.position)
    
    def listener_callback_joy(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        motion = msg.data

        #Expecting (left trigger, rigt trigger)
        linear_rail_speed = self.get_linear_rail_speed(motion[1], motion[0])
        
        #Publishing
        self.msg_linear_rail.data = linear_rail_speed

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.arm_angles
        #print(msg)
        self.arm_position_publisher.publish(msg)
        #print("Left Position: " + str(float(self.absolute_left_EE)))
        self.msg_wrist.left_position = float(self.arm_angles[2] + self.absolute_left_EE)
        self.msg_wrist.right_position = float(self.arm_angles[2] + self.absolute_right_EE)
        
        self.arm_publisher_wrist_left.publish(self.msg_wrist)
        self.arm_publisher_wrist_right.publish(self.msg_wrist)
        
        self.arm_publisher_gripper.publish(self.msg_gripper)
        self.arm_publisher_base.publish(self.msg_linear_rail)

    def change_rotation(self):
        self.absolute_left_EE += self.add_left_EE
        self.absolute_right_EE += self.add_right_EE


    def processPositions(self, arm_positions):
        #Shoulder
        self.arm_angles[0] = arm_positions[0] *(-105.0 / (math.pi/2))

        #Elbow
        self.arm_angles[1] = -(arm_positions[1] - 2*math.pi) *(105.0 / (math.pi/2))

        #End Effector up and down
        self.arm_angles[2] = (arm_positions[2]* (50.0 / (math.pi/2))) + 50 + self.kohler_shift
    
    def listener_callback_buttons(self, msg):
        buttons = msg.data
                
        #Expecting A and B buttons
        gripper_speed = self.get_gripper_speed(buttons[4], buttons[5])

        #Expecting Left and Right of D_pad
        self.set_turning_speed(buttons[2], buttons[3])

        self.msg_gripper.data = float(gripper_speed)

    def set_turning_speed(self, left_turning, right_turning):
            #TODO: Might have to switch the + and minus
            print(f"Left turning: {left_turning} Right turning: {right_turning}")
            if left_turning == 1:
                self.add_left_EE = TURN_SPEED
                self.add_right_EE = -TURN_SPEED
            elif right_turning == 1:
                self.add_left_EE = -TURN_SPEED
                self.add_right_EE = TURN_SPEED
            else:
                self.add_left_EE = 0.0
                self.add_right_EE = 0.0

    def get_gripper_speed(self, a, b) -> float:
        if a == 1:
            return GRIPPER_SPEED_VALUE
        elif b == 1:
            return -GRIPPER_SPEED_VALUE
        else:
            return 0
    def get_linear_rail_speed(self, left, right) -> Float64:
        #Reverse if right is positive
        #Converting -1 -> 1 range of triggers to 0->1
        return ((left+1)/2 - (right+1)/2)

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


