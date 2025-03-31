import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from custom_msgs_srvs.msg import GripperPosition
import math

class SwerveControlSubsrciber(Node):

    def __init__(self):
        super().__init__('swerve_control')
        #TODO:CHANGE IDS
        self.vesc_ids = {"BASE":["79"],
                         "WRIST_LEFT":["80"],
                         "WRIST_RIGHT":["81"],
                         "GRIPPER":["78"]
                        }
        self.max_rpm = 6000
        self.limit_rotation = 0

        self.subscription_base = self.create_subscription(
            Float64,
            'arm_base',
            self.arm_listener_base,
            10)

        self.subscription_wrist_left = self.create_subscription(
            GripperPosition,
            'arm_wrist_left',
            self.arm_listener_wrist_left,
            10)
        
        self.subscription_wrist_right = self.create_subscription(
            GripperPosition,
            'arm_wrist_right',
            self.arm_listener_wrist_right,
            10)
                
        self.subscriptoin_gripper = self.create_subscription(
            Float64,
            'arm_gripper',
            self.arm_listener_gripper,
            10)

        self.publisher_ = self.create_publisher(String, 'can_msg', 10)


    def arm_listener_base(self, msg):
        can_msg_rpm = String()

        rpm = msg.data * self.max_rpm * 1.5
        can_msg_rpm.data = self.vesc_ids["BASE"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        #self.get_logger().info('RPM BASE: "%s"' % can_msg_rpm.data + '\n')

    def arm_listener_wrist_left(self, msg):
        can_msg_angle = String()
        turn_amount = (msg.left_position)
       
        can_msg_angle.data = self.vesc_ids["WRIST_LEFT"][0] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
        #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
        #self.publisher_.publish(can_msg_angle)
        self.get_logger().info('Publishing Angle WRIST_LEFT: "%s"' % can_msg_angle)

    def arm_listener_wrist_right(self, msg):
        can_msg_angle = String()
        turn_amount = (msg.right_position)
        
        can_msg_angle.data = self.vesc_ids["WRIST_RIGHT"][0] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
        #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
        #self.publisher_.publish(can_msg_angle)
        self.get_logger().info('Publishing Angle WRIST_RIGHT: "%s"' % can_msg_angle)

    def arm_listener_gripper(self, msg):
        can_msg_rpm = String()

        rpm = msg.data * self.max_rpm
        can_msg_rpm.data = self.vesc_ids["GRIPPER"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        #self.get_logger().info('RPM GRIPPER: "%s"' % can_msg_rpm.data + '\n')

def main(args=None):
    rclpy.init(args=args)

    swerve_control_subscriber = SwerveControlSubsrciber()
    print("Starting ros2 arm_neo node")
    rclpy.spin(swerve_control_subscriber)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
