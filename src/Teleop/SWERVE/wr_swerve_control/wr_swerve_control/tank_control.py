import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import math

class SwerveControlSubsrciber(Node):

    def __init__(self):
        super().__init__('swerve_control')
        #Even is drive
        #Odd is swerve
        self.vesc_ids = {"FL":["70","71"],
                         "FR":["72","73"],
                         "BL":["74","75"],
                         "BR":["76","77"]
                        }
        self.max_rpm = 10000
        self.limit_rotation = 0
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'tank',
            self.listener_callback,
            10)
        
        self.motion = [0.0, 0.0] #Tank input

        self.pos_comp = 10
        self.neg_comp = -10

        # Timer to publish periodically
        self.publisher_timer = self.create_timer(0.05, self.publish)

        self.publisher_timer_BL = self.create_timer(0.1, self.publish_BL)


        self.publisher_ = self.create_publisher(String, 'can_msg', 1)
        self.can_msg_rpm_FL = String()
        self.can_msg_rpm_FL.data = self.vesc_ids["FL"][0] + " CAN_PACKET_SET_RPM " + str(0.0) + " float"
        self.can_msg_angle_FL = String()
        self.can_msg_angle_FL.data = self.vesc_ids["FL"][1] + " CAN_PACKET_SET_POS " + str(180.0) +" float"

        self.can_msg_rpm_FR = String()
        self.can_msg_rpm_FR.data = self.vesc_ids["FR"][0] + " CAN_PACKET_SET_RPM " + str(0) + " float"
        self.can_msg_angle_FR = String()
        self.can_msg_angle_FR.data = self.vesc_ids["FR"][1] + " CAN_PACKET_SET_POS " + str(180.0) +" float"

        self.can_msg_rpm_BL = String()
        self.can_msg_rpm_BL.data = self.vesc_ids["BL"][0] + " CAN_PACKET_SET_RPM " + str(0) + " float"
        self.can_msg_angle_BL = String()
        self.can_msg_angle_BL.data = self.vesc_ids["BL"][1] + " CAN_PACKET_SET_POS " + str(180.0) +" float"

        self.can_msg_rpm_BR = String()
        self.can_msg_rpm_BR.data = self.vesc_ids["BR"][0] + " CAN_PACKET_SET_RPM " + str(0) + " float"
        self.can_msg_angle_BR = String()
        self.can_msg_angle_BR.data = self.vesc_ids["BR"][1] + " CAN_PACKET_SET_POS " + str(180.0) +" float"


    def publish(self):
        
        combined_msg = "\n".join([
        self.can_msg_rpm_FL.data,
        self.can_msg_rpm_FR.data,
        self.can_msg_rpm_BL.data,
        self.can_msg_rpm_BR.data,
        self.can_msg_angle_FL.data,
        self.can_msg_angle_FR.data,
        self.can_msg_angle_BL.data,
        self.can_msg_angle_BR.data
        ])

        msg = String()
        msg.data = combined_msg

        self.publisher_.publish(msg)


    def listener_callback(self, msg):
        #Should receive [left, right]
        self.motion = msg.data
        self.publish_FL()
        self.publish_FR()
        self.publish_BL()
        self.publish_BR()
    

    def publish_FR(self):
        # Backlash Compensation first
        if (self.motion[1] > 0.0):
            turn_amount = (0/4 + 180)
        else:
            turn_amount = (0/4 + 180)
        
        self.can_msg_angle_FR.data = self.vesc_ids["FR"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"

        rpm = self.motion[1] * self.max_rpm
        self.can_msg_rpm_FR.data = self.vesc_ids["FR"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        #self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    def publish_FL(self):
        # Backlash Compensation first
        if (self.motion[1] > 0.0):
            turn_amount = (0/4 + 180)
        else:
            turn_amount = (0/4 + 180)
        
        self.can_msg_angle_FL.data = self.vesc_ids["FL"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
        
        rpm = self.motion[0] * self.max_rpm
        self.can_msg_rpm_FL.data = self.vesc_ids["FL"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        #self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    def publish_BR(self):
        # Backlash Compensation first
        if (self.motion[1] > 0.0):
            turn_amount = (0/4 + 180)
        else:
            turn_amount = (0/4 + 180)
        
        self.can_msg_angle_BR.data = self.vesc_ids["BR"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
        
        rpm = self.motion[1] * self.max_rpm
        self.can_msg_rpm_BR.data = self.vesc_ids["BR"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        #self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    def publish_BL(self):
        # Backlash Compensation first
        if (self.motion[1] > 0.0):
            turn_amount = (0/4 + 180)
        else:
            turn_amount = (0/4 + 180)
        
        self.can_msg_angle_BL.data = self.vesc_ids["BL"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
        
        rpm = self.motion[0] * self.max_rpm
        self.can_msg_rpm_BL.data = self.vesc_ids["BL"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        #self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    

def main(args=None):
    rclpy.init(args=args)

    swerve_control_subscriber = SwerveControlSubsrciber()

    rclpy.spin(swerve_control_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
