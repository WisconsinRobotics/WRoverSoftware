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
        self.max_rpm_change = 1000
        self.limit_rotation = 0
        self.subscription_FL = self.create_subscription(
            Float32MultiArray,
            'swerve_FL',
            self.swerve_listener_FL,
            10)

        self.subscription_FR = self.create_subscription(
            Float32MultiArray,
            'swerve_FR',
            self.swerve_listener_FR,
            10)
        
        self.subscription_BL = self.create_subscription(
            Float32MultiArray,
            'swerve_BL',
            self.swerve_listener_BL,
            10)
                
        self.subscription_BR = self.create_subscription(
            Float32MultiArray,
            'swerve_BR',
            self.swerve_listener_BR,
            10)

        self.publisher_timer = self.create_timer(0.05, self.publish)

        self.publisher_ = self.create_publisher(String, 'can_msg', 10)

        self.can_msg_rpm_FL = String()
        self.can_msg_rpm_FL.data = self.vesc_ids["FL"][0] + " CAN_PACKET_SET_RPM " + str(0.0) + " float"
        self.can_msg_angle_FL = String()
        self.can_msg_angle_FL.data = self.vesc_ids["FL"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"

        self.can_msg_rpm_FR = String()
        self.can_msg_rpm_FR.data = self.vesc_ids["FR"][0] + " CAN_PACKET_SET_RPM " + str(0) + " float"
        self.can_msg_angle_FR = String()
        self.can_msg_angle_FR.data = self.vesc_ids["FR"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"

        self.can_msg_rpm_BL = String()
        self.can_msg_rpm_BL.data = self.vesc_ids["BL"][0] + " CAN_PACKET_SET_RPM " + str(0) + " float"
        self.can_msg_angle_BL = String()
        self.can_msg_angle_BL.data = self.vesc_ids["BL"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"

        self.can_msg_rpm_BR = String()
        self.can_msg_rpm_BR.data = self.vesc_ids["BR"][0] + " CAN_PACKET_SET_RPM " + str(0) + " float"
        self.can_msg_angle_BR = String()
        self.can_msg_angle_BR.data = self.vesc_ids["BR"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"


        # Previous published RPMs
        self.prev_rpm_FL = 0.0
        self.prev_rpm_FR = 0.0
        self.prev_rpm_BL = 0.0
        self.prev_rpm_BR = 0.0

        self.subscription_FL
        self.subscription_FR
        self.subscription_BL
        self.subscription_BR
        self.get_logger().info("Started SWERVE NODE")



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

    def swerve_listener_FL(self, msg):
        
        
        turn_amount = (msg.data[1]/4 + 180)
        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_FL.data = self.vesc_ids["FL"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle FL: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        if(abs(self.prev_rpm_FL - rpm) < self.max_rpm_change):
            self.can_msg_rpm_FL.data = self.vesc_ids["FL"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
            self.prev_rpm_FL = rpm
            #self.get_logger().info('Publishing RPM FL: "%s"' % self.can_msg_rpm_FL)

    def swerve_listener_FR(self, msg):

        turn_amount = (msg.data[1]/4 + 180)
        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_FR.data = self.vesc_ids["FR"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle FR: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        if(abs(self.prev_rpm_FR - rpm) < self.max_rpm_change):
            self.can_msg_rpm_FR.data = self.vesc_ids["FR"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
            self.prev_rpm_FR = rpm
            #self.get_logger().info('Publishing RPM FR: "%s"' % can_msg_rpm)

    def swerve_listener_BL(self, msg):

        turn_amount = (msg.data[1]/4 + 180)
        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_BL.data = self.vesc_ids["BL"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle BL: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        if(abs(self.prev_rpm_BL - rpm) < self.max_rpm_change):
            self.can_msg_rpm_BL.data = self.vesc_ids["BL"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
            self.prev_rpm_BL = rpm
            #self.get_logger().info('Publishing RPM BL: "%s"' % can_msg_rpm)

    def swerve_listener_BR(self, msg):
 
        turn_amount = (msg.data[1]/4 + 180)
        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_BR.data = self.vesc_ids["BR"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle BR: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        if(abs(self.prev_rpm_BR - rpm) < self.max_rpm_change):
            self.can_msg_rpm_BR.data = self.vesc_ids["BR"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
            self.prev_rpm_BR = rpm
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
