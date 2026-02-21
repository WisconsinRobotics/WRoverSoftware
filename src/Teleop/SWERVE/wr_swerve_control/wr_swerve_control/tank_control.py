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
        self.vesc_ids = {"FL":["70"],
                         "FR":["72"],
                         "BL":["74"],
                         "BR":["76"]
                        }
        self.max_rpm = 6000
        self.limit_rotation = 0
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'tank',
            self.listener_callback,
            10)
        
        self.motion = [0.0, 0.0] #Tank input

        # Timer to publish periodically
        self.publisher_timer_FR = self.create_timer(0.01, self.publish_FR)
        self.publisher_timer_FL = self.create_timer(0.01, self.publish_FL)
        self.publisher_timer_BR = self.create_timer(0.01, self.publish_BR)
        self.publisher_timer_BL = self.create_timer(0.01, self.publish_BL)

        self.publisher_ = self.create_publisher(String, 'can_msg', 10)


    def listener_callback(self, msg):
        #Should receive [left, right]
        self.motion = msg.data
    

    def publish_FR(self):
        can_msg_rpm = String()
        rpm = self.motion[1] * self.max_rpm
        can_msg_rpm.data = self.vesc_ids["FR"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    def publish_FL(self):
        can_msg_rpm = String()
        rpm = self.motion[0] * self.max_rpm
        can_msg_rpm.data = self.vesc_ids["FL"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    def publish_BR(self):
        can_msg_rpm = String()
        rpm = self.motion[1] * self.max_rpm
        can_msg_rpm.data = self.vesc_ids["BR"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    def publish_BL(self):
        can_msg_rpm = String()
        rpm = self.motion[0] * self.max_rpm
        can_msg_rpm.data = self.vesc_ids["BL"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        self.get_logger().info('Publishing RPM BR: "%s"' % can_msg_rpm)

    

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
