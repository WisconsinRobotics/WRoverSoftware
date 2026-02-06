import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import math

class SwerveControlSubsrciber(Node):

    def __init__(self):
        super().__init__('science_control')

        # CHECK (later)
        self.vesc_ids = {"NEO_DRILL_ROT":"51",
                         "NEO_CAROUSEL":"52",
                         "DC":"50"
                        }
        self.max_rpm = 3000
        self.limit_rotation = 0
                
        self.subscription_science_xbox = self.create_subscription(
            Float32MultiArray,
            'science_xbox',
            self.science_xbox_listener,
            10)

        self.publisher_ = self.create_publisher(String, 'can_msg', 10)


        self.subscription_science_xbox
        self.get_logger().info("Started SCIENCE NODE")
                

    # leave this for monday
    def science_xbox_listener(self, msg):
        can_msg_rot = String()
        can_msg_updown = String()
        can_msg_carousel = String()

        rot_rpm = msg.data[1] * self.max_rpm	
        #can_msg_rot.data = self.vesc_ids["NEO_DRILL_ROT"] + " CAN_PACKET_SET_RPM " + str(rot_rpm) +" float"
        can_msg_rot.data = "51 CAN_PACKET_SET_RPM " + str(rot_rpm) +" float"
        self.publisher_.publish(can_msg_rot)
        #self.get_logger().info('Publishing Drill Rotation RPM: "%s"' % can_msg_rot)
        
        # DC
        updown_rpm = msg.data[0] * self.max_rpm	
        #can_msg_updown.data = self.vesc_ids["DC"] + " CAN_PACKET_SET_RPM " + str(updown_rpm) +" float"
        can_msg_updown.data = "50 CAN_PACKET_SET_RPM " + str(updown_rpm) +" float"
        self.publisher_.publish(can_msg_updown)
        #self.get_logger().info('Publishing Drill UpDown RPM: "%s"' % can_msg_updown)
        
        rotational_velocity = -((float(msg.data[2]) + 1) / 2.0) + ((float(msg.data[3]) + 1.0) / 2.0)
        carousel_rpm = rotational_velocity * self.max_rpm
        can_msg_carousel.data = self.vesc_ids["NEO_CAROUSEL"] + " CAN_PACKET_SET_RPM " + str(carousel_rpm) + " float"
        self.publisher_.publish(can_msg_carousel)
        #self.get_logger().info('Publishing Carousel ROM: "%s"' % can_msg_carousel)

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

