import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32
import math

COUNTER_MAX = 20

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
        self.max_rpm = 14000
        self.max_rpm_change = 180
        self.acceleration_ceil = 2000.0
        self.limit_rotation = -10
        self.wheels_straight_angle = 180
        self.angle_error_threshold = 1.5
        self.kI = 100
        self.counter_FL = 0
        self.counter_FR = 0
        self.counter_BL = 0
        self.counter_BR = 0
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
        
        # Encoder position subscriptions
        self.subscription_pid_FL = self.create_subscription(
            Float32,
            'pid_FL',
            self.encoder_correction_FL,
            10)
        
        self.subscription_pid_FR = self.create_subscription(
            Float32,
            'pid_FR',
            self.encoder_correction_FR,
            10)
        
        self.subscription_pid_BL = self.create_subscription(
            Float32,
            'pid_BL',
            self.encoder_correction_BL,
            10)
        
        self.subscription_pid_BR = self.create_subscription(
            Float32,
            'pid_BR',
            self.encoder_correction_BR,
            10)

        self.publisher_timer = self.create_timer(0.05, self.publish)

        self.publisher_ = self.create_publisher(String, 'can_msg', 10)

        self.can_msg_rpm_FL = String()
        self.can_msg_rpm_FL.data = self.vesc_ids["FL"][0] + " CAN_PACKET_SET_DUTY " + str(0.0) + " float"
        self.can_msg_angle_FL = String()
        self.can_msg_angle_FL.data = self.vesc_ids["FL"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"

        self.can_msg_rpm_FR = String()
        self.can_msg_rpm_FR.data = self.vesc_ids["FR"][0] + " CAN_PACKET_SET_DUTY " + str(0) + " float"
        self.can_msg_angle_FR = String()
        self.can_msg_angle_FR.data = self.vesc_ids["FR"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"

        self.can_msg_rpm_BL = String()
        self.can_msg_rpm_BL.data = self.vesc_ids["BL"][0] + " CAN_PACKET_SET_DUTY " + str(0) + " float"
        self.can_msg_angle_BL = String()
        self.can_msg_angle_BL.data = self.vesc_ids["BL"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"

        self.can_msg_rpm_BR = String()
        self.can_msg_rpm_BR.data = self.vesc_ids["BR"][0] + " CAN_PACKET_SET_DUTY " + str(0) + " float"
        self.can_msg_angle_BR = String()
        self.can_msg_angle_BR.data = self.vesc_ids["BR"][1] + " CAN_PACKET_SET_POS " + str(0.0) +" float"

        # Encoder corrections
        self.enc_corr_FL = 0.0
        self.enc_corr_FR = 0.0
        self.enc_corr_BL = 0.0
        self.enc_corr_BR = 0.0

        # Encoder corrections collection flags
        self.collected_FL = False
        self.collected_FR = False
        self.collected_BL = False
        self.collected_BR = False

        # Latest raw encoder readings
        self.current_enc_FL = 0.0
        self.current_enc_FR = 0.0
        self.current_enc_BL = 0.0
        self.current_enc_BR = 0.0

        # Motor errors
        self.error_FL = 0.0
        self.error_FR = 0.0
        self.error_BL = 0.0
        self.error_BR = 0.0

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

    def encoder_correction_FL(self, msg):

        # Initial calibration
        if not self.collected_FL:
            self.enc_corr_FL = msg.data
            self.collected_FL = True
        
        corrected = msg.data - self.enc_corr_FL

        # 0 to 360 such that 180 is forward
        self.current_enc_FL = ((corrected + 180) % 360)

    def encoder_correction_FR(self, msg):

        # Initial calibration
        if not self.collected_FR:
            self.enc_corr_FR = msg.data
            self.collected_FR = True
        
        corrected = msg.data - self.enc_corr_FR

        # 0 to 360 such that 180 is forward
        self.current_enc_FR = ((corrected + 180) % 360)

    def encoder_correction_BL(self, msg):
        # Initial calibration
        if not self.collected_BL:
            self.enc_corr_BL = msg.data
            self.collected_BL = True
        
        corrected = msg.data - self.enc_corr_BL

        # 0 to 360 such that 180 is forward
        self.current_enc_BL = ((corrected + 180) % 360)

    def encoder_correction_BR(self, msg):
        # Initial calibration
        if not self.collected_BR:
            self.enc_corr_BR = msg.data
            self.collected_BR = True
        
        corrected = msg.data - self.enc_corr_BR

        # 0 to 360 such that 180 is forward
        self.current_enc_BR = ((corrected + 180) % 360)

    def publish(self):
        
        combined_msg = "\n".join([
        self.can_msg_angle_BR.data,
        self.can_msg_angle_FL.data,
        self.can_msg_angle_FR.data,
        self.can_msg_angle_BL.data,
        self.can_msg_rpm_FL.data,
        self.can_msg_rpm_FR.data,
        self.can_msg_rpm_BL.data,    
        self.can_msg_rpm_BR.data
        ])

        msg = String()
        msg.data = combined_msg

        self.publisher_.publish(msg)

    def swerve_listener_FL(self, msg):
        
        turn_amount = (msg.data[1]/4 + 180)

        # When driving straight, use corrected encoder reading to fix any drift
        if self.collected_FL and turn_amount == self.wheels_straight_angle:
            self.counter_FL += 1
            if(self.counter_FL > COUNTER_MAX):
                if abs(self.wheels_straight_angle - self.current_enc_FL) > self.angle_error_threshold:
                    self.error_FL -= (self.wheels_straight_angle - self.current_enc_FL) / self.kI
                    self.error_FL = math.copysign(min(abs(self.error_FL), 10), self.error_FL)
        else:
            self.counter_FL = 0
        turn_amount += self.error_FL
        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_FL.data = self.vesc_ids["FL"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle FL: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        delta = rpm - self.prev_rpm_FL
        if abs(delta) > self.max_rpm_change and abs(self.prev_rpm_FL) <= self.acceleration_ceil:
            delta = math.copysign(self.max_rpm_change, delta)

        rpm = self.prev_rpm_FL + delta

        self.prev_rpm_FL = rpm
        self.can_msg_rpm_FL.data = self.vesc_ids["FL"][0] + " CAN_PACKET_SET_DUTY " + str(rpm/self.max_rpm) + " float"

    def swerve_listener_FR(self, msg):

        turn_amount = (msg.data[1]/4 + 180)

        # When driving straight, use corrected encoder reading to fix any drift
        if self.collected_FR and turn_amount == self.wheels_straight_angle:
            self.counter_FR += 1
            if(self.counter_FR > COUNTER_MAX):
                if abs(self.wheels_straight_angle - self.current_enc_FR) > self.angle_error_threshold:
                    self.error_FR -= (self.wheels_straight_angle - self.current_enc_FR) / self.kI
                    self.error_FR = math.copysign(min(abs(self.error_FR), 10), self.error_FR)
        else:
            self.counter_FR = 0
        turn_amount += self.error_FR

        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_FR.data = self.vesc_ids["FR"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle FR: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        delta = rpm - self.prev_rpm_FR
        if abs(delta) > self.max_rpm_change and abs(self.prev_rpm_FR) <= self.acceleration_ceil:
            delta = math.copysign(self.max_rpm_change, delta)

        rpm = self.prev_rpm_FR + delta

        self.prev_rpm_FR = rpm
        self.can_msg_rpm_FR.data = self.vesc_ids["FR"][0] + " CAN_PACKET_SET_DUTY " + str(rpm/self.max_rpm) + " float"

    def swerve_listener_BL(self, msg):

        turn_amount = (msg.data[1]/4 + 180)

        # When driving straight, use corrected encoder reading to fix any drift
        if self.collected_BL and turn_amount == self.wheels_straight_angle:
            self.counter_BL += 1
            if(self.counter_BL > COUNTER_MAX):
                if abs(self.wheels_straight_angle - self.current_enc_BL) > self.angle_error_threshold:
                    self.error_BL -= (self.wheels_straight_angle - self.current_enc_BL) / self.kI
                    self.error_BL = math.copysign(min(abs(self.error_BL), 10), self.error_BL)
        else:
            self.counter_BL = 0
        turn_amount += self.error_BL

        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_BL.data = self.vesc_ids["BL"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle BL: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        delta = rpm - self.prev_rpm_BL
        if abs(delta) > self.max_rpm_change and abs(self.prev_rpm_BL) <= self.acceleration_ceil:
            delta = math.copysign(self.max_rpm_change, delta)

        rpm = self.prev_rpm_BL + delta

        self.prev_rpm_BL = rpm
        self.can_msg_rpm_BL.data = self.vesc_ids["BL"][0] + " CAN_PACKET_SET_DUTY " + str(rpm/self.max_rpm) + " float"

    def swerve_listener_BR(self, msg):
 
        turn_amount = (msg.data[1]/4 + 180)

        # When driving straight, use corrected encoder reading to fix any drift
        if self.collected_BR and turn_amount == self.wheels_straight_angle:
            self.counter_BR += 1
            if(self.counter_BR > COUNTER_MAX):
                if abs(self.wheels_straight_angle - self.current_enc_BR) > self.angle_error_threshold:
                    self.error_BR -= (self.wheels_straight_angle - self.current_enc_BR) / self.kI
                    self.error_BR = math.copysign(min(abs(self.error_BR), 10), self.error_BR)
        else:
            self.counter_BR = 0
        turn_amount += self.error_BR
        self.get_logger().error("Current " + str(self.current_enc_BR))
        self.get_logger().error("turn_amount " + str(turn_amount))
        self.get_logger().error("ERROR " + str(self.error_BR))
        if turn_amount < 135 + self.limit_rotation or turn_amount > 225 - self.limit_rotation:
            self.get_logger().error("SENT INCORRECT ANGLE OF " + str(turn_amount) + ". Has to be between 135-225")
        else:
            self.can_msg_angle_BR.data = self.vesc_ids["BR"][1] + " CAN_PACKET_SET_POS " + str(turn_amount) +" float"
            #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
            #self.get_logger().info('Publishing Angle BR: "%s"' % can_msg_angle)
        
        rpm = msg.data[0] * self.max_rpm
        delta = rpm - self.prev_rpm_BR
        if abs(delta) > self.max_rpm_change and abs(self.prev_rpm_BR) <= self.acceleration_ceil:
            delta = math.copysign(self.max_rpm_change, delta)
        
        rpm = self.prev_rpm_BR + delta

        self.prev_rpm_BR = rpm
        self.can_msg_rpm_BR.data = self.vesc_ids["BR"][0] + " CAN_PACKET_SET_DUTY " + str(rpm/self.max_rpm) + " float"


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
