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

class RailSubscriber(Node):

    def __init__(self):
        super().__init__('arm_logic')

        self.subscription_joy = self.create_subscription(
            Float32MultiArray,
            'rail',
            self.listener_callback_joy,
            10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Define messages beforehand
        self.msg_linear_rail = Float64()
        self.msg_linear_rail.data = 0.0

        self.arm_publisher_base = self.create_publisher(Float64, 'arm_base', 10)

    
    def listener_callback_joy(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        motion = msg.data

        #Expecting (left trigger, rigt trigger)
        linear_rail_speed = self.get_linear_rail_speed(motion[1], motion[0])
        
        #Publishing
        self.msg_linear_rail.data = linear_rail_speed

    def timer_callback(self):        
        self.arm_publisher_base.publish(self.msg_linear_rail)
    
    def get_linear_rail_speed(self, left, right) -> Float64:
        #Reverse if right is positive
        #Converting -1 -> 1 range of triggers to 0->1
        return ((left+1)/2 - (right+1)/2)

def main(args=None):
    rclpy.init(args=args)
    rail_subscriber = RailSubscriber()
    print("Starting ros2 rail_subscriber node")
    rclpy.spin(rail_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rail_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


