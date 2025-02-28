import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from custom_msgs_srvs.msg import GripperPosition
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math
from sensor_msgs.msg import JointState

class IKSubscriber(Node):

    def __init__(self):
        super().__init__('arm_logic')
        self.subscription_joy = self.create_subscription(
            JointState,
            '/relaxed_ik/joint_angle_solutions',
            self.listener_callback,
            10)
            
        self.arm_position_publisher = self.create_publisher(Float32MultiArray, 'arm_angles', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.arm_angles = [0.0, 0.0, 50.0]

    def listener_callback(self, data):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #print(data.position)
        self.processPositions(data.position)

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.arm_angles
        print(msg)
        self.arm_position_publisher.publish(msg)

    def processPositions(self, arm_positions):
        #Shoulder
        self.arm_angles[0] = arm_positions[0] *(-105.0 / (math.pi/2))

        #Elbow
        self.arm_angles[1] = -arm_positions[1] *(105.0 / (math.pi/2))

        #Gripper
        self.arm_angles[2] = (-arm_positions[2]* (50.0 / (math.pi/2))) + 50

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


