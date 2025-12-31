import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
#from custom_msgs_srvs.msg import GripperPosition
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math

L1 =  22.5 / 39.37 #Convert inchest to meters
L2 = 20 / 39.37 #Convert inchest to meters

class ArmLogic(Node):

    def __init__(self):
        super().__init__('arm_logic_vel')
        self.subscription_joy = self.create_subscription(
            Float32MultiArray,
            'joy_arm',
            self.listener_callback_joy,
            10)
        
        
        self.arm_position_publisher = self.create_publisher(Float32MultiArray, 'arm_angles', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.x = L2
        self.y = L1

        self.arm_angles = [0.0, 0.0, 0.0] #q1 (shoulder angle), q2 (elbow angle). These are in absolute ouput
        self.arm_positions = [0.0, 0.0] #q1 (shoulder angle), q2 (elbow angle). These are in radian ouput

    
    #Put publishers in timer to limit rate of publishing
    def timer_callback(self):
        self.get_position_arm()
        msg = Float32MultiArray()
        msg.data = self.arm_angles
        #print(msg)
        self.arm_position_publisher.publish(msg)
        self.get_logger().info(f"q1: {self.arm_positions[0]}. q2: {self.arm_positions[1]}")


    def listener_callback_joy(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if(math.sqrt(self.x*self.x+self.y*self.y) < (L1+L2)):
            self.y += msg.data[0]/1500 #Moving UP and DOWN
            self.x += msg.data[1] /1500 #Moving side to side
        
        #self.get_logger().info(f"x: {self.x}. y: {self.y}")
        self.ik_solver(self.x, self.y, 0, 0, L1, L2, 0)
        

    def get_position_arm(self):
        #105 rotations per 90 degrees of turn was checked visually
        #Shoulder
        self.arm_angles[0] = self.arm_positions[0] *(105.0 / (math.pi/2))

        #Elbow
        self.arm_angles[1] = (self.arm_positions[1]) *(105.0 / (math.pi/2))
        
    
    def ik_solver(self,x, y, z, theta, L1, L2, L4):
        x4 = x
        y4 = y
        #z4 = z

        #qbase = z4

        #We will just do IK into the base of the wrist
        x3 = x4 #- L4 * sin(theta)
        y3 = y4 #+ L4 * cos(theta)

        q1 = math.atan2(y3,x3) + math.acos((L1*L1 + x3*x3 + y3*y3 - L2*L2) / (2 * L1 * math.sqrt(x3*x3 + y3*y3)))
        q2 = math.acos((x3*x3 + y3*y3 - L1*L1 - L2*L2)/ (2 * L1 * L2))
        #q3 = theta - q1 + q2
        q2 =  -math.pi/2+q2
        q1 = -math.pi/2+q1
        self.arm_positions[0] = q1
        self.arm_positions[1] = q2
        #self.get_logger().info(f"q1: {self.arm_positions[0]}. q2: {self.arm_positions[1]}")

        
    


def main(args=None):
    rclpy.init(args=args)
    swerve_subscriber = ArmLogic()
    print("Starting ros2 arm_logic node")
    rclpy.spin(swerve_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



