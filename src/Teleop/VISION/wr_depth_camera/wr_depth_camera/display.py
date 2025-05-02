import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
import numpy
import cv2

from cv_bridge import CvBridge



class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera')
        self.subscription_joint_solutions = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.listener_callback,
            10)
            
        # self.arm_position_publisher = self.create_publisher(Float32MultiArray, 'arm_angles', 10)


    def listener_callback(self, data):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #print(data.position)
        rgb_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow('image',rgb_image)
        cv2.waitKey(1)
        print("callback")

def main(args=None):
    rclpy.init(args=args)
    ik_subscriber = CameraSubscriber()
    print("Starting ros2 arm_logic node")
    rclpy.spin(ik_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ik_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


