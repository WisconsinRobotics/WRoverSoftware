import rclpy
import cv2
from rclpy.node import Node
#from rclpy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import cv_bridge


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_data')
        self.publisher_ = self.create_publisher(Image, 'camera_data_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = cv_bridge.CvBridge()

        self.cam = cv2.VideoCapture(0)
        if not self.cam.isOpened():
            self.get_logger().error('camera not opened. fix it. bye')
        #q, r = self.cam.read()
        #self.get_logger().info(str(q) + str(r.shape) + str(r.dtype))

    def timer_callback(self):
        ret, img = self.cam.read()
        if not ret:
            self.get_logger().warn('something wrong with read')
        
        msg = self.bridge.cv2_to_imgmsg(img, encoding = 'rgb8')
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing camera data')

    def camera_shut_down(self):
        self.cam.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.camera_shut_down()
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
