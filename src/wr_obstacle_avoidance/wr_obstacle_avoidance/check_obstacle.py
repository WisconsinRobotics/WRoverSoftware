import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import time
import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber, TimeSynchronizer
from std_msgs.msg import Bool


class DepthImageSubscriber(Node):
    def __init__(self):
        self.bridge = CvBridge()
        super().__init__('depth_image_subscriber')
        self.depth_sub = Subscriber(self, Image, '/oak/stereo/image_raw')
        self.rgb_sub = Subscriber(self, Image, '/oak/rgb/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.depth_sub, self.rgb_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        self.publisher_ = self.create_publisher(Image, 'modified_depth/image', 100)
        self.publisher_check = self.create_publisher(Bool, 'obstacle_there', 10)
        self.obstacle_there = False

    def sync_callback(self, depth_msg, rgb_msg):
        depths_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        vertical_len = len(depths_image)
        horizontal_len = len(depths_image[0])

        num_sectors = 128

        histogram = np.zeros(num_sectors, dtype=np.float32)
        sector_len = horizontal_len // num_sectors

        for i in range(num_sectors):
            start, end = i * sector_len, (i + 1) * sector_len
            sector = depths_image[:, start:end]
            valid_mask = (sector > 400) & (sector <= 6000)
            depths = (sector[valid_mask] / 1000).astype(np.float32)
            if len(depths) > 0:
                histogram[i] = np.sum(1 / depths ** 2)
            else:
                histogram[i] = np.float32(0)

        histogram_threshold = np.float32(5039)
        #print(h)
        #Check if there is an obstacle in middle half 
        for i in range(int(num_sectors/4), int((3/4)*num_sectors)):
            if histogram[i] > histogram_threshold:
                print(f"section:{i} has an obstacle with value {histogram[i]}") 
                self.obstacle_there = True
                break
            else:
                self.obstacle_there = False
        #Publisher 
        msg = Bool()
        msg.data = self.obstacle_there
        self.publisher_check.publish(msg)

        # Apply green overlay to sectors below threshold
        #for i in range(num_sectors):
        #    if histogram[i] < histogram_threshold:
        #        start, end = i * sector_len, (i + 1) * sector_len
        #        rgb_image[:, start:end, 1] = 255
        #ros_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        #self.publisher_.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)

    depth_image_subscriber = DepthImageSubscriber()

    rclpy.spin(depth_image_subscriber)

    depth_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# header:
#  stamp:
#    sec: 1738889569
#    nanosec: 825827089
#  frame_id: oak_rgb_camera_optical_frame
# height: 720
# width: 1280
# encoding: 16UC1
# is_bigendian: 0
# step: 2560
# data: '<sequence type: uint8, length: 1843200>'
# 921600
