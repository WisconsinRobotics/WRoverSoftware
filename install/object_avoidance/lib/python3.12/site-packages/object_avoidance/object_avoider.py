#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
import message_filters


class SectorDepthClassifier(Node):
    def __init__(self):
        super().__init__('sector_depth_classifier')
        self.bridge = CvBridge()

        # Subscribers
        self.subscription = self.create_subscription(
            Image,  # Message type
            '/oak/stereo/image_raw',  # Topic name
            self.cb,
            10)  # QoS profile depth
        self.subscription

        # Publisher for the overlay
        self.pub = self.create_publisher(Image, 'object_avoidance/overlay', 1)

    def cb(self, depth_msg: Image, pc_msg: PointCloud2):
        # Decode and crop depth image
        raw_full = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        
        depth_full = raw_full.astype(np.float32) / 1000.0
        mask = (depth_full == 0)
        depth_full[mask] = np.float32(100)
        depth_threshold = 2
        print(depth_full.shape)
        focal_length = np.float32(563.33333)

        degrees = np.array([i for i in range(-49, 50, 3)])
        pixel_location = np.tan(np.radians(degrees)) * focal_length + np.float32(648.040894)
        
        """
        depth_full = (depth_full).astype(np.uint8)
        
        for i in pixel_location:
            start_point, end_point = (round(i), 0), (round(i), 719)
            color = (255, 0, 0)
            thickness = 1
            cv2.line(depth_full, start_point, end_point, color, thickness)

            # Publish overlay
        """
        min_list = []
        for x in range(0, 1079):
            min = depth_full[0][x]
            for y in range(1, 719):
                if depth_full[y][x] < min:
                    min = depth_full[y][x]
            min_list.append(min)
        
        print(min_list)

        """
        out_msg = self.bridge.cv2_to_imgmsg(depth_full)
        out_msg.header = depth_msg.header
        self.pub.publish(out_msg)
        """

def main(args=None):
    rclpy.init(args=args)
    node = SectorDepthClassifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

