#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
import message_filters


class SectorDepthClassifier(Node):

    PIXEL_OFFSET = np.float32(648.040894)
    FOCAL_LENGTH = np.float32(563.33333)
    GAP_THRESHOLD = 2 # The minimum distance between two obstacles such that the rover can fit.

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
        depth_full[mask] = np.nan
        depth_threshold = 2
        H,W = depth_full.shape

        degrees = np.array([i for i in range(-49, 50, 3)])
        pixel_location = np.tan(np.radians(degrees)) * self.FOCAL_LENGTH + self.PIXEL_OFFSET
        
        

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
        gap_list = []
        for x in range(0, W):
            min = 1000
            for y in range(0, H):
                if depth_full[y][x] < min:
                    min = depth_full[y][x]
            if min <= depth_threshold:
                gap_list.append(1)
            else:
                gap_list.append(0)
            min_list.append(min)
        print(min_list)

        # if 0 its no object
        # if 1 its no object
        # 0 followed by 1 means end of object
        # 1 followed by 0 means start of object
        # nothing followed by 0 means start of object
        # 0 followed by nothing means end of object
        gaps = []
        gap = ()
        prev_value = None
        for index, value in enumerate(gap_list):
        
            if index == 0:  # nothing followed by 0 means start of object
                if value == 0:
                    gap = gap + (index,)
                prev_value = value
                continue
        
            if prev_value == 0 and value == 1:  # 0 followed by 1 means end of object
                gap = gap + (index,)
                prev_value = value
                gaps.append(gap)
                gap = ()
                continue
        
            if prev_value == 1 and value == 0:  # 1 followed by 0 means start of object
                gap = gap + (index - 1,)
                prev_value = value
                continue
        
            if index == len(gap_list) - 1 and value == 0:  # 0 followed by nothing means end of object
                gap = gap + (index,)
                gaps.append(gap)
                gap = ()
                continue
        
        thetas = []
        distance_monitor_list = []
        for gap in gaps:
            ux1 = gap[0]
            ux2 = gap[1]
            
            theta1 = np.arctan((ux1 - self.PIXEL_OFFSET)/self.FOCAL_LENGTH) 
            theta2 = np.arctan((ux2 - self.PIXEL_OFFSET)/self.FOCAL_LENGTH)

            d1 = np.cos(theta1)/min_list[ux1]
            d2 = np.cos(theta2)/min_list[ux2]
            
            # Calculating the theta for each gap
            
            theta = theta2 - theta1
            thetas.append(theta)
            gap_distance = np.sqrt(d1**2 + d2**2 - (2*d1*d2*np.cos(theta)))
            distance_monitor_list.append(gap_distance)

        print("theta: ", (np.array(thetas)*180)/3.14)
        print("list of gaps :",gaps)
        print("list of distance between gaps :", distance_monitor_list, "\n\n")
        
        depth_full = cv2.normalize(depth_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_full = cv2.cvtColor(depth_full, cv2.COLOR_GRAY2BGR)

        # raw8 = cv2.normalize(depth_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        # base = cv2.cvtColor(raw8, cv2.COLOR_GRAY2BGR)
                
        for gap in gaps:
            start_point, end_point = (gap[0], 0), (gap[1], 719)
            color = (0, 255, 0)
            depth_full = cv2.rectangle(depth_full, start_point, end_point, color, -1)

            # Publish overlay

        cv2.imshow("obstacle avoidance", depth_full)
        cv2.waitKey(0)
        # out_msg = self.bridge.cv2_to_imgmsg(depth_full, 'bgr8')
        # out_msg.header = depth_msg.header
        # self.pub.publish(out_msg)
        

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

