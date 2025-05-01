#!/usr/bin/env python

import cv2 as cv
import rclpy
import numpy as np
import finding_aruco_tag as aruco_lib
from rclpy.node import Node
import cv_bridge
from sensor_msgs.msg import Image
from custom_msgs.msg import VisionTarget

CAMERA_WIDTH = 1280
def process_corners(target_id: int, corners: np.ndarray) -> VisionTarget:
    """
    Creates a VisionTarget message based on the detected ArUco tag

    @param target_id (int): ArUco tag ID
    @param corners (np.ndarray): ArUco tag corners
    @returns VisionTarget: VisionTarget message defined in msg/VisionTarget.msg
    """
    # Find the middle of the ArUco tag in the frame
    #side_lengths = []
    min_x = corners[0][0]
    max_x = corners[0][0]
    for i in range(len(corners)):
        #side_lengths.append(np.linalg.norm(corners[i - 1] - corners[i]))
        min_x = min(min_x, corners[i][0])
        max_x = max(max_x, corners[i][0])
    x_offset = (min_x + max_x - CAMERA_WIDTH) / 2

    # Estimate the distance of the ArUco tag in meters
    distance_estimate = aruco_lib.estimate_distance_m(corners)

    msg = VisionTarget()
    msg.target_id = int(target_id)
    msg.x = int(x_offset)
    msg.dis = float(distance_estimate)

    return msg

class ArucoDetectionPublisher(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.publisher_ = self.create_publisher(VisionTarget, 'aruco_results', 10)
        self.bridge = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera_data_topic',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        (corners, ids, _) = aruco_lib.detect_aruco(img)
        if ids is not None:
            for i, target_id in enumerate(ids):
                msg = process_corners(target_id[0], corners[i][0])
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publishing {msg.target_id}, {msg.x}, {msg.dis}")
        else:
            self.get_logger().info(f"No aruco tags detected")

        #     # Publish even when no target is found to constantly run the navigation callback
        #     self.publisher_.publish(VisionTarget(0, 0, 0, False))


def main(args=None):
    rclpy.init(args=args)
    aruco_detection_publisher = ArucoDetectionPublisher()

    rclpy.spin(aruco_detection_publisher)

    aruco_detection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
