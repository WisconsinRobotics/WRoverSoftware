#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import message_filters
from cv_bridge import CvBridge
import numpy as np
import cv2

class PC2ColorMask(Node):
    def __init__(self):
        super().__init__('pc2_color_mask')
        self.bridge = CvBridge()

        # RGB + PointCloud2 subscriptions
        rgb_sub = message_filters.Subscriber(self, CameraInfo, '/oak/stereo/camera_info')
        
        ts.registerCallback(self.callback)

        self.pub = self.create_publisher(Image, 'highlighted_pc_mask', 2)
        self.declare_parameter('y_thresh', 0.4)

        # We'll cache this dtype after the first message arrives
        self._pc_dtype = None

    def _make_dtype(self, pc_msg: PointCloud2):
        # Find the offset of the 'y' field
        y_field = next(f for f in pc_msg.fields if f.name == 'y')
        # Create a structured dtype that reads only that float32 at offset y_field.offset
        return np.dtype({
            'names':   ['y'],
            'formats': [np.float32],
            'offsets': [y_field.offset],
            'itemsize': pc_msg.point_step
        })

    def callback(self, rgb_msg: CameraInfo, pc_msg: PointCloud2):
        print(rgb_msg+"\n\n\n")
        out_msg = self.bridge.cv2_to_imgmsg(out, 'bgr8')
        out_msg.header = rgb_msg.header
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PC2ColorMask()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

