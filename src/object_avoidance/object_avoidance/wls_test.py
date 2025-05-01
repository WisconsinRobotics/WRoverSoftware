#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import message_filters
from cv_bridge import CvBridge
import numpy as np
import cv2

class PC2ColorMask(Node):
    def __init__(self):
        super().__init__('pc2_color_mask')
        self.bridge = CvBridge()

        # RGB + PointCloud2 subscriptions
        rgb_sub = message_filters.Subscriber(self, Image,      '/oak/rgb/image_raw')
        pc_sub  = message_filters.Subscriber(self, PointCloud2, '/oak/points')

        ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, pc_sub], queue_size=5, slop=0.05)
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

    def callback(self, rgb_msg: Image, pc_msg: PointCloud2):
        # 1) Convert the RGB image
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')

        H, W = pc_msg.height, pc_msg.width
        n_pts = H * W

        # 2) Build/cache our dtype for extracting 'y'
        if self._pc_dtype is None:
            self._pc_dtype = self._make_dtype(pc_msg)

        # 3) Map the raw data buffer into a 1D array of records
        arr = np.frombuffer(pc_msg.data, dtype=self._pc_dtype, count=n_pts)

        # 4) View the 'y' field as a (H, W) float32 array
        y = arr['y'].reshape(H, W)

        # 5) Build our mask (camera-frame Y positive downward)
        y_thresh = self.get_parameter('y_thresh').value
        mask = (y > y_thresh)

        # 6) Highlight masked pixels in blue
        out = rgb.copy()
        out[mask] = (255, 0, 0)

        # 7) Publish
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

