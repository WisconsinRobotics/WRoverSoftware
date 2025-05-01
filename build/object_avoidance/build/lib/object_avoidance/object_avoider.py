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

        # Parameters
        self.declare_parameter('crop_frac',     0.15)     # fraction to crop top
        self.declare_parameter('y_thresh',      0.4)      # ground removal (m)
        self.declare_parameter('near_th',       3.5)      # ≤ near_th → “near”
        self.declare_parameter('mid_th',        6.0)      # ≥ mid_th → “far”
        self.declare_parameter('near_override', 0.10)
        self.declare_parameter('num_sectors',   128)

        # Subscribers
        depth_sub = message_filters.Subscriber(self, Image,      '/oak/stereo/image_raw')
        pc_sub    = message_filters.Subscriber(self, PointCloud2, '/oak/points')
        ts = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, pc_sub], queue_size=5, slop=0.05)
        ts.registerCallback(self.cb)

        # Publisher for the overlay
        self.pub = self.create_publisher(Image, 'object_avoidance/overlay', 1)

        # Cache dtype for Y-channel extraction
        self._pc_dtype = None

    def _make_pc_dtype(self, pc_msg: PointCloud2):
        y_field = next(f for f in pc_msg.fields if f.name == 'y')
        return np.dtype({
            'names':   ['y'],
            'formats': [np.float32],
            'offsets': [y_field.offset],
            'itemsize': pc_msg.point_step
        })

    def cb(self, depth_msg: Image, pc_msg: PointCloud2):
        # Decode and crop depth image
        raw_full = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        depth_full = raw_full.astype(np.float32) / 1000.0

        H_full, W = depth_full.shape
        y0 = int(self.get_parameter('crop_frac').value * H_full)
        raw = raw_full[y0:, :]
        depth_m = depth_full[y0:, :]

        # Extract Y channel from point cloud and crop
        if self._pc_dtype is None:
            self._pc_dtype = self._make_pc_dtype(pc_msg)
        arr = np.frombuffer(pc_msg.data, dtype=self._pc_dtype, count=H_full * W)
        y_full = arr['y'].reshape(H_full, W)
        y = y_full[y0:, :]

        # Remove ground (Y > threshold)
        y_thresh = self.get_parameter('y_thresh').value
        mask_ground = (y > y_thresh)
        depth_m[mask_ground] = np.nan
        depth_m[depth_m <= 0] = np.nan

        # Sector classification
        near_th = self.get_parameter('near_th').value
        mid_th  = self.get_parameter('mid_th').value
        nr_ov   = self.get_parameter('near_override').value
        num_sec = self.get_parameter('num_sectors').value
        H, _ = depth_m.shape
        sect_w = W // num_sec

        results = []
        near_mask = (depth_m < near_th)
        mid_mask  = (depth_m >= near_th) & (depth_m < mid_th)
        far_mask  = (depth_m >= mid_th)
        for i in range(num_sec):
            x0 = i * sect_w
            x1 = W if i == num_sec-1 else (i+1) * sect_w
            tot = H * (x1 - x0)
            nf = np.count_nonzero(near_mask[:, x0:x1]) / tot
            mf = np.count_nonzero(mid_mask [:, x0:x1]) / tot
            ff = np.count_nonzero(far_mask [:, x0:x1]) / tot
            if nf > nr_ov * (nf+mf+ff):
                cls = 'near'
            elif mf * 1.2 > ff:
                cls = 'mid'
            elif (nf+mf) > ff:
                cls = 'near' if nf>mf else 'mid'
            else:
                cls = 'far'
            results.append((i, nf, mf, ff, cls))

        # Log details
        self.get_logger().info('Sector | near | mid | far | cls')
        for (i, nf, mf, ff, cls) in results:
            self.get_logger().info(f'{i:>3d}    | {nf:.2f} | {mf:.2f} | {ff:.2f} | {cls}')

        # Build uniform colored overlay bands
        raw8 = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        base = cv2.cvtColor(raw8, cv2.COLOR_GRAY2BGR)
        band_overlay = np.zeros_like(base)
        colors = {'near': (0, 0, 255), 'mid': (0, 255, 255), 'far': (0, 255, 0)}
        for i, _,_,_, cls in results:
            x0 = i * sect_w
            x1 = W if i == num_sec-1 else (i+1) * sect_w
            band_overlay[:, x0:x1] = colors[cls]

        # Blend overlay with base image
        alpha = 0.3
        overlay = cv2.addWeighted(band_overlay, alpha, base, 1-alpha, 0)

        # Publish overlay
        out_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        out_msg.header = depth_msg.header
        self.pub.publish(out_msg)


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

