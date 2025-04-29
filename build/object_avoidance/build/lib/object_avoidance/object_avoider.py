#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2


class SectorDepthClassifier(Node):
    def __init__(self):
        super().__init__('sector_depth_classifier')
        self.bridge = CvBridge()
        # Parameters
        self.near_th = 3.5  # ≤ this = “near”
        self.mid_th = 6.0  # ≥ mid_th = “far”
        self.near_override = 0.10  # as a percentage
        self.num_sectors = 128
        self.crop_frac = 0.16  # fraction to cut top/bottom

        self.sub = self.create_subscription(
            Image, '/oak/stereo/image_raw', self.cb, 2)
        self.pub = self.create_publisher(
            Image, 'object_avoidance/info', 2)

    def cb(self, msg: Image):
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        
        #median filtering
        raw = cv2.medianBlur(raw, ksize=5)
        # depth in meters
        depth = (raw.astype(np.float32) / 1000.0)
    
        H, W = depth.shape
        y0 = int(self.crop_frac * H)
        y1 = H - y0
        depth = depth[y0:y1, :]  # crop
        raw_crop = raw[y0:y1, :]
        
        depth[depth == 0] = np.nan  # Set invalid readings to NaN
        # Build masks
        near_mask = (depth < self.near_th)
        mid_mask = (depth >= self.near_th) & (depth < self.mid_th)
        far_mask = (depth >= self.mid_th)

        # Compute per-sector fractions
        sect_w = W // self.num_sectors
        results = []
        for i in range(self.num_sectors):
            x0 = i * sect_w
            x1 = W if i == self.num_sectors - 1 else (i + 1) * sect_w
            tot = depth.shape[0] * (x1 - x0)

            nf = np.count_nonzero(near_mask[:, x0:x1]) / tot
            mf = np.count_nonzero(mid_mask[:, x0:x1]) / tot
            ff = np.count_nonzero(far_mask[:, x0:x1]) / tot

            # Classify "near" = 0, mid = 1, far = 2
            if nf > self.near_override * (nf+mf+ff):
                cls = "near"
            elif mf * 1.2 > ff * 1.0:
                cls = "mid"
            elif nf+mf > ff:
                if nf > mf:
                    cls = "near"
                else:
                    cls = "mid"
            else:
                cls = "far"

            #results.append(cls)
            
            
            results.append((i, nf, mf, ff, cls))
        """    
        arr = Int32MultiArray()
        arr.data = results
        self.pub.publish(arr)
        """
        
        # Log table
        self.get_logger().info('Sector | near | mid | far | cls')
        for (i, nf, mf, ff, cls) in results:
            self.get_logger().info(
                f'{i:>3d}    | {nf:.2f} | {mf:.2f} | {ff:.2f} | {cls}'
            )

        # Draw overlay on original
        raw8 = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        overlay = cv2.cvtColor(raw8, cv2.COLOR_GRAY2BGR)
        # crop box for context
        cv2.rectangle(overlay, (0, y0), (W, y1), (255, 0, 0), 2)

        colors = {'near': (0, 0, 255), 'mid': (0, 255, 255), 'far': (0, 255, 0)}
        for i, nf, mf, ff, cls in results:
            x0 = i * sect_w
            x1 = W if i == self.num_sectors - 1 else (i + 1) * sect_w
            color = colors[cls]
            cv2.rectangle(overlay, (x0, y0), (x1, y1), color, 2)
            cv2.putText(overlay, cls[0].upper(),
                        (x0 + 5, y0 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2, cv2.LINE_AA)

        # 8) Publish
        out_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        self.pub.publish(out_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = SectorDepthClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

