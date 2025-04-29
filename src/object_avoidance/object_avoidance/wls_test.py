import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv2.ximgproc as ximgproc
import numpy as np
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

class WLSFilterNode(Node):
    def __init__(self):
        super().__init__('wls_filter_node')
        self.bridge = CvBridge()

        # Set up synchronized subscribers for left and right images
        self.left_sub = Subscriber(self, Image, "/oak/left/image_raw")
        self.right_sub = Subscriber(self, Image, "/oak/right/image_raw")
        self.ats = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=5, slop=0.1)
        self.ats.registerCallback(self.stereo_callback)

        # Publisher for the filtered disparity image
        self.pub_filtered = self.create_publisher(Image, "/stereo/filtered_disparity", 10)

        # Stereo matcher parameters
        self.num_disparities = 96  # Must be divisible by 16
        self.block_size = 15       # Typical values: 15-21

        # Create left matcher (StereoBM for speed)
        self.left_matcher = cv2.StereoBM_create(numDisparities=self.num_disparities, blockSize=self.block_size)
        # Create right matcher using OpenCV's helper function
        self.right_matcher = ximgproc.createRightMatcher(self.left_matcher)

        # Create the WLS filter using the left matcher
        self.wls_filter = ximgproc.createDisparityWLSFilter(self.left_matcher)
        # Tune WLS parameters: lambda controls smoothness, sigma controls edge preservation
        self.wls_filter.setLambda(2000.0)
        self.wls_filter.setSigmaColor(1.0)

    def stereo_callback(self, left_msg, right_msg):
        # Convert incoming ROS Image messages to OpenCV images
        try:
            left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error("CV Bridge conversion failed: " + str(e))
            return

        # Convert to grayscale (required for stereo matching)
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # Compute disparity maps
        # StereoBM returns disparity multiplied by 16 (fixed-point representation)
        left_disp = self.left_matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
        right_disp = self.right_matcher.compute(right_gray, left_gray).astype(np.float32) / 16.0

        # Apply the WLS filter. The filter uses the left disparity,
        # the left image (as guidance) and the right disparity.
        filtered_disp = self.wls_filter.filter(left_disp, left_gray, None, right_disp)

        # (Optional) Normalize disparity for visualization purposes
        filtered_disp_vis = cv2.normalize(filtered_disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        filtered_disp_vis = np.uint8(filtered_disp_vis)

        # Convert the filtered disparity image back to a ROS Image message and publish it
        out_msg = self.bridge.cv2_to_imgmsg(filtered_disp_vis, encoding="mono8")
        self.pub_filtered.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WLSFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

