#!/usr/bin/env python3
"""
follower_node.py
----------------
Simple ROS2 node that follows a list of waypoints in local ENU (meters)
and publishes swerve drive commands to `/swerve_motor`.

Workflow:
 1. Subscribe to `/search_waypoints` (nav_msgs/Path) for a list of poses (x,y).
 2. Subscribe to `/odom` (nav_msgs/Odometry) and `/gps/fix` (sensor_msgs/NavSatFix) for current position.
 3. On a 10 Hz timer, compute distance to current target waypoint:
     - If within a threshold (5 ft ≈ 1.524 m), advance to next waypoint.
     - Else publish a movement command: move toward the target using presets.

Command message: std_msgs/Float32MultiArray with data [y, x, swivel_left, swivel_right], floats in [-1,0,1].

Ensure you have built and sourced your ROS2 workspace, and that `/search_waypoints` publishes a Path.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry

# import helper for lat/lon → local ENU conversion
from swerve_autonomy.path_utils import ll_to_xy

# Threshold to consider a waypoint "reached" (meters)
WAYPOINT_THRESHOLD = 1.524  # 5 feet in meters
# Publisher rate (Hz)
CMD_RATE = 10

# Motor command presets
# [y movement (fwd/back), x movement (left/right), swivel_left, swivel_right]
FWD      = [  1,   0,  0,  0]
BWD      = [ -1,   0,  0,  0]
STRAFE_R = [  0,   1,  0,  0]
STRAFE_L = [  0,  -1,  0,  0]
R90      = [  0,   0,  1, -1]
L90      = [  0,   0, -1,  1]
# LEFT_90  = [  0,   0,  1,  0]
DONUT    = [0.5, -0.5,0.2,  0]
STOP     = [  0,   0,  0,  0]

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Buffer for incoming waypoints
        self.waypoints = []  # list of (x, y) tuples
        self.current_index = 0

        # Latest pose
        self.cur_x = None
        self.cur_y = None

        # Subscribers
        self.create_subscription(Path, '/search_waypoints', self.path_cb, 1)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 5)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 5)

        # Publisher for drive commands
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/swerve_motor', 1)

        # Timer to run control loop
        self.create_timer(1.0 / CMD_RATE, self.control_loop)

    def gps_cb(self, msg: NavSatFix):
        """
        Update current robot position from GPS.
        Converts lat/lon into local ENU x,y relative to centre.
        """
        ex, ny = ll_to_xy(CENTRE_LAT, CENTRE_LON, msg.latitude, msg.longitude)
        self.cur_x, self.cur_y = ex, ny

    def path_cb(self, msg: Path):
        """
        Callback when a new Path of waypoints arrives.
        Extract x,y from each PoseStamped.
        """
        self.waypoints = [(pose.pose.position.x,
                           pose.pose.position.y)
                          for pose in msg.poses]
        self.current_index = 0
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

    def odom_cb(self, msg: Odometry):
        """
        Update current robot position from odometry.
        """
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

    def control_loop(self):
        """
        Runs at CMD_RATE Hz. Computes and publishes drive commands.
        """
        # Need a valid pose
        if self.cur_x is None or self.cur_y is None:
            return
        # Need waypoints
        if not self.waypoints or self.current_index >= len(self.waypoints):
            self.publish_cmd(STOP)
            return

        # Current target
        tx, ty = self.waypoints[self.current_index]
        dx = tx - self.cur_x
        dy = ty - self.cur_y
        dist = math.hypot(dx, dy)

        # Check if we reached this waypoint
        if dist < WAYPOINT_THRESHOLD:
            self.get_logger().info(
                f"Reached waypoint {self.current_index} at ({tx:.2f}, {ty:.2f}).")
            self.current_index += 1
            return

        # Movement logic: move along dominant axis
        if abs(dx) > abs(dy):
            cmd = STRAFE_R if dx > 0 else STRAFE_L
        else:
            cmd = FWD if dy > 0 else BWD

        self.publish_cmd(cmd)

    def publish_cmd(self, cmd_list):
        """
        Wrap a list of 4 numbers into Float32MultiArray and publish.
        """
        msg = Float32MultiArray(data=cmd_list)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
