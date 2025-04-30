import math
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

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
LEFT_90  = [  0,   0,  1,  0]
DONUT    = [0.5, -0.5,0.2,  0]
STOP     = [  0,   0,  0,  0]

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Read target points
        with open("points.json") as f:
            js = json.loads(f)
        
        self.targets = js["targets"]

        # Set for current gps location and target we need
        self.current_gps = None
        self.target_indx = 0
        self.target_gps = self.targets[self.target_indx]

        # Info for angles
        self.compass_angle = None

        # Subscribers
        self.create_subscription(Float64, 'compass_data_topic', self.compass_callback, 5)
        self.create_subscription(NavSatFix, 'fix', self.gps_callback, 5)

        # Publisher for drive commands
        self.cmd_pub = self.create_publisher(Float32MultiArray, 'swerve', 1)

        # Timer to run callbacks
        self.create_timer(1.0 / CMD_RATE, self.compass_callback)
        self.create_timer(1.0 / CMD_RATE, self.gps_callback)
        self.create_timer(1.0 / CMD_RATE, self.swerve_callback)

    def gps_callback(self, msg: NavSatFix):
        """
        Update current robot position from GPS.
        Converts lat/lon into local ENU x,y relative to centre.
        """
        x, y 


    def compass_callback(self, msg: Float64):
        """
        Callback when a new Path of waypoints arrives.
        Extract x,y from each PoseStamped.
        """
        pass        


    def swerve_callback(self):
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
