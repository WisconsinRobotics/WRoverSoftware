import json
import math
import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, Bool
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from math import atan2, radians, degrees, sin, cos, sqrt

# Threshold to consider a waypoint "reached" (meters)
WAYPOINT_THRESHOLD = 1.524  # 5 feet in meters
# Publisher rate (Hz)
CMD_RATE = 10

# Motor command presets
# [y movement (fwd/back), x movement (left/right), swivel_left, swivel_right]
FWD      = [  -1.0,   0.0,  0.0,  0.0]
R90      = [  0.0,   0.0,  1.0, -1.0]
FWD_ROT_90  = [  1.0,   0.0,  0.0,  -1.0]
FWD_ROT_270  = [  1.0,   0.0,  0.0,  1.0]
STOP     = [  0,0,   0.0,  0.0,  0.0]

class WaypointFollower(Node):
    def start_input_thread(self):
        thread = threading.Thread(target=self.user_input_loop)
        thread.daemon = True
        thread.start()

    def user_input_loop(self):
        while rclpy.ok():
            if not self.to_drive:
                user_input = input("To start autonomous driving type START: ")
                if user_input == "START":
                    self.to_drive = True

    def __init__(self):
        super().__init__('waypoint_follower')

        # Read target points
        file_path = os.path.abspath("points.json")
        with open(file_path) as f:
            js = json.load(f)
        
        self.targets = js["targets"]
        self.done = False

        # Set for current gps location and target we need
        self.current_gps = None
        self.target_indx = 0
        self.target_gps = self.targets[self.target_indx]

        # Info for angles and obstacle
        self.compass_angle = None
        self.obstacle_detected = None

        # Control driving
        self.to_drive = True
        self.start_input_thread()

        # Subscribers
        self.create_subscription(Float64, 'compass_data_topic', self.compass_callback, 5)
        self.create_subscription(NavSatFix, 'fix', self.gps_callback, 5)
        self.create_subscription(Bool, 'obstacle_there', self.obstacle_callback, 5)

        # Obstacle checker
        self.obstacle = False

        # Publisher for drive commands
        self.swerve_publisher = self.create_publisher(Float32MultiArray, 'swerve', 1)

        # Timer to run callbacks
        self.create_timer(1.0 / CMD_RATE, self.swerve_callback)

    def obstacle_callback(self, msg: Bool):
        """
        Just gets the data if we have detected the obstacle in the way
        """
        self.obstacle_detected = msg.data

    def gps_callback(self, msg: NavSatFix):
        """
        Update current robot position from GPS.
        Converts lat/lon into local ENU x,y relative to centre.
        """
        lat, lon = msg.latitude, msg.longitude
        self.current_gps = [lat, lon]
        self.get_logger().info(f"Current GPS {self.current_gps}")

    def compass_callback(self, msg: Float64):
        """
        Pigeonhole returns data with negative values, need to refactor it
        to be positive and also it poits to north-west relative to the front wheels
        """
        self.get_logger().info(f"Got angle {msg.data}")
        angle = (90 - msg.data) % 360 
        self.compass_angle = angle

    @staticmethod
    def gps_distance(p1, p2):
        lat1, lon1 = p1
        lat2, lon2 = p2

        dx = (lon2 - lon1) * cos(radians((lat1 + lat2) / 2)) * 111320  # meters
        dy = (lat2 - lat1) * 110540  # meters
        return sqrt(dx*dx + dy*dy)

    @staticmethod
    def compute_bearing(p1, p2):
        """
        Computes the angle between two gps coordinates in degrees

        Args:
            p1 - first gps coordinate
            p2 - second gps coordinate
        Returns:
            angle with respect to north that points into the direction
        """
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Calculate differences in coordinates
        dlon = lon2_rad - lon1_rad

        # Calculate bearing using atan2
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)

        bearing_rad = math.atan2(x, y)

        # Convert bearing from radians to degrees (0° to 360°)
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = (bearing_deg + 360) % 360  # Normalize to 0-360

        return bearing_deg
    
    @staticmethod
    def is_same(p1, p2):
        """
        Determines if two points are close

        Args:
            p1 - first gps coordinate
            p2 - second gps coordinate
        Returns:
            true if they are close and false otherwise
        """
        return WaypointFollower.gps_distance(p1, p2) < 1
    
    def obstacle_callback(self, msg):
        self.obstacle = msg.data

    def swerve_callback(self):
        msg = Float32MultiArray()
        if self.done or (not self.to_drive):
            msg.data = STOP
            self.swerve_publisher.publish(msg)
            self.get_logger().info("Done driving")
            return

        ## Check that both current gps and angles are set up
        if (not self.current_gps) or (not self.compass_angle):
            return

        ## Check that we reached the target
        if WaypointFollower.is_same(self.current_gps, self.target_gps):
            self.target_indx += 1
            if self.target_indx >= len(self.targets):
                self.done = True
                self.get_logger().info(f"Reached the final target {self.target_gps}")
            else:
                self.target_gps = self.targets[self.target_indx]
                ## Make rover stop and wait for the user
                self.to_drive = False
            return
        
        #rover_angle: 
        rover_angle = WaypointFollower.compute_bearing(self.current_gps, self.target_gps)
    
        # This will make an array that goes from 0 to 360. This goes counter clockwise (sorry)
        difference_angle = (self.compass_angle - rover_angle)%360

        ## self.get_logger().info(f"Rover angle: {rover_angle}, compass angle: {self.compass_angle}")
        # if abs(rover_angle - self.compass_angle) < 30:
        #     ## If angles are relatively the same, drive forward
        #     self.get_logger().info("Driving Forward")
        #     msg.data = FWD
        # else:
        #     ## Otherwise, spin
        #     self.get_logger().info("Spinning in place")
        # #
        
        if self.obstacle:
            msg.data = R90
            if difference_angle <= 10 or difference_angle >= 350:
                ## If angles are relatively the same, drive forward
                ##self.get_logger().info("Driving Forward")
                msg.data = FWD
            elif difference_angle < 60 and difference_angle > 10:
                ## If angles are relatively the same, drive forward
                ##self.get_logger().info("Driving Forward")
                msg.data = FWD_ROT_270
            elif difference_angle < 350 and difference_angle > 300:
                msg.data = FWD_ROT_90
            else:
                ## Otherwise, spin
                ## self.get_logger().info("Spinning in place")
                msg.data = R90
        
        self.swerve_publisher.publish(msg)
    
def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()