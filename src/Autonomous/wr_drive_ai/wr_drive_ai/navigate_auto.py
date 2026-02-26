import json
import math
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, Bool
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from math import atan2, radians, degrees, sin, cos, sqrt
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from custom_msgs_srvs.action import Navigation

# Threshold to consider a waypoint "reached" (meters)
WAYPOINT_THRESHOLD = 1.524  # 5 feet in meters
# Publisher rate (Hz)
CMD_RATE = 10

# Motor command presets
# [y movement (fwd/back), x movement (left/right), swivel_left, swivel_right]
FWD      = [  1.0,   0.0,  0.0,  0.0]
R90      = [  0.0,   0.0,  1.0, -1.0]
R270      = [  0.0,   0.0,  -1.0, 1.0]
FWD_ROT_90  = [ 0.5,   0.0,  0.0,  -1.0]
FWD_ROT_270  = [  0.5,   0.0,  0.0,  1.0]
STOP     = [  0,0,   0.0,  0.0,  0.0]

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_server = ActionServer(
            self,
            Navigation,
            'navigate',
            self.execute_action)

        # Set for current gps location 
        self.current_gps = [0.0, 0.0]

        # Info for angles and obstacle
        self.compass_angle = 0.0
        self.obstacle_detected = False

        # Control driving
        self.to_drive = True
        #self.start_input_thread()

        # Subscribers
        self.create_subscription(Float64, 'compass_data_topic', self.compass_callback, 5)
        self.create_subscription(NavSatFix, 'fix', self.gps_callback, 5)
        self.create_subscription(Bool, 'obstacle_there', self.obstacle_callback, 5)

        # Obstacle checker
        self.obstacle = False

        # Publisher for drive commands
        self.swerve_publisher = self.create_publisher(Float32MultiArray, 'swerve', 1)


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
        #self.get_logger().info(f"Got lat: {lat}  long: {lon}")
        self.current_gps = [lat, lon]

    def compass_callback(self, msg: Float64):
        """
        Pigeonhole returns data with negative values, need to refactor it
        to be positive and also it poits to north-west relative to the front wheels
        """
        #self.get_logger().info(f"Got angle {msg.data}")
        angle = ((-msg.data -111) % 360 )
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
        
        return WaypointFollower.gps_distance(p1, p2) < 2
    
    def obstacle_callback(self, msg):
        self.obstacle = msg.data

    def execute_action(self, goal_handle):
        self.get_logger().info('Executing navigation goal ...')
        self.target_gps = goal_handle.request.points
        feedback_msg = Navigation.Feedback()
        msg = Float32MultiArray()

        rate = .1 # 1/.1 = 10 times per second

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = Navigation.Result()
                result.reach_target = False
                return result
            
            # # Safety check: don't drive if not activated
            # if not self.to_drive:
            #     self.get_logger().info("Autonomous driving paused.")
            #     msg.data = STOP
            #     self.swerve_publisher.publish(msg)
            #     continue

            # Ensure sensor data is valid

            # if not self.current_gps or self.compass_angle is None:
            #     self.get_logger().warn("Waiting for GPS and compass data...")
            #     rate.sleep()
            #     continue
            
            #self.get_logger().info(f"Current GPS {self.current_gps}")
            # Feedback to action server
            feedback_msg.distance_away = self.gps_distance(self.current_gps, self.target_gps)
            feedback_msg.target_gps = self.target_gps
            feedback_msg.current_gps = self.current_gps
            goal_handle.publish_feedback(feedback_msg)

            # Compute angles
            rover_angle = WaypointFollower.compute_bearing(self.current_gps, self.target_gps)
            difference_angle = (self.compass_angle - rover_angle) % 360
            #self.get_logger().info("Rover angle: " + str(rover_angle))
            #self.get_logger().info("Comapass angle: " + str(self.compass_angle))
            self.get_logger().info("Difference angle: " + str(difference_angle))

            # Decide movement
            if not self.obstacle:
                if difference_angle <= 10 or difference_angle >= 350:
                    msg.data = FWD
                elif 10 < difference_angle < 60:
                    msg.data = FWD_ROT_90
                elif 300 < difference_angle < 350:
                    msg.data = FWD_ROT_270
                elif 180 < difference_angle < 300:
                    msg.data = R270
                else:
                    msg.data = R90
            else:
                msg.data = R90 if difference_angle <= 180 else R270
            #self.get_logger().info("Rotating: " + str(msg.data))
            # Publish command
            self.swerve_publisher.publish(msg)
            #self.get_logger().info('Target GPS: ' + str(self.target_gps))
            #self.get_logger().info('Current GPS: ' + str(self.current_gps))
            self.get_logger().info('Distance away: ' + str(WaypointFollower.gps_distance(self.current_gps, self.target_gps)))
            # Check for goal completion
            if WaypointFollower.is_same(self.current_gps, self.target_gps):
                self.get_logger().info('Succeeded goal ...')
                goal_handle.succeed()
                result = Navigation.Result()
                result.reach_target = True
                return result
 

def main():
    rclpy.init()
    node = WaypointFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()