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

from custom_msgs_srvs.action import ObjectDetection
from custom_msgs_srvs.msg import VisionTarget
from custom_msgs_srvs.msg import Detection

# Threshold to consider a waypoint "reached" (meters)
WAYPOINT_THRESHOLD = 1.524  # 5 feet in meters
# Publisher rate (Hz)
CMD_RATE = 10

# Motor command presets
# [y movement (fwd/back), x movement (left/right), swivel_left, swivel_right]
FWD      = [  -1.0,   0.0,  0.0,  0.0]
R90      = [  0.0,   0.0,  1.0, -1.0]
R270      = [  0.0,   0.0,  -1.0, 1.0]
FWD_ROT_90  = [  1.0,   0.0,  0.0,  -1.0]
FWD_ROT_270  = [  1.0,   0.0,  0.0,  1.0]
STOP     = [0.0,   0.0,  0.0,  0.0]

class ObjectDetectionClass(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_server = ActionServer(
            self,
            ObjectDetection,
            'object_detection',
            self.execute_action)
        
        self.create_subscription(VisionTarget, 'aruco_results', self.aruco_feedback, 5)
        self.create_subscription(Detection, 'object_there', self.object_feedback, 5)

        # Publisher for drive commands
        self.swerve_publisher = self.create_publisher(Float32MultiArray, 'swerve', 1)
        self.aruco_found = False
        self.object_found = False
        

    def aruco_feedback(self, msg):
        self.aruco_id = msg.target_id
        self.aruco_x =  msg.x
        self.aruco_distance =  msg.dis

    def object_feedback(self, msg):
        self.object_x = msg.x
        self.object_distance = msg.dis

    def execute_action(self, goal_handle):
        self.get_logger().info('Executing navigation goal ...')
        self.type = goal_handle.request.type
        feedback_msg = ObjectDetection.Feedback()
        msg = Float32MultiArray()
        #Initialize values
        self.get_logger().info('INIT VALUES ----------------')
        self.aruco_id = -1
        self.aruco_x =  0
        self.aruco_distance =  -1

        self.object_x = 0
        self.object_distance = -1

        rate = .1 # 1/.1 = 10 times per second
        self.get_logger().info('Executing object detection action')
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = ObjectDetection.Result()
                result.reached_tag = False
                return result
            
            if self.type == 1: #Aruco tag
                
                feedback_msg.found_tag = self.aruco_found
                
                if self.aruco_found == False:
                    if self.aruco_distance > 0:
                        self.aruco_found = True
                if (self.aruco_found):
                    if self.aruco_distance > 2.5:
                        if self.aruco_x > -50: #TODO: make sure middle is -100
                            msg.data = R90
                        elif self.aruco_x < -150:
                            msg.data = R270
                        else:
                            msg.data = FWD
                    else:
                        msg.data = STOP
                        self.swerve_publisher.publish(msg)
                        result = ObjectDetection.Result()
                        result.reached_tag = True
                        return result
                else:
                    msg.data = R90

            else: #Object
                feedback_msg.found_tag = self.object_found
                
                if self.object_found == False:
                    if self.object_distance  > 0:
                        self.object_found = True
                if (self.object_found):
                    if self.object_distance > 2.5:
                        if self.object_x > 500:
                            msg.data = R90
                        elif self.object_x < -500:
                            msg.data = R270
                        else:
                            msg.data = FWD
                    else:
                        msg.data = STOP
                        self.swerve_publisher.publish(msg)
                        result = ObjectDetection.Result()
                        result.reached_tag = True
                        return result
                else:
                    msg.data = R90
            
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Publish command
            self.swerve_publisher.publish(msg)
 

def main():
    rclpy.init()
    node = ObjectDetectionClass()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()