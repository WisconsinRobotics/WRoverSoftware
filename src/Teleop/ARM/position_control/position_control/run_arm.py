import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from relaxed_ik_ros2.msg import  EEPoseGoals
from geometry_msgs.msg import Pose, Twist, PoseArray, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from custom_msgs_srvs.action import DrawPath
from std_msgs.msg import Float32MultiArray
import asyncio
from std_msgs.msg import Bool
import threading

class RunArm(Node):
    def __init__(self):
        super().__init__('run_arm_draw')

        #TODO: Figure out what this is: 
        # # QoS profile for reliability
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10
        # )

        #Subscriber
        self.subscription = self.create_subscription(
            Bool,
            '/position_status',
            self.position_callback,
            10
        )

        # Publishers    
        self.pose_publisher = self.create_publisher(EEPoseGoals, '/relaxed_ik/ee_pose_goals', 1)
        self.rail_publisher = self.create_publisher(Float32MultiArray, 'rail', 1)

        self.rail_out_msg = Float32MultiArray()
        self.rail_out_msg.data = [1.0,-1.0,0.0]
        self.rail_in_msg = Float32MultiArray()
        self.rail_in_msg.data = [-1.0,1.0,0.0]
        self.rail_stop = Float32MultiArray()
        self.rail_stop.data = [0.0,0.0,0.0]

        # Timer to publish periodically
        self.publisher_timer = self.create_timer(0.01, self.publish_messages)

        self.x = 0.0
        self.y = 0.0
        self.counter = 0.0
        self.point = Point()
        
        self.feedback_msg = DrawPath.Feedback()
        self.line_index = 0
        self.point_index = 0
        self.lines = []
        self.delay = 0.1  # Adjust the speed (seconds)
        self.reached_point = False
        self.robot_drawing = True
        self.counter = 0
        self.finished = False
        self.reached_first = False

        # Create action server
        self._action_server = ActionServer(
            self,
            DrawPath,
            'draw_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
    def position_callback(self,msg):
        self.reached_point = msg.data
        
    def goal_callback(self, goal_request):
        """Accepts all goals."""
        self.get_logger().info('Received new goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow goal cancellation."""
        self.get_logger().info('Goal canceled.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute a single-line goal."""
        self.get_logger().info('Executing goal...')

        self._goal_handle = goal_handle
        
 
        self.lines = goal_handle.request.lines  # Multiple Lines
        self.point_index = 0

        self.process_timer = self.create_timer(self.delay, self.process_line_feedback)
        goal_handle.succeed()
        result = DrawPath.Result()
   
        return result

    def process_line_feedback(self):
        """Handle feedback for each liarm_anglesne point."""
        if self.point_index < 1 and self.robot_drawing:
            self.rail_publisher.publish(self.rail_out_msg)
            self.counter = self.counter + 1
            if self.counter > 20:
                if self.line_index < len(self.lines):
                    self.point = self.lines[self.line_index].points[self.point_index]
                #self.get_logger().info(f'Processing point {self.point_index}: ({self.point.x}, {self.point.y})')
                #self.get_logger().info("Moving rail back")
                self.x = self.point.x
                self.y = self.point.y
            if self.counter > 40:
                self.robot_drawing = False
                self.counter = 0
                self.rail_publisher.publish(self.rail_stop) #Stops once we move out

        #Marking that we reached the first point
        if self.reached_first == False and self.reached_point and self.robot_drawing == False:
            self.rail_publisher.publish(self.rail_stop) #Making sure rail stops once we move out
            #self.get_logger().info("Setting reached first to true")
            self.reached_first = True
            
        # Move rail in once we reach first point
        if self.reached_first and not self.robot_drawing:
            #self.get_logger().info("Moving rail in now that I have reached first point")
            self.rail_publisher.publish(self.rail_in_msg) 
            self.counter = self.counter + 1
            if self.counter > 40:
                self.robot_drawing = True
                self.counter = 0

        # Move through line now that we have reached first point and have moved in
        if self.robot_drawing and self.reached_first: 
            self.rail_publisher.publish(self.rail_stop) #Making sure rail stops once we move in enough

            if (self.point_index < len(self.lines[self.line_index].points)) and self.line_index < len(self.lines):
                point = self.lines[self.line_index].points[self.point_index]
            self.point_index = self.point_index + 1
            self.point = point
            #self.get_logger().info(f"Drawing line with point: {point}")
            self.x = point.x
            self.y = point.y
            self.rail_publisher.publish(self.rail_stop) #Stop the rail from moving while drawing

        #Reached last point in line
        if self.point_index >= len(self.lines[self.line_index].points):
            self.rail_publisher.publish(self.rail_stop) #Making sure rail stops once we reach the end
            #self.get_logger().info("Reached last point in line")
            self.reached_first = False
            self.line_index = self.line_index + 1
            self.point_index = 0

        #Finished processing lines
        if self.line_index >= len(self.lines):
            #self.get_logger().info("Reached last line")
            self.finshed = True
            self.line_index = 0
            self.point_index = 0
            self.process_timer.cancel()

        # Send feedback
        feedback = DrawPath.Feedback()
        feedback.line_index = 0
        feedback.current_point = self.point
        self._goal_handle.publish_feedback(feedback)

            

    def move_base(self, right):
        pass

    def publish_messages(self):
        msg = EEPoseGoals()
        # Create Pose
        #self.counter = self.counter - .001
        pose = Pose()
        pose.position.x = float(-1.1) + self.x / 2200
        pose.position.y = 0.0
        pose.position.z = float(0.23) - self.y / 2200

        #self.get_logger().info(f'Processing line: x: {pose.position.x}, y: {pose.position.z}')


        # Keep the orientation fixed (90-degree rotation around X-axis)
        pose.orientation.x = 0.0
        pose.orientation.y = math.sqrt(.5)# Equivalent to math.pow(2, (1/2)/2)
        pose.orientation.z = 0.0
        pose.orientation.w = math.sqrt(.5)  # Equivalent to math.pow(2, (1/2)/2)

        # Create Twist
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        msg.ee_poses.append(pose)
        msg.tolerances.append(twist)
        # Publish messages
        self.pose_publisher.publish(msg)

def main(args=None):
    rclpy.init()
    node = RunArm()
    node.get_logger().info("Starting to spin RunArm...")
    rclpy.spin(node)
    

if __name__ == '__main__':
    main()
