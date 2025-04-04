import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from relaxed_ik_ros2.msg import  EEPoseGoals
from geometry_msgs.msg import Pose, Twist, PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from custom_msgs_srvs.action import DrawPath
import asyncio
from std_msgs.msg import Bool

class EEPublisher(Node):
    def __init__(self):
        super().__init__('ee_publisher')

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

        # Timer to publish periodically
        self.timer = self.create_timer(0.01, self.publish_messages)

        self.x = 0.0
        self.y = 0.0
        self.counter = 0.0
        
        self.feedback_msg = DrawPath.Feedback()
        self.line_index = 0
        self.point_index = 0
        self.lines = []
        self.delay = 0.1  # Adjust the speed (seconds)
        self.timer = None  # Timer will be created later

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
        self.reached_first = msg.data
        
    def goal_callback(self, goal_request):
        """Accepts all goals."""
        self.get_logger().info('Received new goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow goal cancellation."""
        self.get_logger().info('Goal canceled.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a single-line goal."""
        self.get_logger().info('Executing goal...')
        line = goal_handle.request.line[0]
        first_point = line.points[0]

        self.x = first_point.x
        self.y = first_point.y

        self.get_logger().info('Waiting to reach first point...')

        # Wait in a non-blocking way
        while not self.reached_first:
            await asyncio.sleep(0.1)

        self.get_logger().info('First point reached! Executing...')
        

        self._goal_handle = goal_handle
        self.result = DrawPath.Result()

        self.line = goal_handle.request.line  # Only one Line in the list
        self.point_index = 0

        self._result_future = asyncio.get_event_loop().create_future()

        self.timer = self.create_timer(self.delay, self.process_points)

        return await self._result_future

    def process_line_feedback(self):
        """Handle feedback for each line point."""
        if self.point_index < len(self.line.points):
            point = self.line.points[self.point_index]
            self.get_logger().info(f'Processing point {self.point_index}: ({point.x}, {point.y})')
            self.x = point.x
            self.y = point.y
            # Send feedback
            feedback = DrawPath.Feedback()
            feedback.line_index = 0
            feedback.current_point = point
            self._goal_handle.publish_feedback(feedback)

            self.point_index += 1
        else:
            self.get_logger().info('Line fully processed.')
            self.timer.cancel()
            self.move_base(1)
            self._goal_handle.succeed()
            self.result.result = "Line processed successfully!"
            self._result_future.set_result(self.result)

    def move_base(self, right):
        pass

    def publish_messages(self):
        msg = EEPoseGoals()
        # Create Pose
        #self.counter = self.counter - .001
        pose = Pose()
        pose.position.x = float(-0.6) - self.x / 1000
        pose.position.y = 0.0
        pose.position.z = float(0.6) - self.y / 1000

        #self.get_logger().info(f'Processing line: x: {self.x}, y: {self.y}')


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
    rclpy.init(args=args)
    node = EEPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
