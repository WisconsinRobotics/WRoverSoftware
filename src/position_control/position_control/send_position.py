import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from relaxed_ik_ros2.msg import  EEPoseGoals
from geometry_msgs.msg import Pose, Twist, PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from draw_xy_action.action import DrawPath

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

        # Publishers
        self.pose_publisher = self.create_publisher(EEPoseGoals, '/relaxed_ik/ee_pose_goals', 1)

        # Timer to publish periodically
        self.timer = self.create_timer(0.1, self.publish_messages)

        self.counter = 0.0

        # Create action server
        self._action_server = ActionServer(
            self,
            DrawPath,
            'draw_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
    def goal_callback(self, goal_request):
        """Accepts all goals."""
        self.get_logger().info('Received new goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow goal cancellation."""
        self.get_logger().info('Goal canceled.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the action goal."""
        self.get_logger().info('Executing goal...')

        for idx, point in enumerate(goal_handle.request.points):
            self.get_logger().info(f'Point {idx + 1}: x={point.x}, y={point.y}')
            
            # Send feedback
            feedback_msg = DrawPath.Feedback()
            feedback_msg.current_point = point
            goal_handle.publish_feedback(feedback_msg)

        # Complete the action
        goal_handle.succeed()
        result = DrawPath.Result()
        result.result = f'Processed {len(goal_handle.request.points)} points.'
        return result
    
    def publish_messages(self):
        # Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"

        msg = EEPoseGoals()
        self.counter = self.counter + 0.01
        # Create Pose
        pose = Pose()
        pose.position.x = float(-self.counter)
        pose.position.y = 0.0
        pose.position.z = float(self.counter)

        # Keep the orientation fixed (90-degree rotation around X-axis)
        pose.orientation.x = 0.0
        pose.orientation.y = math.sqrt(0.5)   # Equivalent to math.pow(2, (1/2)/2)
        pose.orientation.z = 0.0
        pose.orientation.w = math.sqrt(0.5)   # Equivalent to math.pow(2, (1/2)/2)


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

        

        self.get_logger().info("Z: " + str(self.counter))

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
