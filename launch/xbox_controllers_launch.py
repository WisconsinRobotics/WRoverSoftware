from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wr_xbox_controller',
            executable='arm_xbox_control',
            name='arm_xbox'
        ),
        Node(
            package='wr_xbox_controller',
            executable='xbox_controller',
            name='xbox_controller'
        )
    ])