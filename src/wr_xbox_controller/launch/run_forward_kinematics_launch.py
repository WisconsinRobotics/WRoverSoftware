from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='wr_xbox_controller',
            namespace='',
            executable='arm_rail_xbox',
            name='arm_rail_xbox',
        ),
        Node(
            package='wr_xbox_controller',
            namespace='',
            executable='xbox_controller',
            name='xbox_controller',
        ),
    ])