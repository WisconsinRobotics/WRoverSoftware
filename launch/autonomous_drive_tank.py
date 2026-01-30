from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wr_swerve_control',
            executable='tank_control',
            name='tank_control'
        ),
        Node(
            package='wr_can_comms',
            executable='can_comms',
            name='can_comms'
        )
    ])
