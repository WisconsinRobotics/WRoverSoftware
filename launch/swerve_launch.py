from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wr_swerve_control',
            executable='swerve_control',
            name='swerve_control'
        ),
        Node(
            package='wr_swerve_motor',
            executable='swerve_motor',
            name='swerve_motor'
        ),
          Node(
            package='wr_can_comms',
            executable='can_comms',
            name='can_comms'
        )
    ])
