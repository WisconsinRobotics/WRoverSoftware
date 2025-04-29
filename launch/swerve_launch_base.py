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
            package='wr_xbox_controller',
            executable='drive_controller',
            name='drive_controller'
        ),
        Node(
            package='wr_xbox_controller',
            executable='arm_xbox_ik',
            name='arm_xbox_ik'
        ),
        Node(
            package='wr_xbox_controller',
            executable='rail_gripper_controller',
            name='rail_gripper_controller'
        )
    ])
