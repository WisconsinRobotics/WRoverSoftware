from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wr_science_control',
            executable='science_control',
            name='science_control'
        ),

        Node(
            package='wr_xbox_controller',
            executable='science_controller',
            name='science_controller'
        ),
         #Node(
         #    package='wr_xbox_controller',
         #    executable='arm_xbox_ik',
         #    name='arm_xbox_ik'
         #),
         #Node(
         #    package='wr_xbox_controller',
         #    executable='rail_gripper_controller',
         #    name='rail_gripper_controller'
         #)#,
       # Node(
       #     package='wr_depth_camera',
       #     executable='display',
       #     name='display'
       # )
    ])
