from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_test',
            executable='ik_test_node',
            name='ik_test_node'
        ),
        Node(
            package='arm_test_python',
            executable='science_neo',
            name='science_neo'
        ),
        Node(
            package='arm_test_python',
            executable='ik_subscriber',
            name='ik_subscriber'
        ),
        Node(
            package='wr_can_comms',
            executable='can_comms',
            name='can_comms'
        )
    ])

