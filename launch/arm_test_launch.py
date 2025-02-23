from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_test_python',
            executable='arm_test_logic',
            name='arm_test_logic'
        ),
        Node(
            package='arm_test_python',
            executable='arm_test_neo',
            name='arm_test_neo'
        ),
        Node(
            package='arm_test',
            executable='arm_test_node',
            name='arm_test_node'
        )
    ])