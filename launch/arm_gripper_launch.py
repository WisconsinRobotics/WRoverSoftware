from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='arm_test',
        #    executable='arm_test_node',
        #    name='arm_test'
        #),
        
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
        #Node(
        #    package='arm_test_python',
        #    executable='rail_subscriber',
        #    name='rail_subscriber'
        #),
        Node(
            package='wr_can_comms',
            executable='can_comms',
            name='can_comms'
        )
        #Control with keyboard
        # Node(
        #    package='wr_controller',
       #     executable='keyboard_controller',
        #    name='keyboard_controller'
        #)

    ])
