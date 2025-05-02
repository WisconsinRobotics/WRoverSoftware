from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Running falcons with forward Kinematics
        Node(
            package='arm_test',
            executable='arm_test_node',
            name='arm_test'
        ),

        # #Running NEO's to send to can_comms
        Node(
            package='arm_test_python',
            executable='arm_test_neo',
            name='arm_test_neo'
        ),

        # #Logic for rail
        Node(
            package='arm_test_python',
            executable='rail_subscriber',
            name='rail_subscriber'
        ),

         # #Logic for wrist
        # Node(
        #     package='arm_test_python',
        #     executable='arm_wrist_logic',
        #     name='arm_wrist_logic'
        # ),

        Node(
            package='wr_can_comms',
            executable='can_comms',
            name='can_comms'
        )
    ])
