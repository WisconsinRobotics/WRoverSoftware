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
            executable='arm_test_neo',
            name='arm_test_neo'
        ),

        #RUN GRIPPER WITH IK
        Node(
            package='arm_test_python',
            executable='ik_subscriber',
            name='ik_subscriber'
        ),

        #RUN GRIPPER W/OUT IK
        #Node(
        #    package='arm_test_python',
        #    executable='arm_test_logic',
        #    name='arm_test_logic'
        #),


        #Logic for rail
        Node(
            package='arm_test_python',
            executable='rail_subscriber',
            name='rail_subscriber'
        ),


        Node(
           package='wr_can_comms',
           executable='can_comms',
           name='can_comms'
        )
    ])
