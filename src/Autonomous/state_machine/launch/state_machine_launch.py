import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import ament_index_python.packages

config_file = os.path.join(
    get_package_share_directory('wr_LED_matrix'),
    'config',
    'params.yaml'
)
# config_directory = os.path.join(
#     ament_index_python.packages.get_package_share_directory('ublox_'),
#     'config')
# params = os.path.join(config_directory, 'zed_f9p.yaml')
# ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
#                                             executable='ublox_gps_node',
#                                             output='both',
#                                             parameters=[params])

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('ublox_gps'), 'launch'])
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='state_machine',
            executable='state_machine',
            name='state_machine',
            parameters=[config_file]),

        launch_ros.actions.Node(
            package='wr_LED_matrix',
            executable='led_service',
            name='led_service',
            parameters=[config_file]),

        launch_ros.actions.Node(
            package='wr_drive_ai',
            executable='navigate_auto',
            name='navigate_auto'),

        launch_ros.actions.Node(
            package='wr_imu_compass',
            executable='compass',
            name='compass'),

        # Launching nodes to motor
        launch_ros.actions.Node(
            package='wr_swerve_control',
            executable='swerve_control',
            name='swerve_control'
        ),
        launch_ros.actions.Node(
            package='wr_swerve_motor',
            executable='swerve_motor',
            name='swerve_motor'
        ),
        launch_ros.actions.Node(
            package='wr_can_comms',
            executable='can_comms',
            name='can_comms'
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'ublox_gps_node_zedf9p-launch.py'])
        ),
        # #ublox_gps_node,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=ublox_gps_node,
        #         on_exit=[launch.actions.EmitEvent(
        #             event=launch.events.Shutdown())],
        #     )),

        #camera and detection nodes
        launch_ros.actions.Node(
            package='wr_autonomous_camera',
            executable='camera_data',
            name='camera_data'
        ),
        launch_ros.actions.Node(
            package='auto_implementation',
            executable='aruco_detection',
            name='aruco_detection'
        ),
        
        launch_ros.actions.Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection'),

    ])


