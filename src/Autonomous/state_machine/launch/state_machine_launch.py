import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

config_file = os.path.join(
    get_package_share_directory('wr_LED_matrix'),
    'config',
    'params.yaml'
)

def generate_launch_description():
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
    ])