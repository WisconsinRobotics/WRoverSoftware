import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='state_machine',
            executable='state_machine',
            name='state_machine'),

        launch_ros.actions.Node(
            package='wr_LED_matrix',
            executable='led_service',
            name='led_service'),

        launch_ros.actions.Node(
            package='wr_drive_ai',
            executable='navigate_auto',
            name='navigate_auto'),
    ])