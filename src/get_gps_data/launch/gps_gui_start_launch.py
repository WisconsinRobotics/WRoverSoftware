import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='get_gps_data',
            executable='display_gps',
            name='display_gps'),
        
        launch_ros.actions.Node(
            package='get_gps_data',
            executable='start_gui',
            name='start_gui'),
  ])
