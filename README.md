Must have OpenCV verison - 4.6.0 and NumPY version 1.26.4

Make sure to replace "<distro>" with whichever version of ros2 is in use (ex: humble)
Install DepthAI library using the command: sudo apt install ros-<distro>-depthai-ros 

First connect to OAK-D W with the following command: ros2 launch depthai_ros_driver camera.launch.py
then build and run the package.

Subscribe to the "object_avoidance/info" topic for an integer[128] array with three values:
0 - close
1 - mid
2 - far
