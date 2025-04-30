src directory that holds all project packages


run the following commands to download dependencies:
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
sudo apt install ros-humble-depthai-ros
connect the camera via usb
ros2 launch depthai_ros_driver camera.launch.py   this turns on the camera fully
roslaunch depthai_ros_driver rgbd_pcl.launch.py      this turns on the camera without neural networks which are unnecessary (recommended for memory)
should be ready to run
