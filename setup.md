Modified for this tutorial: https://aleksandarhaber.com/how-to-create-urdf-and-launch-files-in-ros2-and-display-them-in-rviz/

Install the following:
```bash
sudo apt-get update

sudo apt-get install gedit

sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro

sudo apt install ros-jazzy-xacro
```

mkdir src
cd src
ros2 pkg create --build-type ament_cmake urdf_visualization
cd urdf_visualization/
mkdir launch urdf

cd ../..
colcon build

Add all the doc changes... (replaced urdf_test with urdf_visualization)


colcon build
source install/setup.bash
ros2 launch urdf_visualization display.launch.py