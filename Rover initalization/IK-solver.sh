cd /home/wiscrobo/workspace/IK_Solver/WRoverSoftware
cd src/relaxed_ik_ros2/relaxed_ik_core
cargo build
cd ../../..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
. install/setup.bash
ros2 launch arm_ik demo.launch.py