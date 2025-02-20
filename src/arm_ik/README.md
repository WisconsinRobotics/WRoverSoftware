# Arm Repo

This package computes inverse kinematics for the mars rover arm. That means if you have cartesian (x, y, z) position or velocities, this package lets you map them to joint positions or velocites.


For using this package, a modified version of [relaxed_ik_ros2](https://github.com/uwgraphics/relaxed_ik_ros2) has been added to this workspace. This modification was adding urc_arm.urdf to [relaxed_ik_ros2/relaxed_ik_core/configs/urdfs](../relaxed_ik_ros2/relaxed_ik_core/configs/urdfs/urc_arm.urdf). If the arm CAD is ever updated, it is EXTREMELY important that this urdf along with the corresponding meshes in this project are updated.

Please see the main readme in this repo for runnign this package.


## Sources
* Visualizing URDF in RVIZ:
    * https://aleksandarhaber.com/how-to-create-urdf-and-launch-files-in-ros2-and-display-them-in-rviz/
    * https://www.learnros2.com/ros/ros2-building-blocks/configurations/rviz-configuration
* Relaxed IK: https://github.com/uwgraphics/relaxed_ik_ros2

