# Arm Repo

This is our repo for testing the arm inverse kinematics and control.


## Setup
If you want to run this repo on a docker container (recommended), please follow the instructions in [DOCKER_README.md](DOCKER_README.md). If you do not want to run this on docker, please first follow the instruction in  [NON_DOCKER_README.md](/NON_DOCKER_README.md).

Now build and source ros:
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```
## Running
To run the RVIZ visualization, do the following
```bash
ros2 launch urdf_visualization display.launch.py
```

## Notes
* Anytime you change anything in this repo, you will need to rebuild and source the code:
    ```bash
    colcon build
    source install/setup.bash
   
    ```
## Sources
* Visualizing URDF in RVIZ:
    * https://aleksandarhaber.com/how-to-create-urdf-and-launch-files-in-ros2-and-display-them-in-rviz/
    * https://www.learnros2.com/ros/ros2-building-blocks/configurations/rviz-configuration