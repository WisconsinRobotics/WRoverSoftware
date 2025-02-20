# Arm Repo

This is our repo for testing the arm inverse kinematics and control.


## Setup
If you want to run this repo on a docker container (recommended), please follow the instructions in [DOCKER_README.md](DOCKER_README.md). If you do not want to run this on docker, please first follow the instruction in  [NON_DOCKER_README.md](/NON_DOCKER_README.md).

Now build and source ros:
```bash
cd src/relaxed_ik_ros2/relaxed_ik_core
cargo build
cd ../../..

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
. install/setup.bash
```
## Running

* To run the IK with keyboard controls, run the following:
    ```bash
    ros2 launch arm_ik demo.launch.py 

    # Run this in another terminal:
    ros2 run relaxed_ik_ros2 keyboard_input.py 
    ```

    The following commands in the second terminal will move the bot:
    ```bash
    c - kill the controller controller script
    w - move end effector along +X
    x - move end effector along -X
    a - move end effector along +Y
    d - move end effector along -Y
    q - move end effector along +Z
    z - move end effector along -Z
    1 - rotate end effector around +X
    2 - rotate end effector around -X
    3 - rotate end effector around +Y
    4 - rotate end effector around -Y
    5 - rotate end effector around +Z
    6 - rotate end effector around -Z
    ```

## Notes
* Anytime you change anything in this repo, you will need to rebuild and source the code:
    ```bash
    colcon build
    source install/setup.bash
   
    ```
