#!/bin/bash

cd /home/old-arm/workspace/WRoverSoftware
./canableStart.sh
source install/local_setup.bash
cd launch
ros2 launch arm_gripper_launch.py
