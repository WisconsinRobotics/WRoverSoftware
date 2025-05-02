#!/bin/bash

./canableStart.sh
source install/local_setup.bash
cd launch
ros2 launch arm_ik_launch.py
cd ..
