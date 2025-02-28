#!/bin/bash

canableStart.sh

colcon build
source install/local_setup.bash

cd launch
ros2 launch swerve_launch_rover.py
cd ..
