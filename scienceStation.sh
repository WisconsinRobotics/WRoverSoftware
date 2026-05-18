#!/bin/bash

source install/local_setup.bash

cd launch
ros2 launch science_launch_base_launch.py
cd ..