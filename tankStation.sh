#!/bin/bash

source install/local_setup.bash

cd launch
ros2 launch tank_launch.py
cd ..

#ros2 launch arm_ik demo.launch.py
