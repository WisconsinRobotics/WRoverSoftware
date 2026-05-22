#!/bin/bash

./canableStart.sh
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch wr_can_comms can_launch.py
