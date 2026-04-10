#!/bin/bash

./canableStart.sh
source install/local_setup.bash
ros2 launch wr_can_comms can_launch.py
