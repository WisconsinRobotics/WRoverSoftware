#!/bin/bash

source install/local_setup.bash

ros2 run wr_swerve_control swerve_control &&
	ros2 run wr_swerve_motor swerve_motor

