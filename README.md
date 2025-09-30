Must have OpenCV verison - 4.6.0 and NumPY version 1.26.4

Make sure to replace "<distro>" with whichever version of ros2 is in use (ex: humble)
Install DepthAI library using the command: sudo apt install ros-[distro]-depthai-ros 

First connect to OAK-D W with the following command: ros2 launch depthai_ros_driver camera.launch.py
then build and run the package.

checklist for progress:

Phase 1: basic prototype(vfh algorithm)
- pixel location to angle ✔️
- find min value of sectors ✔️
- Gap detection ✔️
- rudementary ground removal without plane detection (no ransac) ✔️
- choosing paths and moving to gnss location

Phase 2: 
- better ground detection using ransac
- cost function / danger function for concave rock problem
- terrain classifications

and lots and lots of testing :>
