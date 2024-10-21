#!/bin/bash
cd /workspace/src/relaxed_ik_ros2
git config --global --add safe.directory /workspace
git submodule init
git submodule update
cd relaxed_ik_core
cargo build
cd /workspace/src
colcon build --symlink-install
