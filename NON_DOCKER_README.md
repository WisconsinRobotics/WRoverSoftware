# Non-Docker Setup
If you are not using docker, you need to follow these instructions.

### 1. Setup Environment
Make sure you are running Ubuntu 24.04. You may need to do this in a VM.

### 2. Install ROS 2 and Rust
Install ROS 2 Jazzy by following [this tutorial](https://docs.ros.org/en/jazzy/Installation.html).

Install Rust by following [these directions](https://doc.rust-lang.org/cargo/getting-started/installation.html).

### 3. Install you apt dependencies
```bash
sudo apt-get update 
sudo apt-get install -y \
    curl\
    python3-pip\
    wget\
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro
```

You're a supestar, now you are ready to continue to [README.md](/README.md).