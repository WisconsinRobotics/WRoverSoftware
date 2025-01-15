### INSTRUCTIONS
# NOTE: ON WINDOWS, ROS2 commands work fine, but compiling with colcon build has many issues. 
#
# 1: Install docker
# 2: Ensure you're in the WRoverSoftware dir (with Dockerfile)
# 2.5: ON WINDOWS, make sure docker engine is running (launch Docker Desktop and leave open)
# 3: Build the image:
#     $ docker build . -t wr_ros2
# 4: Run the image:
#     NOTE: ON WINDOWS, replace $(pwd) with full path
#     $ docker run -it -v $(pwd):/workspace --net=host wr_ros2
#
#
# If you need to open another teminal in the same docker container:
# 1. Run the steps above to enter the container
# 2. Open another terminal
# 3. Find the CONTAINER ID of the container:
#     $ docker ps
# 3. Enter the container:
#     $ docker exec -it {CONTAINER ID} bash
# 4. Source ROS2:
#     $ source install/setup.bash


# Use an official Ubuntu 20.04 LTS as a parent image
FROM osrf/ros:humble-desktop-full

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# INSTALL PACKAGES
RUN apt-get update && \
    apt-get install -y \
    curl\
    python3-pip\
    wget\
    python3-pykdl\
    vim

# Install python packages
RUN pip install --upgrade pip
RUN pip install --break-system-packages urdf-parser-py\
    python-can\
    pygame

# Set ROS2 dir to /workspace
WORKDIR /workspace/

# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]

