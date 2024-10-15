# Use an official Ubuntu 20.04 LTS as a parent image
FROM osrf/ros:jazzy-desktop-full

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

###### INSTALL PACKAGES ######
RUN apt-get update && \
    apt-get install -y \
    curl\
    python3-pip\
    wget\
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro 

##############################

WORKDIR /workspace/

# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]

