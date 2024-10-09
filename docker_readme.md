# URC

## Setup
We have only tested these instructions on Ubuntu and Windows.

### Install Docker
Make sure you have installed docker. You can install docker [here for windows](https://docs.docker.com/desktop/install/windows-install/) or [here for linux](https://docs.docker.com/desktop/install/linux/)

### First you need to setup display forwarding

**If you are on windows...**
Install https://sourceforge.net/projects/vcxsrv/. Start XLaunch (from the VcXsrv program group), set display settings to multiple windows, and ensure "Disable access control" is checked.

**If you are linux...**
Set up display forwarding by running:
```bash
xhost +local:
```
### Build and Start The Container
Now  build the container image and start the container. Make sure you are in this directories root directory. These commands use the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.

**If you are on linux...**
```bash
sudo docker build -t urc-container .
sudo docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host urc-container
```

**If you are on Windows...**
```bash
docker build -t urc-container .
docker run -it -e DISPLAY=host.docker.internal:0.0 -v ${PWD}:/workspace --net=host urc-container
```

### Setup Container
Now your container should be running and you should be in your container's terminal.
```bash
source /opt/ros/jazzy/setup.bash
cd workspace
```

Now you are ready to run everything in README.md.


## Notes
* To open another docker terminal for a running container, run the following on your home-machine:
```bash
# Show your running CONTAINER_ID
docker ps 

# Open another terminal using that CONTAINER_ID
docker exec -it  <YOUR_CONTAINER_ID> bash

# Source ROS properly
source /opt/ros/jazzy/setup.sh
cd workspace
source install/setup.bash
```
