# URC

## Setup
Make sure you have installed docker.

If you are on windows, don't do the following and instead install https://sourceforge.net/projects/vcxsrv/. Start XLaunch (from the VcXsrv program group), choose the display settings (e.g., multiple windows), and ensure "Disable access control" is checked.

First set up display forwarding:
```bash
xhost +local:
```

Now  build the container image and start the container. Make sure you are in this directories root directory. These commands use the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. TJese commands also allow the containers display to be forwarded to your host machine so that you can see it.
```bash
sudo docker build -t urc-container . # Don't use sudo if on windows

# If you are on linux run
sudo docker run -it --e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host urc-container

# If you are on windows, run the following instead
sudo docker run -it --e DISPLAY=host.docker.internal:0.0 -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host urc-container

```

```bash
source /opt/ros/jazzy/setup.bash
cd workspace
```

Now you can run everything in README.md


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