## Installations Required

### Prerequisites
Ensure you have Python 3 and `pip` installed on your system.
You need to be running a linux machine.

### Required Libraries
Setting up a python virtual environment is recommended.
Install the following Python libraries using `pip`:
```bash
pip install torch ultralytics opencv-python numpy zmq
```

## Running the object detection publisher
Before running, make sure that the camera is publishing to port 5555 via zmq.
Follow these steps to launch the object detection publisher:
1. Run the following command inside the `WRoverSoftware/` directory:
   ```bash
   colcon build
   ```
2. Open a new terminal and source the setup file:
   ```bash
   source install/setup.bash
   ```
3. Launch the object detection publisher:
   ```bash
   ros2 run object_detection_package yolo_detection
   ```
