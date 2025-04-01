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
Before running, __make sure that the camera is publishing to port 5555 via zmq__.  
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
   
## Subscribing to the object detection publisher
The object detection publisher uses the custom 'Detection' message type, and publishes to the topic 'detection_results' when a bottle/hammer is detected.  

Detection.msg is as follows:
```
int64 x
int64 y
float64 dis
```  

``x`` and ``y`` are the coordinates of the center of the bounding box of the detected object.  
Both ``x`` and ``y`` range from 0 to 640, where the top-left corner has coordinates (0, 0).  
``dis`` is the estimated distance between the detected object and the camera, in meters.
