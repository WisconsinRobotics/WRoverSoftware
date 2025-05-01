# Wisconsin Robotics -- 2024-2025 Software System
Codebase for the URC rover at UW-Madison, 2024-2025.

## Object Detection

currently uses yolo to detect

package: ```object_detection_package```

node: ```yolo_detection```

main code in located in ```src/Autonomous/object_detection_package/object_detection_package/yolo_detection.py```

This subscribes to the topic ```/camera_data_topic``` to get camera image data and then publishes to the topic ```/detection_results```

The data published is:
```
int64 x
int64 y
float dis
```

