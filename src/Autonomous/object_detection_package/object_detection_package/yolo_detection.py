# Import libraries
import os
import zmq
import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import String

# Import custom messages
from custom_msgs_srvs.msg import Detection

# Import package share directory path
from ament_index_python.packages import get_package_share_directory


class YOLODetectionPublisher(Node):
    # Initialization
    def __init__(self):
        # Create publisher
        super().__init__('yolo_detection_publisher')
        self.publisher_ = self.create_publisher(Detection, 'detection_results', 10)

        # Set timer to process frames every timer_period seconds
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize ZeroMQ Context and SUB socket for camera video stream
        context = zmq.Context()
        self.camera_socket = context.socket(zmq.SUB)
        
        # Subscribe to all messages and connect to publisher port
        self.camera_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.camera_socket.connect("tcp://localhost:5555")

        # Load YOLO model
        self.model = YOLO(os.path.join(get_package_share_directory("object_detection_package"), "model.pt"))


    # Async fuction for YOLO inference
    async def yolo_inference(self, frame):
        return self.model(frame, verbose=False)
    
    # Crop and resize frames to 640x640 to fit the model
    def crop_and_resize(self, frame):
        # Get current frame dimensions
        height, width, _ = frame.shape
        
        # Crop the frame into a square
        x, y = width // 2, height // 2
        size = min(width, height)

        x1 = x - (size // 2)
        y1 = y - (size // 2)
        x2 = x + (size // 2)
        y2 = y + (size // 2)
        
        frame = frame[y1:y2, x1:x2]
        
        # Resize the frame to 640x640
        frame = cv2.resize(frame, (640, 640))
        
        return frame


    # Async function for frame processing
    async def timer_callback(self):
        # Flush the buffer to ensure most recent frame
        while True:
            try:
                _ = self.camera_socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                self.get_logger().warn(f"Frame not captured. Retrying...")
                break
                
        # Decode frame data from camera socket
        data = np.frombuffer(self.camera_socket.recv(), dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            
        # Crop and resize the frame
        frame = self.crop_and_resize(frame)
        
        # Perform YOLO inference asynchronously
        results = await self.yolo_inference(frame)

        best_box = None
        best_conf = 0

        # Process results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get bounding box, class and confidence level
                x1, y1, x2, y2 = box.xyxy[0]
                conf = box.conf[0].item()

                # Store the object with the highest confidence level
                if (conf > best_conf):
                    best_box = (x1, y1, x2, y2)
                    best_conf = conf

        # If an object is detected
        if best_box is not None:
            # Display bounding boxes and detection info
            color = (0, 255, 0)
            x1, y1, x2, y2 = best_box
            
            # Janky algorithm for estimating distance
            interp = 0.004
            length = (x2 - x1) if (x2 - x1) > (y2 - y1) else (y2 - y1)
            dis = 1 / (length * interp)
            
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            label = f"{best_conf:.2f} {dis:.2f}"
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Publish message
            msg = Detection()
            msg.x = int((x1 + x2) / 2)
            msg.y = int((y1 + y2) / 2)
            msg.dis = float(dis)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing {msg.x}, {msg.y}, {msg.dis}")

        # Graphical display
        cv2.imshow("YOLO Object Detection", frame)
        cv2.waitKey(1)


    # Cleanup
    def __del__(self):
        # Release camera access
        if self.video.isOpened():
            self.video.release()

        # Terminate graphical display
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    yolo_detection_publisher = YOLODetectionPublisher()

    rclpy.spin(yolo_detection_publisher)

    yolo_detection_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

