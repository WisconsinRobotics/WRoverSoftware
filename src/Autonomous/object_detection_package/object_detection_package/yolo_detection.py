# Import libraries
import os
import cv2
import time
import rclpy
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

        # Initialize video capture to use webcam
        self.video = cv2.VideoCapture(0)

        # Set camera height and weidth
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)

        # Ensure the buffer only stores the most recent frame
        self.video.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # If webcam cannot be opened
        if not self.video.isOpened():
            self.get_logger().error("Failed to open webcam")

        # Load YOLO model
        self.model = YOLO(os.path.join(get_package_share_directory("object_detection_package"), "model.pt"))


    # Async fuction for YOLO inference
    async def yolo_inference(self, frame):
        return self.model(frame, verbose=False)


    # Async function for frame processing
    async def timer_callback(self):
        # Capture a frame from the webcam
        ret, frame = self.video.read()

        # If image capture failed
        if not ret:
            self.get_logger().error("Failed to capture image from webcam")
            return

        # Perform YOLO inference asynchronously
        results = await self.yolo_inference(frame)

        best_box = None
        best_cls = None
        best_conf = 0

        # Process results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get bounding box, class and confidence level
                x1, y1, x2, y2 = box.xyxy[0]
                cls = box.cls[0].item()
                conf = box.conf[0].item()

                # Store the object with the highest confidence level
                if (conf > best_conf):
                    best_box = (x1, y1, x2, y2)
                    best_cls = int(cls)
                    best_conf = conf

        # If an object is detected
        if best_box is not None:
            # Display bounding boxes and detection info
            color = (0, 255, 0)
            x1, y1, x2, y2 = best_box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            best_cls = "bottle" if best_cls == 0 else "hammer"
            label = f"{best_cls} {best_conf:.2f}"
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Publish message
            msg = Detection()
            msg.x = int(x1 + x2 / 2)
            msg.y = int(y1 + y2 / 2)
            msg.cls = 0 if best_cls == "bottle" else 1
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing {msg.x}, {msg.y}, {msg.cls}")

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
