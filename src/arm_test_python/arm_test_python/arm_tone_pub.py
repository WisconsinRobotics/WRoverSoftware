import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class TonePublisher(Node):
    def __init__(self):
        super().__init__('tone_publisher')
        
        # Publisher for tone frequencies
        self.publisher_ = self.create_publisher(Float32, 'tone_freq', 10)
        
        # Timer to publish tones periodically (e.g., every 0.5 sec)
        self.timer = self.create_timer(0.1 , self.publish_tone)
        
        # Example: simple sine wave tone sequence (frequencies in Hz)
        self.frequencies = [261.0,261.0,261.0,261.0,261.0,261.0,
    392.0,392.0,392.0,392.0,392.0,392.0,
    440.0,440.0,440.0,440.0,440.0,440.0,
    392.0,392.0,392.0,392.0,392.0,392.0,392.0,392.0,392.0,392.0,392.0,392.0,
    349.0,349.0,349.0,349.0,349.0,349.0,
    329.0,329.0,329.0,329.0,329.0,329.0,
    293.0,293.0,293.0,293.0,293.0,293.0,
    261.0,261.0,261.0,261.0,261.0,261.0,261.0,261.0,261.0,261.0,261.0,261.0]   # C x12  # C4 to B4
        self.index = 0

    def publish_tone(self):
        # Select current frequency
        freq = self.frequencies[self.index]
        msg = Float32()
        msg.data = freq
        
        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published frequency: {freq} Hz')
        
        # Move to next frequency
        self.index = (self.index + 1) % len(self.frequencies)

def main(args=None):
    rclpy.init(args=args)
    node = TonePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

