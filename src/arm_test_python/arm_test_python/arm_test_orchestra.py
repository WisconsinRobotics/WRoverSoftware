import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from phoenix6 import hardware, orchestra

class OrchestraSubscriber(Node):

    def __init__(self):
        super().__init__('arm_music')
        self.music_path = 'onwisc.chrp'
        self.elbow_motor = hardware.talon_fx.TalonFX(1, 'can0')
        self.shoulder_motor = hardware.talon_fx.TalonFX(0, 'can0')
        self.orchestra = orchestra.Orchestra([self.elbow_motor,self.shoulder_motor])
        self.orchestra.load(self.music_path)

        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons',
            self.arm_listener_music,
            10)

    def arm_listener_music(self, msg):
        if msg.data[5] == 1:
            if not self.orchestra.is_playing():
                self.orchestra.play()
                self.get_logger().info(f'Started playing file at {self.music_path}')


def main(args=None):
    rclpy.init(args=args)

    orchestra_subscriber = OrchestraSubscriber()
    print("Starting ros2 arm music node")
    rclpy.spin(orchestra_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orchestra_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
