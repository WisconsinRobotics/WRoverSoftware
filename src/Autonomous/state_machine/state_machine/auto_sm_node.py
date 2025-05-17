import rclpy
from rclpy.node import Node
from state_machine.Autonomous_State_Machine import AutonomousStateMachine

class AutonomousNode(Node):
    def __init__(self):
        super().__init__('autonomous_node')

        # Embed the state machine and give it access to the Node via model=self
        self.sm = AutonomousStateMachine(model=self)

        # You can now trigger transitions based on timers, callbacks, or results
        # self.timer = self.create_timer(2.0, self.run_state_machine)
        
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        node.get_logger().info("Shutting down.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    