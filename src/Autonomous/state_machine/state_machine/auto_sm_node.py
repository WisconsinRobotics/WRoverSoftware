import rclpy
from rclpy.node import Node
from state_machine.Autonomous_State_Machine import AutonomousStateMachine
from rclpy.executors import MultiThreadedExecutor


class AutonomousNode(Node):
    def __init__(self):
        super().__init__('autonomous_node')

        # Embed the state machine and give it access to the Node via model=self
        self.sm = AutonomousStateMachine(model=self)

        # You can now trigger transitions based on timers, callbacks, or results
        # self.timer = self.create_timer(2.0, self.run_state_machine)
        
def main(args=None):
    rclpy.init()
    node = AutonomousNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
    