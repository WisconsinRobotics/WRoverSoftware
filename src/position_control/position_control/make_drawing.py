import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from position_control.action import DrawPath
from geometry_msgs.msg import Point

import tkinter as tk


class DrawApp(Node):
    def __init__(self):
        super().__init__('draw_client')
        
        # ROS2 action client
        self._action_client = ActionClient(self, DrawPath, 'draw_path')

        # GUI setup
        self.root = tk.Tk()
        self.root.title("Draw and Send to ROS2")

        self.canvas = tk.Canvas(self.root, bg="white", width=600, height=600)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.coordinates = []
        self.drawing = False

        # Event bindings
        self.canvas.bind("<Button-1>", self.start_drawing)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drawing)

        # Button to send coordinates
        self.btn = tk.Button(self.root, text="Send X-Y Coordinates to ROS2", command=self.send_coordinates)
        self.btn.pack(pady=10)

    def start_drawing(self, event):
        """Start drawing on the canvas."""
        self.drawing = True
        self.coordinates = [(event.x, event.y)]

    def draw(self, event):
        """Draw the line and store coordinates."""
        if self.drawing:
            x, y = event.x, event.y
            self.canvas.create_line(self.coordinates[-1][0], self.coordinates[-1][1], x, y, fill="black")
            self.coordinates.append((x, y))

    def stop_drawing(self, event):
        """Stop drawing when the mouse is released."""
        self.drawing = False

    def send_coordinates(self):
        """Send the drawn X-Y coordinates to the ROS2 action server."""
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = DrawPath.Goal()
        
        # Convert coordinates to Point messages
        goal_msg.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in self.coordinates]

        # Send goal
        self.get_logger().info(f'Sending {len(goal_msg.points)} points...')
        self._action_client.send_goal_async(goal_msg)

    def run(self):
        """Run the Tkinter GUI."""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    app = DrawApp()
    app.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
