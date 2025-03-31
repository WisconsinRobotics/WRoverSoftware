import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_msgs_srvs.action import DrawPath
from custom_msgs_srvs.msg import Line
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

        self.lines = []  # Store multiple lines
        self.current_line = []
        self.drawing = False

        # Event bindings
        self.canvas.bind("<Button-1>", self.start_drawing)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drawing)

        # Buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=10)

        self.send_btn = tk.Button(btn_frame, text="Send Lines to ROS2", command=self.send_coordinates)
        self.send_btn.pack(side=tk.LEFT, padx=10)

        self.reset_btn = tk.Button(btn_frame, text="Reset Canvas", command=self.reset_canvas)
        self.reset_btn.pack(side=tk.LEFT, padx=10)

    def start_drawing(self, event):
        """Start a new line."""
        self.drawing = True
        self.current_line = [(event.x, event.y)]

    def draw(self, event):
        """Draw the line and store coordinates."""
        if self.drawing:
            x, y = event.x, event.y
            self.canvas.create_line(self.current_line[-1][0], self.current_line[-1][1], x, y, fill="black")
            self.current_line.append((x, y))

    def stop_drawing(self, event):
        """Stop drawing and store the line."""
        if self.current_line:
            self.lines.append(self.current_line)
        self.drawing = False
        self.current_line = []

    def send_coordinates(self):
        """Send the drawn lines to the ROS2 action server."""
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = DrawPath.Goal()

        # Convert each line to a Line message
        for line in self.lines:
            line_msg = Line()
            line_msg.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in line]
            goal_msg.lines.append(line_msg)

        # Send goal
        self.get_logger().info(f'Sending {len(self.lines)} lines...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_point))

    def reset_canvas(self):
        """Clear the canvas and reset all lines."""
        self.canvas.delete("all")
        self.lines.clear()
        self.get_logger().info("Canvas reset!")

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
