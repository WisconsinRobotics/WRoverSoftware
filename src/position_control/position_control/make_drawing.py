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
        self.i = 0
        self.drawing = False
        self.delay = .01

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
        """Send the drawn lines to the ROS2 action server, waiting for each response before sending the next."""
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server not available!')
            return
        
        self.line_index = 0  # Track which line is being sent
        self.send_next_line()  # Start sending the first line

    def send_next_line(self):
        """Send the next line if there are more to send."""
        if self.line_index >= len(self.lines):
            self.get_logger().info('All lines have been sent.')
            return  # Stop when all lines are sent

        goal_msg = DrawPath.Goal()
        line_msg = Line()
        
        # Convert the current line to a Line message
        line_msg.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in self.lines[self.line_index]]
        goal_msg.line = line_msg  # Ensure it's a list

        self.get_logger().info(f'Sending line {self.line_index + 1} of {len(self.lines)}...')

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.drawing = True  # Mark as drawing

    def goal_response_callback(self, future):
        """Callback executed when the action server responds to the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal {self.line_index + 1} was rejected.')
            self.drawing = False
            return
        
        self.get_logger().info(f'Goal {self.line_index + 1} accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback executed when the action server provides a result."""
        result = future.result().result
        self.get_logger().info(f'Line {self.line_index + 1} completed with result: {result.result}')

        self.drawing = False  # Allow sending the next line
        self.line_index += 1  # Move to the next line
        self.send_next_line()  # Send the next line


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
