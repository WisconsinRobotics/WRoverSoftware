import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_msgs_srvs.action import DrawPath
from custom_msgs_srvs.msg import Line
from geometry_msgs.msg import Point

import tkinter as tk
from tkinter import filedialog
from svgpathtools import svg2paths
import numpy as np


class DrawApp(Node):
    def __init__(self):
        super().__init__('draw_client')
        self._action_client = ActionClient(self, DrawPath, 'draw_path')

        # GUI
        self.root = tk.Tk()
        self.root.title("Load SVG and Send to ROS2")

        self.canvas = tk.Canvas(self.root, bg="white", width=120, height=100)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.lines = []

        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=10)

        self.load_btn = tk.Button(btn_frame, text="Load SVG", command=self.load_svg)
        self.load_btn.pack(side=tk.LEFT, padx=10)

        self.send_btn = tk.Button(btn_frame, text="Send Lines to ROS2", command=self.send_coordinates)
        self.send_btn.pack(side=tk.LEFT, padx=10)

        self.reset_btn = tk.Button(btn_frame, text="Reset Canvas", command=self.reset_canvas)
        self.reset_btn.pack(side=tk.LEFT, padx=10)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        self.root.geometry(f"{screen_width}x{screen_height}+0+0")

        self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))


    def load_svg(self):
        """Open an SVG file and extract path data, scaled and centered on canvas."""
        file_path = filedialog.askopenfilename(filetypes=[("SVG files", "*.svg")])
        if not file_path:
            return

        self.get_logger().info(f"Loading SVG: {file_path}")
        self.lines.clear()
        self.canvas.delete("all")

        paths, attributes = svg2paths(file_path)

        # Get all points first to calculate bounding box
        all_points = []
        raw_lines = []

        for path in paths:
            for segment in path:
                sampled_points = []

                num_samples = max(int(segment.length() / 5), 5)
                for i in range(num_samples):
                    pt = segment.point(i / (num_samples - 1))
                    x, y = pt.real, pt.imag
                    sampled_points.append((x, y))

                if len(sampled_points) > 1:
                    raw_lines.append(sampled_points)
                    all_points.extend(sampled_points)

        if not all_points:
            self.get_logger().warn("No points found in SVG.")
            return

        # Compute bounding box
        min_x = min(p[0] for p in all_points)
        max_x = max(p[0] for p in all_points)
        min_y = min(p[1] for p in all_points)
        max_y = max(p[1] for p in all_points)

        bbox_width = max_x - min_x
        bbox_height = max_y - min_y

        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        # Fallback if canvas hasn't fully rendered yet
        if canvas_width <= 1 or canvas_height <= 1:
            canvas_width = 1200
            canvas_height = 1000

        # Determine scale to fit canvas while maintaining aspect ratio
        scale = min(canvas_width / bbox_width, canvas_height / bbox_height) * 0.9  # add some margin

        # Compute offset to center
        offset_x = (canvas_width - bbox_width * scale) / 2
        offset_y = (canvas_height - bbox_height * scale) / 2

        # Transform and store lines
        for line in raw_lines:
            transformed = [
                (
                    (x - min_x) * scale + offset_x,
                    (y - min_y) * scale + offset_y
                ) for x, y in line
            ]
            self.lines.append(transformed)

            # Draw it
            for i in range(len(transformed) - 1):
                x0, y0 = transformed[i]
                x1, y1 = transformed[i + 1]
                self.canvas.create_line(x0, y0, x1, y1, fill="black")

        self.get_logger().info(f"Loaded and scaled {len(self.lines)} lines.")


    def send_coordinates(self):
        """Send lines to ROS2 action server."""
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = DrawPath.Goal()
        goal_msg.lines = []

        for line in self.lines:
            line_msg = Line()
            line_msg.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in line]
            goal_msg.lines.append(line_msg)

        self.get_logger().info(f'Sending {len(goal_msg.lines)} lines')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal was rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Lines completed with result: {result.result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_point}')

    def reset_canvas(self):
        self.canvas.delete("all")
        self.lines.clear()
        self.get_logger().info("Canvas reset!")

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    app = DrawApp()
    app.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
