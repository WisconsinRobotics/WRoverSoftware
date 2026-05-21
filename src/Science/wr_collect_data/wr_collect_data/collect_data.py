import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String
import csv

class DataCollectionSubscriber(Node):
    def __init__(self):
        super().__init__('collect_data')

        self.sub_fluoro = self.create_subscription(
            Int16MultiArray, '/sci_fluoro_raw', self.collect_fluoro, 10)
        self.sub_soil = self.create_subscription(
            Int16MultiArray, '/sci_soil_raw', self.collect_soil, 10)
        self.sub_arduino = self.create_subscription(
            String, '/sci_arduino_messages', self.collect_arduino, 10)

        self.fluoro_file = open("fluoro.csv", "a", newline="")
        self.fluoro_writer = csv.writer(self.fluoro_file)
        if self.fluoro_file.tell() == 0:
            self.fluoro_writer.writerow(["515nm", "590nm"])

        self.soil_file = open("soil.csv", "a", newline="")
        self.soil_writer = csv.writer(self.soil_file)
        if self.soil_file.tell() == 0:
            self.soil_writer.writerow(["temperature", "moisture"])

    def collect_fluoro(self, msg):
        self.fluoro_writer.writerow(list(msg.data))

    def collect_soil(self, msg):
        self.soil_writer.writerow(list(msg.data))

    def collect_arduino(self, msg):
        self.get_logger().info(f"[Arduino]: {msg.data}")

    def destroy_node(self):
        self.fluoro_file.close()
        self.soil_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()