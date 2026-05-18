import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int16MultiArray
import csv

class DataCollectionSubscriber(Node):
    def __init__(self):
        super().__init__('collect_data')
        
        self.num_columns = 9

        self.sub_fluoro = self.create_subscription(
            Int16MultiArray,
            '/sci_fluoro_raw',
            self.collect_fluoro,
            10
        )

        self.fluoro_file = open("fluoro.csv", "a", newline="")
        self.fluoro_writer = csv.writer(self.fluoro_file)

    def collect_fluoro(self, msg):
        data = list(msg.data)
        for i in range(0, len(data), self.num_columns):
            self.fluoro_writer.writerow(data[i:i + self.num_columns])

    def destroy_node(self):
        self.fluoro_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()