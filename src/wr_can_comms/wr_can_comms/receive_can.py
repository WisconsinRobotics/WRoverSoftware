# Key performance fixes:
# 1. DO NOT reopen can.Bus inside receive_canbus() every timer cycle
# 2. Use bus.recv(timeout=0.0) instead of iterating self.bus
# 3. Increase polling frequency
# 4. Drain queue each callback
# 5. Avoid unnecessary parsing/logging
# 6. Use hardware/kernel CAN filters
# 7. Remove duplicate current conversions

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import can

CAROUSEL_VESC = 80
IN_OUT_VESC = 82
FR_SWERVE_VESC = 73


class CANSubscriber(Node):

    def __init__(self):
        super().__init__('can_subscriber')
        # Publishers
        self.pid_publisher = self.create_publisher(Float32, 'pid', 10)
        self.current_side_publisher = self.create_publisher(Bool, 'current_side_to_side', 10)
        self.current_in_out_publisher = self.create_publisher(Bool, 'current_in_out', 10)

        self.current_side_msg = Bool()
        self.current_side_msg.data = True

        self.current_in_out_msg = Bool()
        self.current_in_out_msg.data = True

        # Faster timer for lower latency
        timer_freq = 0.002  # 2 ms instead of 10 ms
        self.timer = self.create_timer(timer_freq, self.timer_callback)

        # Open CAN bus ONCE
        self.bus = can.Bus(
            channel='can0',
            interface='socketcan',
            can_filters=[
                {"can_id": CAROUSEL_VESC, "can_mask": 0xFF, "extended": True},
                {"can_id": IN_OUT_VESC, "can_mask": 0xFF, "extended": True},
                {"can_id": FR_SWERVE_VESC, "can_mask": 0xFF, "extended": True},
            ]
        )

    def carousel_publish(self, pid_value: float):
        msg = Float32()
        msg.data = pid_value
        self.pid_publisher.publish(msg)

    def timer_callback(self):
        # Drain all available messages quickly
        while True:
            msg = self.bus.recv(timeout=0.0)  # Non-blocking
            if msg is None:
                break
            self.process_can_message(msg)

    def process_can_message(self, msg):
        arb_id = msg.arbitration_id
        command_id = (arb_id >> 8) & 0xFF
        vesc_id = arb_id & 0xFF
        b = msg.data
        
    

        # STATUS_4
        if command_id == 16:
            if vesc_id == CAROUSEL_VESC:
                pid_pos = int.from_bytes(b[6:8], 'big', signed=True) / 50
                self.carousel_publish(pid_pos)

        # STATUS
        elif command_id == 9:
            current = int.from_bytes(b[4:6], 'big', signed=True) / 10

            if vesc_id == CAROUSEL_VESC:
                self.current_side_msg.data = current <= 19
                self.current_side_publisher.publish(self.current_side_msg)

            elif vesc_id == IN_OUT_VESC:
                # Fast current access here
                self.current_in_out_msg.data = current <= 1.35
                self.current_in_out_publisher.publish(self.current_in_out_msg)
                # Optional debug:
                # self.get_logger().info(f"IN_OUT Current: {current}")
                pass
         # STATUS
        elif command_id == 58:
            adc = int.from_bytes(b[2:4], 'big', signed=True)
            self.get_logger().info(f"ADC: {adc}")

    def send_msg(self, compiled_msg: can.Message):
        self.bus.send(compiled_msg)

    def build_msg(self, command: str, value: int, vesc_id: int, raw: bool = False):
        command_map = {
            "CAN_PACKET_SET_DUTY": (0, 100000, False),
            "CAN_PACKET_SET_CURRENT": (1, 1000, False),
            "CAN_PACKET_SET_CURRENT_BRAKE": (2, 1000, False),
            "CAN_PACKET_SET_RPM": (3, 1, False),
            "CAN_PACKET_SET_POS": (4, 1000000, False),
            "CAN_PACKET_SET_CURRENT_REL": (10, 100000, False),
            "CAN_PACKET_SET_CURRENT_BRAKE_REL": (11, 100000, False),
            "CAN_PACKET_SET_CURRENT_HANDBRAKE": (12, 1000, False),
            "CAN_PACKET_SET_CURRENT_HANDBRAKE_REL": (13, 100000, False),
            "CAN_PACKET_STATUS": (9, 0, True),
            "CAN_PACKET_STATUS_2": (14, 0, True),
            "CAN_PACKET_STATUS_3": (15, 0, True),
            "CAN_PACKET_STATUS_4": (16, 0, True),
            "CAN_PACKET_STATUS_5": (27, 0, True),
            "CAN_PACKET_STATUS_6": (28, 0, True),
        }

        if command not in command_map:
            raise Exception(f"{command} with value {value} not known.")

        command_id, scaling, is_status = command_map[command]

        int_id = (command_id << 8) | vesc_id

        int_data = int(value * scaling)
        data = int_data.to_bytes(4, byteorder='big', signed=True)

        if raw:
            return bin(int_id), data

        return can.Message(
            arbitration_id=int_id,
            data=data,
            is_extended_id=True
        ), is_status


def main(args=None):
    rclpy.init(args=args)

    can_subscriber = CANSubscriber()

    rclpy.spin(can_subscriber)

    can_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()