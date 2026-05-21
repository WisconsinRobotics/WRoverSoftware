# Initial Coder: AREN. Optimized with chat and Claude
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

SIDE_TO_SIDE_VESC = 80
IN_OUT_VESC = 82
FR_SWERVE_VESC = 73
UP_DOWN_VESC = 81

FL_VESC = 70
FR_VESC = 72
BL_VESC = 74
BR_VESC = 76

class CANSubscriber(Node):

    def __init__(self):
        super().__init__('can_subscriber')
        # Publishers
        self.pid_publisher = self.create_publisher(Float32, 'pid_CAROUSEL', 10)
        self.pid_FL_publisher = self.create_publisher(Float32, 'pid_FL', 10)
        self.pid_FR_publisher = self.create_publisher(Float32, 'pid_FR', 10)
        self.pid_BL_publisher = self.create_publisher(Float32, 'pid_BL', 10)
        self.pid_BR_publisher = self.create_publisher(Float32, 'pid_BR', 10)
        self.current_side_publisher = self.create_publisher(Bool, 'current_side_to_side', 10)
        self.current_in_out_publisher = self.create_publisher(Bool, 'current_in_out', 10)
        self.current_up_down_publisher = self.create_publisher(Bool, 'current_up_down', 10)

        self.current_side_msg = Bool()
        self.current_side_msg.data = True

        self.current_in_out_msg = Bool()
        self.current_in_out_msg.data = True

        self.current_up_down_msg = Bool()
        self.current_up_down_msg.data = True

        # Faster timer for lower latency
        timer_freq = 0.002  # 2 ms instead of 10 ms
        self.timer = self.create_timer(timer_freq, self.timer_callback)

        # Open CAN bus ONCE
        self.bus = can.Bus(
            channel='can0',
            interface='socketcan',
            can_filters=[
                # STATUS (cmd 9) for CAROUSEL and IN_OUT only (the only ones handled)
                {"can_id": (9 << 8) | SIDE_TO_SIDE_VESC, "can_mask": 0xFFFF, "extended": True},
                {"can_id": (9 << 8) | IN_OUT_VESC,   "can_mask": 0xFFFF, "extended": True},
                {"can_id": (9 << 8) | UP_DOWN_VESC,   "can_mask": 0xFFFF, "extended": True},

                # STATUS_4 (cmd 16) for all wheel VESCs and CAROUSEL
                {"can_id": (16 << 8) | SIDE_TO_SIDE_VESC, "can_mask": 0xFFFF, "extended": True},
                {"can_id": (16 << 8) | FL_VESC,       "can_mask": 0xFFFF, "extended": True},
                {"can_id": (16 << 8) | FR_VESC,       "can_mask": 0xFFFF, "extended": True},
                {"can_id": (16 << 8) | BL_VESC,       "can_mask": 0xFFFF, "extended": True},
                {"can_id": (16 << 8) | BR_VESC,       "can_mask": 0xFFFF, "extended": True},
            ]
        )

    def FL_publish(self, pid_value: float):
        msg = Float32()
        msg.data = pid_value
        self.pid_FL_publisher.publish(msg)
    def FR_publish(self, pid_value: float):
        msg = Float32()
        msg.data = pid_value
        self.pid_FR_publisher.publish(msg)
    def BL_publish(self, pid_value: float):
        msg = Float32()
        msg.data = pid_value
        self.pid_BL_publisher.publish(msg)
    def BR_publish(self, pid_value: float):
        msg = Float32()
        msg.data = pid_value
        self.pid_BR_publisher.publish(msg)

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

            if vesc_id == FL_VESC:
                pid_pos = int.from_bytes(b[6:8], 'big', signed=True) / 50
                self.FL_publish(pid_pos)
            elif vesc_id == FR_VESC:
                pid_pos = int.from_bytes(b[6:8], 'big', signed=True) / 50
                self.FR_publish(pid_pos)
            elif vesc_id == BL_VESC:
                pid_pos = int.from_bytes(b[6:8], 'big', signed=True) / 50
                self.BL_publish(pid_pos)
            elif vesc_id == BR_VESC:
                pid_pos = int.from_bytes(b[6:8], 'big', signed=True) / 50
                self.BR_publish(pid_pos)

        # STATUS
        elif command_id == 9:
            current = int.from_bytes(b[4:6], 'big', signed=True) / 10

            if vesc_id == SIDE_TO_SIDE_VESC:
                self.current_side_msg.data = current <= 19
                self.current_side_publisher.publish(self.current_side_msg)
                #self.get_logger().info(f"SIDE_TO_SIDE Current: {current}")

            elif vesc_id == IN_OUT_VESC:
                # Fast current access here
                self.current_in_out_msg.data = current <= 1.35
                self.current_in_out_publisher.publish(self.current_in_out_msg)
                # Optional debug:
                # self.get_logger().info(f"IN_OUT Current: {current}")
            elif vesc_id == UP_DOWN_VESC:
                # Fast current access here
                self.current_up_down_msg.data = current <= 19
                self.current_up_down_publisher.publish(self.current_up_down_msg)
                # Optional debug:
                #self.get_logger().info(f"UP_DOWN Current: {current}")

    def send_msg(self, compiled_msg: can.Message):
        self.bus.send(compiled_msg)

    

def main(args=None):
    rclpy.init(args=args)

    can_subscriber = CANSubscriber()

    rclpy.spin(can_subscriber)

    can_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()