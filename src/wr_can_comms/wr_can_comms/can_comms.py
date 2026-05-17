import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can
import time
import threading

# See https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md

# ---------------------------------------------------------------------------
# Command lookup table — (command_id, scaling, is_status)
# Replaces the large match/case block and makes adding new commands trivial.
# ---------------------------------------------------------------------------
COMMAND_TABLE: dict[str, tuple[int, int, bool]] = {
    # Simple commands
    "CAN_PACKET_SET_DUTY":                  (0,  100000, False),  # % / 100, range -1.0..1.0
    "CAN_PACKET_SET_CURRENT":               (1,    1000, False),  # A, range ±MOTOR_MAX
    "CAN_PACKET_SET_CURRENT_BRAKE":         (2,    1000, False),  # A, range ±MOTOR_MAX
    "CAN_PACKET_SET_RPM":                   (3,       1, False),  # RPM
    "CAN_PACKET_SET_POS":                   (4, 1000000, False),  # Degrees, 0..360
    "CAN_PACKET_SET_CURRENT_REL":           (10, 100000, False),  # % / 100
    "CAN_PACKET_SET_CURRENT_BRAKE_REL":     (11, 100000, False),  # % / 100
    "CAN_PACKET_SET_CURRENT_HANDBRAKE":     (12,   1000, False),  # A
    "CAN_PACKET_SET_CURRENT_HANDBRAKE_REL": (13, 100000, False),  # % / 100
    # Status commands (scaling unused; value is zeroed)
    "CAN_PACKET_STATUS":   (9,  0, True),   # ERPM, Current, Duty Cycle
    "CAN_PACKET_STATUS_2": (14, 0, True),   # Amp Hours, Amp Hours Chg
    "CAN_PACKET_STATUS_3": (15, 0, True),   # Watt Hours, Watt Hours Chg
    "CAN_PACKET_STATUS_4": (16, 0, True),   # Temp FET, Temp Motor, Current In, PID Pos
    "CAN_PACKET_STATUS_5": (27, 0, True),   # Tachometer, Volts In
    "CAN_PACKET_STATUS_6": (28, 0, True),   # ADC1, ADC2, ADC3, PPM
}

CHANNEL = "can0"


def build_msg(command: str, value: int | float, vesc_id: int, raw: bool = False):
    """
    Build a VESC CAN message from a command name, value, and VESC ID.

    Returns:
        (can.Message, is_status)  — or (id_bitstring, data_bytes) if raw=True.

    Raises:
        KeyError  if command is not in COMMAND_TABLE.
    """
    try:
        command_id, scaling, is_status = COMMAND_TABLE[command]
    except KeyError:
        raise ValueError(f"Unknown command: '{command}' (value={value})")

    # Arbitration ID: [13 unused bits][8 command bits][8 vesc bits]
    arb_id = (command_id << 8) | (vesc_id & 0xFF)

    # Payload: scaled integer, big-endian, 4 bytes, signed
    int_data = int(value * scaling)
    data = int_data.to_bytes(4, byteorder="big", signed=True)

    if raw:
        id_bits = bin(arb_id)[2:].zfill(29)  # 29-bit extended frame
        return id_bits, data

    return can.Message(arbitration_id=arb_id, data=data, is_extended_id=True), is_status


class CANSubscriber(Node):
    """
    ROS 2 node that subscribes to 'can_msg' and forwards messages to a VESC
    over a persistent CAN bus connection.

    Message format: "<vesc_id> <command> <value> <value_type>"
    Example:        "1 CAN_PACKET_SET_RPM 3000 int"
    """

    def __init__(self):
        super().__init__("can_subscriber")

        # Persistent CAN bus — opened once, reused for every message.
        self._bus = can.Bus(channel=CHANNEL, interface="socketcan")
        self.get_logger().info(f"CAN bus opened on {CHANNEL}")

        # Latest-value store: only the most recent message per (vesc_id, command)
        # is kept. For motor control, stale commands are harmful — if the bus is
        # saturated we want to send the newest setpoint, not a backlog of old ones.
        self._pending: dict[tuple, can.Message] = {}
        self._pending_lock = threading.Lock()
        self._pending_event = threading.Event()

        self._sender_thread = threading.Thread(
            target=self._sender_loop, daemon=True, name="can_sender"
        )
        self._sender_thread.start()

        self.subscription = self.create_subscription(
            String, "can_msg", self._listener_callback, 10
        )

    # ------------------------------------------------------------------
    # ROS callback — fast path: parse → enqueue, never blocks on CAN I/O
    # ------------------------------------------------------------------

    def _listener_callback(self, msg: String):
        try:
            lines = msg.data.strip().splitlines()

            for line in lines:
                parts = line.split()
                if len(parts) != 4:
                    raise ValueError(f"Expected 4 fields, got {len(parts)}: '{msg.data}'")

                vesc_id   = int(parts[0])
                command   = parts[1]
                raw_value = parts[2]
                vtype     = parts[3]

                value = _parse_value(raw_value, vtype)
                can_msg, is_status = build_msg(command=command, value=value, vesc_id=vesc_id)

                # TODO: handle status commands if manual status requests are needed
                if not is_status:
                    key = (vesc_id, command)
                    with self._pending_lock:
                        self._pending[key] = can_msg
                    self._pending_event.set()  # wake sender

        except Exception as e:
            self.get_logger().error(f"Failed to process message '{msg.data}': {e}")

    # ------------------------------------------------------------------
    # Sender thread — drains the queue and writes to the CAN bus
    # ------------------------------------------------------------------

    def _sender_loop(self):
        MAX_RETRIES = 5
        RETRY_DELAY = 0.001  # 1 ms between retries — one CAN frame at 500kbps is ~200µs

        while rclpy.ok():
            # Sleep until there's something to send
            self._pending_event.wait(timeout=0.1)
            self._pending_event.clear()

            # Drain the entire pending dict in one snapshot.
            # Any message that arrives while we're sending will sit in _pending
            # for the next iteration — we'll always send the freshest value.
            with self._pending_lock:
                snapshot = list(self._pending.values())
                self._pending.clear()

            for can_msg in snapshot:
                for attempt in range(MAX_RETRIES):
                    try:
                        self._bus.send(can_msg)
                        break
                    except can.CanError as e:
                        if attempt < MAX_RETRIES - 1:
                            time.sleep(RETRY_DELAY)
                        else:
                            self.get_logger().warn(
                                f"Dropping message after {MAX_RETRIES} retries: {e}"
                            )

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Shutting down CAN subscriber…")
        self._bus.shutdown()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_value(raw: str, vtype: str) -> int | float:
    """Convert a raw string value to the appropriate Python type."""
    match vtype:
        case "float":  return float(raw)
        case "int":    return int(raw)
        case "string": return raw
        case _:        raise TypeError(f"Unsupported value type: '{vtype}'")


def main(args=None):
    rclpy.init(args=args)
    node = CANSubscriber()
    print("Starting CAN subscriber…")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()