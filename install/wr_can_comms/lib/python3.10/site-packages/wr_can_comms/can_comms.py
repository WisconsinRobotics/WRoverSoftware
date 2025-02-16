import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can

class CANSubscriber(Node):

    def __init__(self):
        super().__init__('can_subscriber')
        self.subscription = self.create_subscription(
            String,
            'can_msg',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        # Parse message for data
        # NOTE: Assuming "vesc_id COMMAND value value_type"
        can_msg = msg.data.split(' ')
        vesc_id = int(can_msg[0])
        command = can_msg[1]
        # Process value
        value_type = can_msg[3]
        if value_type == 'float':
            value = float(can_msg[2])
        elif value_type == 'int':
            value = int(can_msg[2])
        else:
            raise TypeError(f"Type {value_type} not known/used.")

        # Build message
        compiled_msg = build_msg(command=command, value=value, vesc_id=vesc_id)

        # TODO: Add capability to synchronize message consumption
        # Send message
        send_msg(compiled_msg=compiled_msg)

def send_msg(compiled_msg: can.message.Message):
    """Immediately send a compiled CAN message"""
    channel = 'can0'
    #print(f"Sending {compiled_msg.arbitration_id} with {compiled_msg.data}")
    with can.Bus(channel=channel, interface='socketcan') as bus:
        bus.send(compiled_msg)

def build_msg(command: str, value: int, vesc_id: int, raw: bool = False):
    """
    Builds a VESC message. Can be sent to VESC with bus.send(msg)

    Possible commands:

    - CAN_PACKET_SET_DUTY
    - CAN_PACKET_SET_CURRENT
    - CAN_PACKET_SET_CURRENT_BRAKE
    - CAN_PACKET_SET_RPM
    - CAN_PACKET_SET_POS
    - CAN_PACKET_SET_CURRENT_REL
    - CAN_PACKET_SET_CURRENT_BRAKE_REL
    - CAN_PACKET_SET_CURRENT_HANDBRAKE
    - CAN_PACKET_SET_CURRENT_HANDBRAKE_REL

    Args:
        command: name of command.
        value: value to send motor
        vesc_id: id of vesc unit

    Returns:
        PyCAN message object. 
        Or, if raw == True, a tuple of (id(bitstring), data(bytes))
    """
    command_id = None
    scaling = None

    # search for command
    match command:
        case "CAN_PACKET_SET_DUTY":
            # Unit is % / 100
            # Desc is Duty Cycle
            # Range is -1.0 to 1.0
            command_id = 0
            scaling = 100000
        case "CAN_PACKET_SET_CURRENT":
            # Unit is A
            # Desc is Motor Current
            # Range is -MOTOR_MAX to MOTOR_MAX
            command_id = 1
            scaling = 1000
        case "CAN_PACKET_SET_CURRENT_BRAKE":
            # Unit is A
            # Desc is Braking Current
            # Range is -MOTOR_MAX to MOTOR_MAX
            command_id = 2
            scaling = 1000
        case "CAN_PACKET_SET_RPM":
            # Unit is RPM
            # Desc is RPM
            # Range is -MAX_RPM to MAX_RPM
            command_id = 3
            scaling = 1
        case "CAN_PACKET_SET_POS":
            # Unit is Degrees
            # Range is 0 to 360
            command_id = 4
            scaling = 1000000
        case "CAN_PACKET_SET_CURRENT_REL":
            # Unit is % / 100
            # Range is -1.0 to 1.0
            command_id = 10
            scaling = 100000
        case "CAN_PACKET_SET_CURRENT_BRAKE_REL":
            # Unit is % / 100
            # Range is -1.0 to 1.0
            command_id = 11
            scaling = 100000
        case "CAN_PACKET_SET_CURRENT_HANDBRAKE":
            # Unit is A
            # Range is -MOTOR_MAX to MOTOR_MAX
            command_id = 12
            scaling = 1000
        case "CAN_PACKET_SET_CURRENT_HANDBRAKE_REL":
            # Unit is % / 100
            # Range is -1.0 to 1.0
            command_id = 13
            scaling = 100000
        case _:
            raise Exception(f"{command} with value {value} not known.")

    # build arbitration_id
    unused_bits = '0000000000000'
    command_bits = bin(command_id)[2:].zfill(8)
    vesc_bits = bin(vesc_id)[2:].zfill(8)
    # finalize arb_id
    id = unused_bits + command_bits + vesc_bits
    # specify base 2 for type conversion
    int_id = int(id, 2)

    # build data
    int_data = value * scaling
    int_data = int(int_data)
    # VESC uses big endian, and we need 4 bytes
    data = int_data.to_bytes(4, byteorder='big')

    # build pycan msg
    if raw:
        return id, data
    # VESC uses extended ids
    return can.Message(arbitration_id=int_id, data=data, is_extended_id=True)


def main(args=None):
    rclpy.init(args=args)

    can_subscriber = CANSubscriber()

    print("Starting CAN subscriber...")
    rclpy.spin(can_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
