import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from std_msgs.msg import String, Float32, Bool
=======
from std_msgs.msg import String, Float32
>>>>>>> 76a8d3727477b7d0c45c3973aaadd89eca38adbe
import can
import time

# See https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
class CANSubscriber(Node):

    def __init__(self):
        super().__init__('can_subscriber')
        # Subscriber for CAN requests


        self.pid_publisher = self.create_publisher(
            Float32,
            'pid',
            10)
        
        # Publishers for canbus data
        # NOTE: This may need to be tuned
        max_queue = 10
        timer_freq = 0.01 # seconds
        # self.temp_fet_publisher = self.create_publisher(Float32, 'temp_fet', max_queue)
        # self.temp_fet_publisher = self.create_publisher(Float32, 'temp_motor', max_queue)
        # self.temp_fet_publisher = self.create_publisher(Float32, 'current_in', max_queue)
        self.temp_fet_publisher = self.create_publisher(Float32, 'pid_position', max_queue)
        self.current_side_publisher = self.create_publisher(Bool, 'current_side_to_side',max_queue)
        self.current_side_msg = Bool()
        self.current_side_msg.data = True
        self.timer = self.create_timer(timer_freq, self.timer_callback)
        self.bus = can.Bus(channel='can0', interface='socketcan')

   

    def carousel_publish(self, car_pid_msg: Float32):
        self.pid_publisher.publish(car_pid_msg)
    
    def timer_callback(self):
        self.receive_canbus(2, carousel_publish_func = self.carousel_publish)

    def receive_canbus(self, num_messages: int, infty: bool = False, carousel_publish_func = None):
        """
        Queries the canbus for data. 

        Args:
            num_messages: Number of messages to parse before exiting. 
            infty: Whether to query the canbus until it runs out of messages. 
        """
        channel = 'can0'
        with can.Bus(channel=channel, interface='socketcan') as bus:
            # if you try to query for all messages in the canbus,
            # the canbus publishes more messages than you can parse
            i = 0
            CAROUSEL_VESC = 80
            for msg in self.bus:
                if i == num_messages and not infty:
                    break
                arb_id = msg.arbitration_id
                command_id = (arb_id >> 8) & 0xFF
                vesc_id = arb_id & 0xFF

                #self.get_logger().info(f"Command id {command_id} with vesc id {vesc_id}")

                b = msg.data

                match command_id:
                    case 16:
                        temp_fet = int.from_bytes(b[0:2], 'big', signed=True) / 10
                        temp_motor = int.from_bytes(b[2:4], 'big', signed=True) / 10
                        current = int.from_bytes(b[4:6], 'big', signed=True) / 10
                        pid_pos = int.from_bytes(b[6:8], 'big', signed=True) / 50
                        #self.get_logger().info(f"Temp fet:  {temp_fet}, temp mot:  {temp_motor}, current:  {current}, pid_pos:  {pid_pos}, ")
                        car_pid_msg = Float32()
                        car_pid_msg.data = pid_pos

                        if carousel_publish_func and vesc_id == CAROUSEL_VESC:
                            carousel_publish_func(car_pid_msg)
                            #self.get_logger().info(f"Car_pid_msg {car_pid_msg} with vesc id {vesc_id}")
                    case 9:
                        #self.get_logger().info(f"Bytes:  {b}")
                        current = int.from_bytes(b[4:6], 'big', signed=True) / 10
                        if(current > 19):
                            self.current_side_msg.data = False
                        else:
                            self.current_side_msg.data = True
                        self.current_side_publisher.publish(self.current_side_msg)
                        self.get_logger().info(f"Current {current} with vesc id {vesc_id}")
                            self.get_logger().info(f"Car_pid_msg {car_pid_msg} with vesc id {vesc_id}")
                i += 1

    def send_msg(self, compiled_msg: can.message.Message):
        """Immediately send a compiled CAN message"""
        #channel = 'can0'
        #print(f"Sending {compiled_msg.arbitration_id} with {compiled_msg.data}")
        #with can.Bus(channel=channel, interface='socketcan') as bus:
        self.bus.send(compiled_msg)

    def build_msg(self, command: str, value: int, vesc_id: int, raw: bool = False):
        """
        Builds a VESC message. Can be sent to VESC with bus.send(msg). 
        Simple commands set a value and are self-explanatory. 
        Status commands provide 2-4 values in response. Value is zeroed for status. 

        Possible commands:

        Single-Frame (simple) Commands
        - CAN_PACKET_SET_DUTY
        - CAN_PACKET_SET_CURRENT
        - CAN_PACKET_SET_CURRENT_BRAKE
        - CAN_PACKET_SET_RPM
        - CAN_PACKET_SET_POS
        - CAN_PACKET_SET_CURRENT_REL
        - CAN_PACKET_SET_CURRENT_BRAKE_REL
        - CAN_PACKET_SET_CURRENT_HANDBRAKE
        - CAN_PACKET_SET_CURRENT_HANDBRAKE_REL

        Status Commands
        - CAN_PACKET_STATUS
            - ERPM
            - Current
            - Duty Cycle
        - CAN_PACKET_STATUS_2
            - Amp Hours
            - Amp Hours Chg
        - CAN_PACKET_STATUS_3
            - Watt Hours
            - Watt Hours Chg
        - CAN_PACKET_STATUS_4
            - Temp FET
            - Temp Motor
            - Current In
            - PID Pos
        - CAN_PACKET_STATUS_5
            - Tachometer
            - Volts In
        - CAN_PACKET_STATUS_6
            - ADC1
            - ADC2
            - ADC3
            - PPM

        Args:
            command: name of command.
            value: value to send motor
            vesc_id: id of vesc unit

        Returns:
            PyCAN message object and whether the command was status (unpacked tuple). 
            Or, if raw == True, a tuple of (id(bitstring), data(bytes))
        """
        command_id = None
        scaling = None
        is_status = False

        # search for command
        match command:
            # Simple commands
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

            # Status commands
            case "CAN_PACKET_STATUS":
                # ERPM is RPM
                # Current is A
                # Duty Cycle is % / 100
                command_id = 9
                scaling = 0
                is_status = True
            case "CAN_PACKET_STATUS_2":
                # Amp Hours is Ah
                # Amp Hours Chg is Ah
                command_id = 14
                scaling = 0
                is_status = True
            case "CAN_PACKET_STATUS_3":
                # Watt Hours is Wh
                # Watt Hours Chg is Wh
                command_id = 15
                scaling = 0
                is_status = True
            case "CAN_PACKET_STATUS_4":
                # Temp FET is DegC
                # Temp Motor is DegC
                # Current In is A
                # PID Pos is Deg
                command_id = 16
                scaling = 0
                is_status = True
            case "CAN_PACKET_STATUS_5":
                # Tachometer is EREV
                # Volts In is V
                command_id = 27
                scaling = 0
                is_status = True
            case "CAN_PACKET_STATUS_6":
                # ADC1 is V
                # ADC2 is V
                # ADC3 is V
                # PPM is % / 100
                command_id = 28
                scaling = 0
                is_status = True

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
        data = int_data.to_bytes(4, byteorder='big', signed=True)

        # build pycan msg
        if raw:
            return id, data
        # VESC uses extended ids
        return can.Message(arbitration_id=int_id, data=data, is_extended_id=True), is_status

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
