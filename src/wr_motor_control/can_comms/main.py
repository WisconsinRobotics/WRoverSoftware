import can
from vesc_msg import build_msg

interface = 'socketcan'
channel = 'can0'

def send_msg(msg):
    """Sends one message via the arbitration_id param of can.Message"""
    with can.Bus(channel=channel, interface=interface) as bus:
        bus.send(msg)

msg = build_msg(command='CAN_PACKET_SET_POS', value=90, vesc_id=41)
print(msg)

send_msg(msg)
