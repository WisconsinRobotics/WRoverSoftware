import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wiscrobo/WRoverMotorControl/WRoverSoftware/install/wr_can_comms'
