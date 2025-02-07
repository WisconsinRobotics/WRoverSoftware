import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/devansh/WRoverMotorControl/WRoverSoftware/install/wr_swerve_control'
