import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/devansh/WRoverForPushing/WRoverSoftware/install/wr_xbox_controller'
