import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wiscrobo/WRoverSoftware/install/wr_xbox_controller'
