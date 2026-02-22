import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gauss/Desktop/my_robot_ws/install/my_robot_pkg'
