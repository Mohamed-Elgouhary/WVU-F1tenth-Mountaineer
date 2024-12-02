import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/WVU-AD/mae00018/F1tenth_Github/pure_pursuit_ws/install/pure_pursuit_pkg'
