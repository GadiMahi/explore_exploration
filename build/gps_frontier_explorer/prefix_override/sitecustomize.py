import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/raj/ROS2/gps_frontier_exploration/explore_exploration/install/gps_frontier_explorer'
