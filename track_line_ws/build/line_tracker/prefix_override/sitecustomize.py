import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robotics/_BenjaminVargas/ros2_line_tracking_car/ROS2-LineTracking-Car/track_line_ws/install/line_tracker'
