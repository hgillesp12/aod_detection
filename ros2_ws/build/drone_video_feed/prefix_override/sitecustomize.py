import sys
if sys.prefix == '/Users/hannahgillespie/miniconda3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/hannahgillespie/ros2_ws/install/drone_video_feed'
