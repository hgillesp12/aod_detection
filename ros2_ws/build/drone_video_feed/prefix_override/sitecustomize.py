import sys
if sys.prefix == '/Users/hannahgillespie/miniconda3/envs/ros2_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/hannahgillespie/aod_detection/ros2_ws/install/drone_video_feed'
