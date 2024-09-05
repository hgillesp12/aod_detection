#!/bin/bash

# Initialize Conda for the current shell
echo "Launching simulation in Gazebo and RVIZ2..."
colcon build
ros2 launch drone_video_feed launch.py