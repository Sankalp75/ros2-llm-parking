#!/bin/bash
# Run this script INSIDE the distrobox (same distrobox session or new one)

cd /home/Beast/.gemini/antigravity/scratch/ackermann_llm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'Parallel park between the cars'}"
