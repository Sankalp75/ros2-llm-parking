#!/bin/bash
# Headless mode - no GUI but physics works

cd /home/Beast/.gemini/antigravity/scratch/ackermann_llm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_LOCALHOST_ONLY=1

# Run Gazebo without GUI (server only)
ros2 launch vehicle_gazebo full_demo.launch.py ollama_url:=http://172.17.0.1:11434 gz_args:="-s"
