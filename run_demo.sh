#!/bin/bash
# Run this script INSIDE the distrobox

# 1. Setup Environment
cd /home/Beast/.gemini/antigravity/scratch/ackermann_llm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. Fix Network & Graphics
export ROS_LOCALHOST_ONLY=1
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=0
export MESA_GL_VERSION_OVERRIDE=3.3

# 3. Host IP (Hardcoded for stability)
HOST_IP="10.201.116.14"
echo "Using Host IP: $HOST_IP"

# 4. Launch with Dynamic IP
echo "Launching simulation... Connecting to Ollama at $HOST_IP"
ros2 launch vehicle_gazebo full_demo.launch.py ollama_url:=http://$HOST_IP:11434
