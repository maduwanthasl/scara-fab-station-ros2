#!/bin/bash
# Fix snap library conflicts with ROS
unset GTK_PATH
unset GTK_MODULES

# Set proper library path excluding snap
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# Source ROS
source /opt/ros/jazzy/setup.bash

# Launch MoveIt Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py
