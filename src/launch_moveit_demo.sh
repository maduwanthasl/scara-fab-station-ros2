#!/bin/bash
# Fix snap library conflicts with ROS
unset GTK_PATH
unset GTK_MODULES

# Set proper library path excluding snap
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# Source ROS
source /opt/ros/jazzy/setup.bash
source /home/shevi/sr_ws/install/setup.bash

# Launch MoveIt demo
ros2 launch arm_moveit_config demo.launch.py
