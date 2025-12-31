#!/bin/bash
# Unset snap library paths that conflict with ROS
unset GTK_PATH
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu

# Source ROS
source /home/shevi/sr_ws/install/setup.bash

# Launch Gazebo with the robot
ros2 launch scara_fab_station_description view_gz.launch.py
