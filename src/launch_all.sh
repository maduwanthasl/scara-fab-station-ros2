#!/bin/bash
# Fix snap library conflicts with ROS
unset GTK_PATH
unset GTK_MODULES

# Set proper library path excluding snap
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# Source ROS
source /opt/ros/jazzy/setup.bash
source /home/shevi/sr_ws/install/setup.bash

# Launch everything: Gazebo + RViz + Joint Controller
ros2 launch scara_fab_station_description view_all.launch.py
