from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    
    # Declare arguments
    waypoints_file = LaunchConfiguration('waypoints_file')
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='star_waypoints_xyz_meters.csv',
        description='Name of the waypoints CSV file'
    )
    
    # Waypoint follower node using the installed executable
    waypoint_follower_node = Node(
        package='scara_fab_station_description',
        executable='follow_waypoints.py',
        name='waypoint_follower',
        output='screen',
        parameters=[
            {'waypoints_file': waypoints_file}
        ]
    )
    
    return LaunchDescription([
        waypoints_file_arg,
        waypoint_follower_node,
    ])
