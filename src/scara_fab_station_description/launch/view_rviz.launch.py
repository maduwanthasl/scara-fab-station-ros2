from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("scara_fab_station_description")
    
    urdf_path = os.path.join(pkg_share, "urdf", "scara_fab_station.urdf")

    # Read URDF file
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Joint State Publisher with GUI (to control joints)
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "view_robot.rviz")] 
                  if os.path.exists(os.path.join(pkg_share, "rviz", "view_robot.rviz")) 
                  else []
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
