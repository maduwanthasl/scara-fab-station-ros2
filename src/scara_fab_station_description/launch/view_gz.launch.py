from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("scara_fab_station_description")
    
    # Set Gazebo resource path to include the package installation directory
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, '..', '..') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    urdf_path = os.path.join(pkg_share, "urdf", "scara_fab_station.urdf")

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), 
        value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Gazebo (gz-sim) 
    gz = ExecuteProcess(
        cmd=["gz", "sim", "empty.sdf", "-r"],
        output="screen",
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_share, '..', '..')},
    )

    # Spawn entity into Gazebo from /robot_description with delay
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=["-name", "scara_fab_station", "-topic", "robot_description"],
            )
        ]
    )

    return LaunchDescription([
        gz_resource_path,
        rsp,
        gz,
        spawn
    ])
