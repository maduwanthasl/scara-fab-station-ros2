from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("scara_fab_station_description")
    
    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, '..', '..') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    urdf_path = os.path.join(pkg_share, "urdf", "scara_fab_station.urdf")
    world_path = os.path.join(pkg_share, "world", "scara_world.sdf")

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

    # Joint State Publisher GUI (to control joints)
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Gazebo Sim with custom world
    gz = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r"],
        output="screen",
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_share, '..', '..')},
    )

    # Spawn robot into Gazebo on top of the table with delay
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name", "scara_fab_station", 
                    "-topic", "robot_description",
                    "-x", "0.0",
                    "-y", "0.0", 
                    "-z", "0.415",
                    "-Y", "0.0"
                ],
            )
        ]
    )

    # Bridge to sync joint states between Gazebo and ROS
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        output="screen",
    )

    # RViz2 - launch without config file
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        gz_resource_path,
        robot_state_publisher,
        joint_state_publisher,
        gz,
        spawn,
        bridge,
        rviz,
    ])
