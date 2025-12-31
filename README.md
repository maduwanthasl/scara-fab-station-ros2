# SCARA Fabrication Station - ROS 2

A complete ROS 2 simulation package for a 4-DOF SCARA (Selective Compliance Assembly Robot Arm) robot with autonomous waypoint trajectory following and real-time path visualization.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![License](https://img.shields.io/badge/license-MIT-green)

## Overview

This project demonstrates a complete robotic manipulation system featuring:
- **SCARA robot simulation** in Gazebo with custom fabrication table environment
- **MoveIt 2 motion planning** with multiple planning algorithms
- **Inverse kinematics solver** for Cartesian waypoint conversion
- **Autonomous trajectory following** with CSV-based waypoint system
- **Real-time path visualization** showing end effector trajectory in RViz


## Demo Videos

### Waypoint Following Simulation
![Cube Toolpath]((https://github.com/maduwanthasl/scara-fab-station-ros2/blob/main/videos/cube_Toolpath.webm))
*Autonomous waypoint following with real-time path visualization*

### MoveIt Planning and Execution
![Plan and Execute]((https://github.com/maduwanthasl/scara-fab-station-ros2/blob/main/videos/plan_and_exhicute_robot.webm))
*Interactive motion planning and trajectory execution with MoveIt 2*

### URDF Visualization in RViz
![SCARA in RViz](https://github.com/maduwanthasl/scara-fab-station-ros2/blob/main/videos/scara_fab_station_rviz_control.webm)
*Robot model loaded in RViz with joint state control visualization*

### Zero and Home Pose Sequence
![Zero and Home Pose](https://github.com/maduwanthasl/scara-fab-station-ros2/blob/main/videos/zero_and_home.webm)
*Automated sequence: Zero pose → Home pose initialization*


## Features

### 🤖 Robot Configuration
- **4-DOF SCARA Robot**
  - 2 prismatic joints (Y-axis, Z-axis) for vertical positioning
  - 2 revolute joints for XY plane manipulation
  - Link lengths: L1=0.194m, L2=0.240m (to end effector)
  - Colored visual representation for easy identification

### 🎮 Motion Planning
- **MoveIt 2 Integration** (v2.12.3)
  - OMPL (RRTConnect) planner
  - STOMP, CHOMP, and Pilz Industrial Motion planners
  - Joint trajectory execution with configurable limits
  - Collision detection and path optimization

### 🎯 Waypoint System
- **CSV-Based Waypoints**: Define trajectories in simple X,Y,Z format
- **Inverse Kinematics**: Automatic conversion from Cartesian to joint space
- **Forward Kinematics**: Real-time end effector position calculation
- **Programmable Sequences**: Zero pose → Home pose → Waypoints → Home pose

### 📊 Visualization
- **MarkerArray Path Visualization**
  - Green line strip showing continuous trajectory
  - Red spheres marking waypoint positions
  - Real-time updates as robot moves
  - Frame-accurate positioning

### 🌍 Simulation Environment
- **Custom Gazebo World**
  - 2m × 2m fabrication table (0.4m height)
  - Realistic physics simulation
  - Ground plane and table with legs

## System Requirements

- **OS**: Ubuntu 24.04 LTS
- **ROS**: ROS 2 Jazzy Jalisco
- **Gazebo**: Gazebo Sim (Harmonic)
- **MoveIt**: MoveIt 2 (v2.12.3)
- **Python**: Python 3.12+

## Installation

### 1. Install ROS 2 Jazzy

```bash
# Set up sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

### 2. Install Dependencies

```bash
# Install MoveIt 2 and related packages
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gazebo-ros2-control \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions
```

### 3. Clone and Build

```bash
# Create workspace
mkdir -p ~/sr_ws/src
cd ~/sr_ws/src

# Clone repository
git clone https://github.com/maduwanthasl/scara-fab-station-ros2/ scara_fab_station_description

# Build workspace
cd ~/sr_ws
colcon build

# Source workspace
source install/setup.bash
```

## Usage

### Quick Start

#### Terminal 1: Launch Simulation and MoveIt

```bash
cd ~/sr_ws
source install/setup.bash
./src/scara_fab_station_description/launch/launch_all.sh
```

This launches:
- Gazebo simulation with robot and table
- RViz with MoveIt planning interface
- ros2_control controllers
- MoveIt move_group node

#### Terminal 2: Run Waypoint Follower

```bash
cd ~/sr_ws
source install/setup.bash
ros2 launch scara_fab_station_description follow_waypoints.launch.py
```

#### Terminal 3: Visualize Path (in RViz)

1. Click **Add** button in RViz
2. Select **MarkerArray** (by display type)
3. Set topic to `/waypoint_path`
4. Watch the green path and red waypoint spheres appear!

### Execution Sequence

The robot follows this automated sequence:

1. **Zero Pose**: `[0.0, 0.0, 0.0, 0.0]` - All joints at origin
2. **Home Pose**: `[0.45, 0.3, 1.57, -1.57]` - Safe starting position
3. **Waypoint Following**: Execute all waypoints from CSV file
4. **Return Home**: `[0.45, 0.3, 1.57, -1.57]` - Return to home position
5. **Shutdown**: Automatic node termination

### Custom Waypoints

Create your own waypoint file in CSV format:

```csv
x_m,y_m,z_m
0.200,0.020,0.000
0.250,0.100,0.050
0.300,0.020,0.100
...
```

Place it in: `install/scara_fab_station_description/share/scara_fab_station_description/waypoints/`

Modify launch file to use your CSV:
```python
waypoints_file='your_waypoints.csv'
```

## Package Structure

```
sr_ws/
├── src/
│   └── scara_fab_station_description/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── urdf/
│       │   ├── scara_fab_station.urdf          # Robot description
│       │   └── scara_world.sdf                 # Gazebo world
│       ├── meshes/                             # STL mesh files
│       ├── launch/
│       │   ├── view_all.launch.py              # Main launch file
│       │   ├── follow_waypoints.launch.py      # Waypoint follower launch
│       │   ├── launch_all.sh                   # Wrapper script (fixes snap conflicts)
│       │   └── launch_moveit_demo.sh           # MoveIt launch wrapper
│       ├── scripts/
│       │   └── follow_waypoints.py             # Waypoint follower node
│       ├── waypoints/
│       │   └── star_waypoints_xyz_meters.csv   # Example waypoints (star pattern)
│       ├── rviz/
│       │   └── view_all.rviz                   # RViz configuration
│       └── arm_moveit_config/                  # MoveIt configuration
│           └── config/
│               ├── moveit_controllers.yaml
│               ├── joint_limits.yaml
│               └── ros2_controllers.yaml
├── build/
├── install/
└── log/
```

## Technical Details

### Robot Kinematics

**Joint Configuration:**
- `joint_y`: Prismatic Y-axis (range: -2.0 to 2.0m, velocity: 3.0m/s)
- `joint_z`: Prismatic Z-axis (range: -2.0 to 2.0m, velocity: 3.0m/s)
- `joint_1`: Revolute Z-axis (range: ±π rad, velocity: 1.5rad/s)
- `joint_2`: Revolute -Z-axis (range: ±π rad, velocity: 1.5rad/s)

**Inverse Kinematics:**
```python
# 2-link planar IK using law of cosines
L1 = 0.194m  # Link 1 length
L2 = 0.240m  # Link 2 length (to end effector)

# Elbow-up configuration
cos_θ2 = (d² - L1² - L2²) / (2·L1·L2)
θ2 = arccos(cos_θ2)
θ1 = arctan2(y_target, x_target) - arctan2(L2·sin(θ2), L1 + L2·cos(θ2))
```

**Forward Kinematics:**
```python
# End effector position from joint angles
x = base_offset + L1·cos(θ1) + L2·cos(θ1 + θ2)
y = joint_y
z = joint_z
```

### MoveIt Configuration

**Planning Pipeline:**
- Primary Planner: OMPL (RRTConnect)
- Planning Time: 5 seconds
- Planning Attempts: 10
- Velocity Scaling: 0.5
- Acceleration Scaling: 0.5

**Controllers:**
- `arm_controller`: FollowJointTrajectory action
- `joint_state_broadcaster`: Joint state publisher

### Visualization

**MarkerArray Topics:**
- `/waypoint_path`: Combined LINE_STRIP and SPHERE markers
  - Green line (0.01m width): Continuous trajectory path
  - Red spheres (0.02m diameter): Individual waypoint positions
  - Frame: `world`
  - Updates in real-time during execution

## Troubleshooting

### Issue: Snap Library Conflicts

**Symptom:** `libpthread.so.0: cannot allocate memory in static TLS block`

**Solution:** Use provided wrapper scripts that clean environment variables:
```bash
./src/scara_fab_station_description/launch/launch_all.sh
```

### Issue: Controller Not Found

**Symptom:** `Returned 0 controllers in list`

**Solution:** Check controller configuration:
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Ensure `arm_controller` is loaded and active.

### Issue: Planning Failures

**Symptom:** `PLANNING_FAILED` or `INVALID_MOTION_PLAN`

**Solutions:**
- Increase planning time in `follow_waypoints.py`
- Check joint limits in `joint_limits.yaml`
- Verify waypoints are reachable (within workspace)
- Check for collisions in RViz planning scene

### Issue: Path Not Visible

**Symptom:** MarkerArray not showing in RViz

**Solutions:**
1. Add MarkerArray display: **Add → MarkerArray**
2. Set topic: `/waypoint_path`
3. Check Fixed Frame: Should be `world` or `base_footprint`
4. Verify markers are being published: `ros2 topic echo /waypoint_path`

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

### Development Guidelines

1. Follow ROS 2 coding standards
2. Test changes in simulation before submitting
3. Update documentation for new features
4. Add example waypoint files for new use cases

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Built with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- Motion planning by [MoveIt 2](https://moveit.ros.org/)
- Simulation with [Gazebo](https://gazebosim.org/)

## Citation

If you use this project in your research, please cite:

```bibtex
@software{scara_fab_station_ros2,
  title = {SCARA Fabrication Station - ROS 2 Autonomous Waypoint Follower},
  author = {Hirusha Maduwantha},
  year = {2025},
  url = {https://github.com/maduwathasl/scara-fab-station-ros2}
}
```

## Contact

For questions or support, please open an issue on GitHub.

---
