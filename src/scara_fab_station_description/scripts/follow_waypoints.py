#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
    RobotState
)
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import csv
import os
from ament_index_python.packages import get_package_share_directory
import time


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.get_logger().info("Initializing Waypoint Follower...")
        
        # Create action client for MoveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected!")
        
        # Joint names for the arm group
        self.joint_names = ['joint_y', 'joint_z', 'joint_1', 'joint_2']
        
        # Define zero and home poses
        self.zero_pose = [0.0, 0.0, 0.0, 0.0]  # All joints at zero
        self.home_pose = [0.45, 0.3, 1.57, -1.57]  # Home position
        
        # Create marker publisher for path visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_path', 10)
        
        # Store path points for visualization
        self.path_points = []
        
        # Load waypoints from CSV
        self.waypoints = self.load_waypoints()
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        
    def load_waypoints(self):
        """Load waypoints from CSV file"""
        waypoints = []
        package_path = get_package_share_directory('scara_fab_station_description')
        csv_path = os.path.join(package_path, 'waypoints', 'star_waypoints_xyz_meters.csv')
        
        self.get_logger().info(f"Loading waypoints from: {csv_path}")
        
        with open(csv_path, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                waypoint = {
                    'x': float(row['x_m']),
                    'y': float(row['y_m']),
                    'z': float(row['z_m'])
                }
                waypoints.append(waypoint)
                self.get_logger().info(f"  Waypoint: x={waypoint['x']:.3f}, y={waypoint['y']:.3f}, z={waypoint['z']:.3f}")
        
        return waypoints
    
    def cartesian_to_joint_positions(self, x, y, z):
        """
        Convert Cartesian coordinates (end effector position) to joint positions for SCARA robot
        
        SCARA Robot Configuration from URDF:
        - Link 1 length (L1): 0.194m (joint_1 to joint_2)
        - Link 2 length (L2): 0.24m (joint_2 to end effector)
        - joint_y: Prismatic Y-axis
        - joint_z: Prismatic Z-axis
        - joint_1: Revolute around Z
        - joint_2: Revolute around -Z
        
        End effector position = joint_2_origin + 0.24m in the direction of link_2
        """
        import math
        
        # SCARA link lengths from URDF
        L1 = 0.194  # link_1 length (joint_1 to joint_2 origin in X)
        L2 = 0.240  # link_2 length (joint_2 to end effector in X)
        
        # Base offsets from URDF
        # joint_z is at position [0.227, -0.02, -0.017] relative to link_z origin
        base_x_offset = 0.227  # X offset of joint_1 from link_z
        
        # Prismatic joints directly control Y and Z position
        joint_y = y
        joint_z = z
        
        # For the revolute joints, we need to solve 2-link planar arm IK
        # The arm operates in the X-Y plane (from robot's perspective)
        # Target position in arm's coordinate frame
        target_x = x - base_x_offset  # Subtract base offset
        
        # Distance from joint_1 to end effector (2D distance in arm plane)
        distance = math.sqrt(target_x**2)
        
        # Check if target is reachable
        if distance > (L1 + L2):
            self.get_logger().warn(f"Target ({x:.3f}, {y:.3f}, {z:.3f}) is out of reach! Distance: {distance:.3f} > {L1+L2:.3f}")
            # Clamp to maximum reach
            scale = (L1 + L2) / distance
            target_x *= scale
            distance = L1 + L2
        
        if distance < abs(L1 - L2):
            self.get_logger().warn(f"Target too close! Distance: {distance:.3f} < {abs(L1-L2):.3f}")
            distance = abs(L1 - L2) + 0.01
        
        # Use law of cosines to find joint angles
        # Elbow-up configuration
        cos_joint2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_joint2 = max(-1.0, min(1.0, cos_joint2))  # Clamp to valid range
        
        joint_2_angle = math.acos(cos_joint2)  # Elbow angle
        
        # Calculate joint_1 angle
        alpha = math.atan2(0, target_x)  # Angle to target (0 for pure X direction)
        beta = math.atan2(L2 * math.sin(joint_2_angle), L1 + L2 * math.cos(joint_2_angle))
        joint_1_angle = alpha - beta
        
        # Since joint_2 rotates in opposite direction (axis="0 0 -1"), negate it
        joint_2_final = -joint_2_angle
        
        joint_positions = [joint_y, joint_z, joint_1_angle, joint_2_final]
        
        self.get_logger().info(f"IK: XYZ({x:.3f}, {y:.3f}, {z:.3f}) -> Joints({joint_y:.3f}, {joint_z:.3f}, {joint_1_angle:.3f}, {joint_2_final:.3f})")
        
        return joint_positions
    
    def publish_path_visualization(self):
        """Publish visualization markers for the waypoint path"""
        marker_array = MarkerArray()
        
        # Line strip marker for the path
        line_marker = Marker()
        line_marker.header.frame_id = "world"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "waypoint_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.01  # Line width - made thicker
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        
        for point in self.path_points:
            line_marker.points.append(point)
        
        marker_array.markers.append(line_marker)
        
        # Sphere markers for waypoints
        for i, point in enumerate(self.path_points):
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "world"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "waypoint_spheres"
            sphere_marker.id = i + 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = point
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.02  # Made bigger
            sphere_marker.scale.y = 0.02
            sphere_marker.scale.z = 0.02
            sphere_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
            
            marker_array.markers.append(sphere_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published path visualization with {len(self.path_points)} points")
    
    def forward_kinematics(self, joint_y, joint_z, joint_1, joint_2):
        """
        Calculate end effector position from joint angles
        Returns (x, y, z) in world coordinates
        """
        import math
        
        # SCARA link lengths from URDF
        L1 = 0.194  # link_1 length
        L2 = 0.240  # link_2 length to end effector
        
        # Base offsets from URDF
        base_x_offset = 0.227  # X offset of joint_1 from link_z
        
        # Calculate end effector position
        # joint_1 and joint_2 control the arm in X direction
        # Note: joint_2 has negative axis, so we negate it
        x = base_x_offset + L1 * math.cos(joint_1) + L2 * math.cos(joint_1 + joint_2)
        y = joint_y  # Y position from prismatic joint
        z = joint_z  # Z position from prismatic joint
        
        return (x, y, z)
    
    def add_path_point(self, x, y, z):
        """Add a point to the path visualization"""
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        self.path_points.append(point)
        self.publish_path_visualization()
    
    def create_move_group_goal(self, joint_positions):
        """Create a MoveGroup action goal"""
        goal = MoveGroup.Goal()
        
        # Set up the motion plan request
        goal.request = MotionPlanRequest()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Set goal constraints
        goal.request.goal_constraints.append(Constraints())
        
        for i, joint_name in enumerate(self.joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_positions[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal.request.goal_constraints[0].joint_constraints.append(joint_constraint)
        
        # Set planning options
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Plan and execute
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        
        return goal
    
    def move_to_pose(self, joint_positions, pose_name):
        """Move to a specific joint pose"""
        self.get_logger().info(f"\n=== Moving to {pose_name} ===")
        self.get_logger().info(f"Joint targets: {[f'{jp:.3f}' for jp in joint_positions]}")
        
        # Create goal
        goal = self.create_move_group_goal(joint_positions)
        
        # Send goal
        self.get_logger().info("Sending goal to MoveGroup...")
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for {pose_name}")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info(f"✓ Reached {pose_name}")
            return True
        else:
            self.get_logger().error(f"Failed to reach {pose_name}, error code: {result.error_code.val}")
            return False
    
    def execute_waypoints(self):
        """Execute trajectory through all waypoints"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("STARTING WAYPOINT FOLLOWING SEQUENCE")
        self.get_logger().info("="*60)
        
        # Step 1: Move to zero pose
        self.get_logger().info("\n[STEP 1/4] Moving to ZERO pose...")
        if not self.move_to_pose(self.zero_pose, "Zero Pose"):
            return False
        time.sleep(1.0)
        
        # Step 2: Move to home pose
        self.get_logger().info("\n[STEP 2/4] Moving to HOME pose...")
        if not self.move_to_pose(self.home_pose, "Home Pose"):
            return False
        time.sleep(1.0)
        
        # Step 3: Follow waypoints
        self.get_logger().info("\n[STEP 3/4] Following waypoints...")
        for i, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f"\n=== Moving to waypoint {i+1}/{len(self.waypoints)} ===")
            self.get_logger().info(f"Target: x={waypoint['x']:.3f}, y={waypoint['y']:.3f}, z={waypoint['z']:.3f}")
            
            # Convert to joint positions
            joint_positions = self.cartesian_to_joint_positions(
                waypoint['x'], waypoint['y'], waypoint['z']
            )
            
            self.get_logger().info(f"Joint targets: {[f'{jp:.3f}' for jp in joint_positions]}")
            
            # Create goal
            goal = self.create_move_group_goal(joint_positions)
            
            # Send goal
            self.get_logger().info("Sending goal to MoveGroup...")
            send_goal_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f"Goal rejected for waypoint {i+1}")
                return False
            
            self.get_logger().info("Goal accepted, waiting for result...")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result().result
            
            if result.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f"✓ Reached waypoint {i+1}")
                
                # Calculate actual end effector position using forward kinematics
                ee_x, ee_y, ee_z = self.forward_kinematics(
                    joint_positions[0], joint_positions[1], 
                    joint_positions[2], joint_positions[3]
                )
                
                # Add actual end effector position to path visualization
                self.add_path_point(ee_x, ee_y, ee_z)
                self.get_logger().info(f"End effector at: ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f})")
            else:
                self.get_logger().error(f"Failed to reach waypoint {i+1}, error code: {result.error_code.val}")
                return False
            
            # Small delay between waypoints
            time.sleep(0.5)
        
        self.get_logger().info("\n✓ All waypoints completed!")
        
        # Step 4: Return to home pose
        self.get_logger().info("\n[STEP 4/4] Returning to HOME pose...")
        if not self.move_to_pose(self.home_pose, "Home Pose"):
            return False
        time.sleep(1.0)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("WAYPOINT SEQUENCE COMPLETED SUCCESSFULLY!")
        self.get_logger().info("="*60)
        return True


def main(args=None):
    rclpy.init(args=args)
    
    try:
        follower = WaypointFollower()
        
        # Wait a bit for everything to be ready
        time.sleep(2.0)
        
        # Execute waypoints
        success = follower.execute_waypoints()
        
        if success:
            follower.get_logger().info("\n✓ Task completed successfully! Shutting down...")
        else:
            follower.get_logger().error("\n✗ Task failed!")
        
        # Clean shutdown
        time.sleep(1.0)
        follower.destroy_node()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
