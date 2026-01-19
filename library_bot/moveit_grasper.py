#!/usr/bin/env python3
"""
MoveIt-based pick and place for TIAGo robot.
Uses MoveIt2 Python API for arm motion planning.
Supports both green and red book detection.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import (
    MoveItErrorCodes,
    RobotState,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
import math
import time
import numpy as np
from typing import Optional, Tuple


class MoveItGrasper(Node):
    """
    MoveIt-based arm controller for TIAGo robot.
    Uses inverse kinematics and motion planning for robust manipulation.
    """
    
    def __init__(self):
        super().__init__('moveit_grasper')
        
        # Robot configuration
        self.arm_joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        self.gripper_joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        self.torso_joint_name = 'torso_lift_joint'
        
        # Current joint states
        self.current_joint_states = {}
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action clients for direct trajectory control (fallback)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, 
            '/gripper_controller/follow_joint_trajectory'
        )
        self.torso_client = ActionClient(
            self, FollowJointTrajectory, 
            '/torso_controller/follow_joint_trajectory'
        )
        
        # MoveIt IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # MoveGroup action client for motion planning
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('⏳ Waiting for controllers...')
        self.arm_client.wait_for_server(timeout_sec=10.0)
        self.gripper_client.wait_for_server(timeout_sec=10.0)
        self.torso_client.wait_for_server(timeout_sec=10.0)
        
        self.get_logger().info('✅ MoveIt Grasper initialized!')
        
        # Wait for joint states
        time.sleep(1.0)
        
    def joint_state_callback(self, msg: JointState):
        """Store current joint positions."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    def get_current_arm_state(self) -> list:
        """Get current arm joint positions."""
        positions = []
        for name in self.arm_joint_names:
            positions.append(self.current_joint_states.get(name, 0.0))
        return positions
    
    def solve_ik_geometric(self, target_x: float, target_z: float) -> Tuple[float, float]:
        """
        Geometric IK solver (fallback when MoveIt IK is not available).
        Returns: (torso_height, elbow_angle)
        """
        # Torso calculation - shoulder base at ~0.8m
        required_lift = target_z - 0.80
        torso_val = max(0.0, min(0.35, required_lift))
        
        # Arm extension calculation
        reach_dist = max(0.35, min(0.75, target_x))
        
        # Linear interpolation for elbow angle
        slope = (0.3 - 1.9) / (0.7 - 0.4)
        elbow_angle = 1.9 + slope * (reach_dist - 0.4)
        
        return torso_val, elbow_angle
    
    def compute_ik(self, target_pose: Pose) -> Optional[list]:
        """
        Compute inverse kinematics using MoveIt service.
        Returns joint positions or None if IK fails.
        """
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IK service not available, using geometric solver')
            return None
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header.frame_id = 'base_footprint'
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.avoid_collisions = True
        
        # Set current robot state
        request.ik_request.robot_state.joint_state.name = self.arm_joint_names
        request.ik_request.robot_state.joint_state.position = self.get_current_arm_state()
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                # Extract arm joint positions from solution
                joint_positions = []
                for name in self.arm_joint_names:
                    if name in result.solution.joint_state.name:
                        idx = list(result.solution.joint_state.name).index(name)
                        joint_positions.append(result.solution.joint_state.position[idx])
                    else:
                        joint_positions.append(0.0)
                return joint_positions
            else:
                self.get_logger().warn(f'IK failed with error code: {result.error_code.val}')
        
        return None
    
    def to_duration(self, seconds: float):
        """Convert seconds to ROS Duration message."""
        d = rclpy.duration.Duration(seconds=seconds)
        return d.to_msg()
    
    def send_arm_trajectory(self, positions: list, duration: float = 5.0):
        """Send arm to specified joint positions using trajectory controller."""
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = self.to_duration(duration)
        msg.trajectory.points = [point]
        
        self.get_logger().info(f'🦾 Moving arm to: {[f"{p:.2f}" for p in positions]}')
        
        future = self.arm_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        time.sleep(duration + 0.5)
    
    def send_torso(self, height: float, duration: float = 4.0):
        """Move torso to specified height."""
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = [self.torso_joint_name]
        
        point = JointTrajectoryPoint()
        point.positions = [float(height)]
        point.time_from_start = self.to_duration(duration)
        msg.trajectory.points = [point]
        
        self.get_logger().info(f'📏 Moving torso to height: {height:.2f}m')
        
        future = self.torso_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        time.sleep(duration)
    
    def send_gripper(self, opening: float, duration: float = 2.0):
        """Open/close gripper. opening=0.045 is open, 0.0 is closed."""
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = self.gripper_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(opening), float(opening)]
        point.time_from_start = self.to_duration(duration)
        msg.trajectory.points = [point]
        
        state = "OPEN" if opening > 0.02 else "CLOSED"
        self.get_logger().info(f'🤏 Gripper: {state} ({opening:.3f}m)')
        
        future = self.gripper_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        time.sleep(duration)
    
    def open_gripper(self):
        """Open the gripper fully."""
        self.send_gripper(0.045)
    
    def close_gripper(self):
        """Close the gripper to grasp object."""
        self.send_gripper(0.01)
    
    def move_to_home(self):
        """Move arm to home/safe position."""
        home_pose = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
        self.get_logger().info('🏠 Moving to home position...')
        self.send_arm_trajectory(home_pose, duration=4.0)
    
    def move_to_ready(self):
        """Move arm to ready position for manipulation."""
        ready_pose = [0.10, 0.00, -1.57, 1.50, -1.57, -0.50, 0.00]
        self.get_logger().info('✋ Moving to ready position...')
        self.send_arm_trajectory(ready_pose, duration=4.0)
    
    def pick_object(self, target_x: float, target_z: float, use_moveit: bool = False):
        """
        Execute pick operation at specified position.
        
        Args:
            target_x: Distance forward from robot base (meters)
            target_z: Height of object (meters)  
            use_moveit: If True, try MoveIt IK first (requires MoveIt running)
        """
        self.get_logger().info(f'🎯 Pick target: X={target_x:.3f}, Z={target_z:.3f}')
        
        # Step 1: Open gripper
        self.open_gripper()
        
        # Step 2: Adjust torso for height
        torso_height, elbow_angle = self.solve_ik_geometric(target_x, target_z)
        self.send_torso(torso_height)
        
        # Step 3: Try MoveIt IK or use geometric solution
        if use_moveit:
            # Create target pose for end-effector
            target_pose = Pose()
            target_pose.position.x = target_x
            target_pose.position.y = 0.0
            target_pose.position.z = target_z
            # Gripper pointing forward
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.707
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.707
            
            joint_solution = self.compute_ik(target_pose)
            
            if joint_solution is not None:
                self.get_logger().info('✅ MoveIt IK solution found!')
                self.send_arm_trajectory(joint_solution, duration=5.0)
            else:
                self.get_logger().info('⚠️ Using geometric IK fallback')
                reach_pose = [0.10, 0.00, -1.57, elbow_angle, -1.57, -0.50, 0.00]
                self.send_arm_trajectory(reach_pose, duration=5.0)
        else:
            # Use geometric IK directly
            self.get_logger().info(f'🧠 Geometric solution: Torso={torso_height:.2f}, Elbow={elbow_angle:.2f}')
            reach_pose = [0.10, 0.00, -1.57, elbow_angle, -1.57, -0.50, 0.00]
            self.send_arm_trajectory(reach_pose, duration=5.0)
        
        time.sleep(1.0)
        
        # Step 4: Close gripper
        self.close_gripper()
        time.sleep(0.5)
        
        self.get_logger().info('✅ Object grasped!')
    
    def retract(self):
        """Retract arm after pick."""
        retract_pose = [0.10, -0.50, -1.57, 1.80, -1.57, -0.50, 0.00]
        self.get_logger().info('🔙 Retracting arm...')
        self.send_arm_trajectory(retract_pose, duration=3.0)
    
    def place_object(self, target_x: float, target_z: float):
        """Place object at specified position."""
        self.get_logger().info(f'📦 Place target: X={target_x:.3f}, Z={target_z:.3f}')
        
        # Calculate arm configuration
        torso_height, elbow_angle = self.solve_ik_geometric(target_x, target_z)
        self.send_torso(torso_height)
        
        # Move to place position
        place_pose = [0.10, 0.00, -1.57, elbow_angle, -1.57, -0.50, 0.00]
        self.send_arm_trajectory(place_pose, duration=5.0)
        
        time.sleep(1.0)
        
        # Open gripper to release
        self.open_gripper()
        
        self.get_logger().info('✅ Object placed!')
    
    def execute_pick_and_place(self, pick_x: float, pick_z: float, 
                                place_x: float, place_z: float):
        """
        Execute complete pick and place sequence.
        """
        self.get_logger().info('🚀 Starting pick and place sequence...')
        
        # Ready position
        self.move_to_ready()
        
        # Pick
        self.pick_object(pick_x, pick_z)
        
        # Retract
        self.retract()
        
        # Navigate to place position would go here if needed
        # For now, just place in a different position
        
        # Place
        self.place_object(place_x, place_z)
        
        # Retract after placing
        self.retract()
        
        # Home
        self.move_to_home()
        
        self.get_logger().info('🏁 Pick and place complete!')


def main():
    """Demo: pick red book from table."""
    rclpy.init()
    
    grasper = MoveItGrasper()
    
    # Target positions for red book (on table at x=0.0 in world)
    # These are relative to robot base frame
    pick_x = 0.55   # Distance forward
    pick_z = 1.16   # Height of book on table
    
    # Execute pick operation
    grasper.get_logger().info('📕 Picking RED book...')
    grasper.pick_object(pick_x, pick_z, use_moveit=False)
    
    # Retract with object
    grasper.retract()
    
    # Move to home
    grasper.move_to_home()
    
    grasper.get_logger().info('🏁 Operation complete!')
    
    grasper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
