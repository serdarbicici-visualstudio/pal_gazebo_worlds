#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import time

class GeometricGrasper(Node):
    def __init__(self):
        super().__init__('geometric_grasper')
        
        # --- DYNAMIC INPUTS ---
        # The robot uses these coordinates to calculate angles.
        # You can change these, and the robot will adapt automatically.
        self.target_x = 0.55  # Distance forward (meters)
        self.target_z = 1.16  # Height (meters)
        
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self.get_logger().info('⏳ Waiting for controllers...')
        self.torso_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        self.get_logger().info(f'✅ Solver Ready. Planning reach to X={self.target_x}, Z={self.target_z}')
        self.execute_grasp()

    def solve_ik(self, target_x, target_z):
        """
        THE BRAIN: Calculates joint angles based on Target X, Z.
        Returns: [torso_height, arm_2, arm_4]
        """
        # 1. TORSO CALCULATION
        # Shoulder starts at 0.8m. We lift torso to match target height.
        required_lift = target_z - 0.80
        torso_val = max(0.0, min(0.35, required_lift)) # Clamp to limits
        
        # 2. ARM EXTENSION CALCULATION (Geometric IK)
        # We model the arm as a triangle to find elbow bend (arm_4).
        # We map distance (0.4m to 0.8m) to elbow angle (2.0rad to 0.2rad)
        
        # Clamp distance to reachable area
        reach_dist = max(0.35, min(0.75, target_x))
        
        # Linear Interpolation: 
        # Short reach (0.4m) -> Bent elbow (1.9 rad)
        # Long reach (0.7m) -> Straight elbow (0.3 rad)
        slope = (0.3 - 1.9) / (0.7 - 0.4) 
        elbow_angle = 1.9 + slope * (reach_dist - 0.4)
        
        return torso_val, elbow_angle

    def execute_grasp(self):
        # --- STEP 1: SOLVE THE MATH ---
        torso_lift, elbow_bend = self.solve_ik(self.target_x, self.target_z)
        self.get_logger().info(f'🧠 Solution Found: Torso={torso_lift:.2f}, Elbow={elbow_bend:.2f}')

        # --- STEP 2: OPEN GRIPPER ---
        self.send_gripper(0.045) # Open

        # --- STEP 3: MOVE TORSO ---
        self.send_torso(torso_lift)

        # --- STEP 4: MOVE ARM (Reach) ---
        # We inject the calculated 'elbow_bend' into the standard pose
        # [Turn, Shoulder, Elbow_Rot, ELBOW_BEND, Wrist_Rot, Wrist_Bend, Hand_Rot]
        reach_pose = [0.10, 0.00, -1.57, elbow_bend, -1.57, -0.50, 0.00]
        self.send_arm(reach_pose, duration=5.0)
        time.sleep(1.0)

        # --- STEP 5: GRASP ---
        self.send_gripper(0.01) # Close
        time.sleep(1.0)

        # --- STEP 6: RETRACT ---
        self.get_logger().info('🔙 Retracting...')
        retract_pose = [0.10, -0.50, -1.57, 1.80, -1.57, -0.50, 0.00] 
        self.send_arm(retract_pose, duration=3.0)
        
        self.get_logger().info('🏁 Dynamic Grasp Complete!')

    # --- HELPER FUNCTIONS ---
    def send_torso(self, value):
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = ['torso_lift_joint']
        point = JointTrajectoryPoint(positions=[float(value)], time_from_start=self.to_duration(4))
        msg.trajectory.points = [point]
        self.torso_client.send_goal_async(msg)
        # We don't wait here so arm can move while torso moves (optional)
        time.sleep(4.0) 

    def send_arm(self, positions, duration):
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        point = JointTrajectoryPoint(positions=[float(p) for p in positions], time_from_start=self.to_duration(duration))
        msg.trajectory.points = [point]
        self.arm_client.send_goal_async(msg)
        time.sleep(duration)

    def send_gripper(self, value):
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint(positions=[value, value], time_from_start=self.to_duration(2))
        msg.trajectory.points = [point]
        self.gripper_client.send_goal_async(msg)
        time.sleep(2.0)

    def to_duration(self, seconds):
        d = rclpy.duration.Duration(seconds=seconds)
        return d.to_msg()

def main():
    rclpy.init()
    node = GeometricGrasper()
    rclpy.shutdown()

if __name__ == '__main__':
    main()