#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class FinalDemo(Node):
    def __init__(self):
        super().__init__('final_demo_node')
        
        # --- Publishers & Clients ---
        self.vel_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)
        
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        self.get_logger().info('🚀 FINAL DEMO SEQUENCE INITIATED')
        self.run_sequence()

    def send_body_cmd(self, client, joints, positions, duration_sec):
        """Generic function to move any part of the robot"""
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Controller for {joints[0]} not found!")
            return
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=duration_sec)
        goal.trajectory.points = [point]
        
        future = client.send_goal_async(goal)
        # We wait for the result to ensure step-by-step execution
        while rclpy.ok():
            if future.done(): break
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.5) # Stability pause

    def drive_straight(self, speed, distance):
        """Drives the base forward/backward blindly"""
        twist = Twist()
        twist.linear.x = speed
        
        # Calculate duration based on speed (Distance = Speed * Time)
        duration = abs(distance / speed)
        loops = int(duration * 10) # 10Hz publish rate
        
        for _ in range(loops):
            self.vel_pub.publish(twist)
            time.sleep(0.1)
        
        self.vel_pub.publish(Twist()) # Hard Stop
        time.sleep(0.5)

    def run_sequence(self):
        # --- STEP 1: PREPARE TORSO (Critical for TIAGo Reach) ---
        self.get_logger().info('1. Setting Torso Height...')
        # Lift to 0.35m (Ideal for Z=0.85 book height)
        self.send_body_cmd(self.torso_client, ['torso_lift_joint'], [0.35], 4)

        # --- STEP 2: LOOK AT TARGET ---
        self.get_logger().info('2. Eyes on Target...')
        self.send_body_cmd(self.head_client, ['head_1_joint', 'head_2_joint'], [0.0, -0.4], 2)

        # --- STEP 3: ARM TO READY POSE ---
        self.get_logger().info('3. Deploying Arm...')
        # This pose extends the arm straight out, wrist vertical for "side pinch"
        # Joints: [1, 2, 3, 4, 5, 6, 7]
        ready_pose = [1.60, 0.30, 0.00, 1.40, -1.57, 0.20, 0.00]
        self.send_body_cmd(self.arm_client, 
            ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'], 
            ready_pose, 5)

        # --- STEP 4: OPEN GRIPPER ---
        self.get_logger().info('4. Opening Gripper...')
        self.send_body_cmd(self.gripper_client, ['gripper_left_finger_joint', 'gripper_right_finger_joint'], [0.044, 0.044], 2)

        # --- STEP 5: FINAL APPROACH (The "Creep") ---
        self.get_logger().info('5. Final Approach (15cm)...')
        # Drive forward 15cm to slide fingers around the book spine
        self.drive_straight(0.1, 0.15) 

        # --- STEP 6: GRASP ---
        self.get_logger().info('6. PINCHING BOOK!')
        self.send_body_cmd(self.gripper_client, ['gripper_left_finger_joint', 'gripper_right_finger_joint'], [0.0, 0.0], 2)
        time.sleep(1.0) # Wait for friction

        # --- STEP 7: LIFT & RETREAT ---
        self.get_logger().info('7. Lifting Book...')
        # Lift arm slightly to clear the shelf
        lift_pose = [1.60, 1.00, 0.00, 1.50, -1.57, 0.20, 0.00]
        self.send_body_cmd(self.arm_client, 
            ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'], 
            lift_pose, 4)
        
        self.get_logger().info('8. Backing Away...')
        self.drive_straight(-0.2, 0.30) # Back up 30cm

        self.get_logger().info('✅ DEMO COMPLETE: Book Retrieved.')

def main():
    rclpy.init()
    try:
        node = FinalDemo()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()