#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def main():
    rclpy.init()
    node = Node('fix_arm_node')
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    print("⏳ Server aranıyor...")
    # 2 saniye bekle, bulamazsa hata ver
    if not client.wait_for_server(timeout_sec=5.0):
        print("❌ HATA: Action Server bulunamadı! 'ros2 action list' komutunu kontrol et.")
        return

    print("✅ Server bulundu! Gönderiliyor...")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
    
    p = JointTrajectoryPoint()
    # Kuğu Boynu (Yüksek ve Dik)
    p.positions = [2.0, 0.9, 0.0, 2.6, -0.5, 0.0, 0.0]
    p.time_from_start.sec = 4
    goal.trajectory.points = [p]

    client.send_goal_async(goal)
    print("🚀 Komut gönderildi!")
    
    # Komutun gitmesi için azıcık bekle
    import time
    time.sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()