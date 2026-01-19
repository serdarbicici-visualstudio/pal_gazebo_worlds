#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

def main():
    rclpy.init()
    node = Node('test_head_node')
    pub = node.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
    
    node.get_logger().info('TEST BAŞLADI: Kafa Aşağı İniyor...')
    time.sleep(1) # Bağlantı otursun

    # 1. AŞAĞI BAK
    traj = JointTrajectory()
    traj.joint_names = ['head_1_joint', 'head_2_joint']
    p = JointTrajectoryPoint()
    p.positions = [0.0, 1.0] # 1.0 radyan (Aşağı)
    p.time_from_start.sec = 1 # 1 saniyede yap
    traj.points = [p]
    pub.publish(traj)
    
    node.get_logger().info('Komut gönderildi! (Aşağı)')
    time.sleep(3)

    # 2. YUKARI BAK
    p.positions = [0.0, 0.0] # 0.0 radyan (Düz)
    traj.points = [p]
    pub.publish(traj)
    node.get_logger().info('Komut gönderildi! (Yukarı)')

    rclpy.shutdown()

if __name__ == '__main__':
    main()