#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class FinalGrasperMoveIt(Node):
    def __init__(self):
        super().__init__('final_grasper_moveit')
        
        # MoveIt ve Gripper Action Client'ları
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # 1. ADIM: Robotu masanın önünde tam hizaladığını varsayıyoruz (Serdar'ın görevi) [cite: 38]
        # Kitap koordinatlarını senin verdiğin değerlere göre "local" çerçeveye (base_footprint) çevirdik
        self.BOOK_X = 0.68  # Robottan ileri mesafe
        self.BOOK_Y = 0.0   # Tam merkez
        self.BOOK_Z = 0.85  # Masa üstü (0.775) + kitabın kavrama noktası [cite: 15]

        self.get_logger().info('📚 V52: 12. HAFTA MOVEIT OPERASYONU BAŞLIYOR...')
        self.execute_full_sequence()

    def control_gripper(self, position):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        # 0.044 tam açık, 0.0 tam kapalı
        point.positions = [position, position]
        point.time_from_start.sec = 1
        goal_msg.trajectory.points = [point]
        self._gripper_client.send_goal_async(goal_msg)

    def send_move_goal(self, x, y, z, ox, oy, oz, ow):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_footprint'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = ox
        target_pose.pose.orientation.y = oy
        target_pose.pose.orientation.z = oz
        target_pose.pose.orientation.w = ow
        
        # Hedefi MoveIt'e gönder 
        self._move_group_client.wait_for_server()
        self._move_group_client.send_goal_async(goal_msg)

    def execute_full_sequence(self):
        # 1. Gripper'ı Aç
        self.control_gripper(0.044)
        time.sleep(2)

        # 2. Yaklaşma Pozisyonu (Pre-grasp): Kitabın 10cm yukarısı
        self.get_logger().info('🏗️ MoveIt: Kitap üzerine yaklaşılıyor...')
        self.send_move_goal(self.BOOK_X, self.BOOK_Y, self.BOOK_Z + 0.1, 0.707, 0.707, 0.0, 0.0)
        time.sleep(6)

        # 3. Alçalış: Tam tutma noktası
        self.get_logger().info('📉 MoveIt: Alçalıyor...')
        self.send_move_goal(self.BOOK_X, self.BOOK_Y, self.BOOK_Z, 0.707, 0.707, 0.0, 0.0)
        time.sleep(4)

        # 4. TUT!
        self.get_logger().info('✊ TUTULUYOR...')
        self.control_gripper(0.0)
        time.sleep(2)

        # 5. KALDIR (Zafer!)
        self.get_logger().info('🏆 KALDIRILIYOR!')
        self.send_move_goal(self.BOOK_X, self.BOOK_Y, 1.1, 0.707, 0.707, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = FinalGrasperMoveIt()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()