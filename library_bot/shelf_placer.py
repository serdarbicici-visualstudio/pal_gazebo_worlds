#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import numpy as np

class TiagoShelfFinal(Node):
    def __init__(self):
        super().__init__('tiago_shelf_final_node')
        
        # --- AYARLAR ---
        self.turn_angle = 7.5  # 135 Derece (Radyan cinsinden: 2.356)
        self.shelf_dist = 1.4  # Rafa 70cm kala dur
        self.torso_h = 0.28     # Raf yüksekliği
        # ---------------

        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)
        
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/parallel_gripper_right_controller/follow_joint_trajectory')

        self.front_dist = 99.9
        self.state = "BACK_OFF" 
        self.start_time = time.time()
        
        # Dönüş için zamanlayıcı değişkenleri
        self.rot_start_time = 0
        self.rotation_duration = 0
        self.rotation_started = False

        self.action_done = False

        self.get_logger().info('🚀 BAŞLADI: Geri -> 135° Sola Dön -> Asansör -> İleri')

        self.torso_client.wait_for_server()
        self.arm_client.wait_for_server()
        
        self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        mid = len(msg.ranges) // 2
        window = 20
        valid = [r for r in msg.ranges[mid-window : mid+window] if not np.isinf(r) and not np.isnan(r)]
        if len(valid) > 0:
            self.front_dist = min(valid)

    def control_loop(self):
        # --- ADIM 1: GERİ GİT ---
        if self.state == "BACK_OFF":
            if (time.time() - self.start_time) < 3.0: 
                self.drive(-0.3, 0.0)
                self.get_logger().info('🔙 Geri çıkılıyor...', once=True)
            else:
                self.stop()
                self.state = "ROTATE"
                self.get_logger().info('✅ Geri çıkış bitti. Dönüş başlıyor.')

        # --- ADIM 2: DÖNÜŞ (NON-BLOCKING) ---
        elif self.state == "ROTATE":
            if not self.rotation_started:
                # İlk giriş: Süreyi hesapla
                speed = 0.5
                self.rotation_duration = abs(self.turn_angle / speed)
                self.rot_start_time = time.time()
                self.rotation_started = True
                self.get_logger().info(f'🔄 Dönüş Başladı! Hedef: {self.turn_angle} rad ({self.rotation_duration:.1f} sn)')

            # Süre kontrolü
            if (time.time() - self.rot_start_time) < self.rotation_duration:
                # Hala dönmem lazım
                self.drive(0.0, 0.5) # +0.5 = Sola Dönüş (CCW)
            else:
                # Süre bitti
                self.stop()
                self.state = "LIFT_TORSO"
                self.action_done = False
                self.get_logger().info('✅ Dönüş Tamamlandı!')
                time.sleep(1.0) # Durulması için kısa bir bekleme

        # --- ADIM 3: TORSOYU KALDIR ---
        elif self.state == "LIFT_TORSO":
            if not self.action_done:
                self.lift_torso()
                self.action_done = True

        # --- ADIM 4: RAFA GİT ---
        elif self.state == "APPROACH_SHELF":
            if self.front_dist > self.shelf_dist:
                self.drive(0.2, 0.0)
                self.get_logger().info(f'👀 Raf aranıyor... Mesafe: {self.front_dist:.2f}m', once=True)
            else:
                self.stop()
                self.get_logger().info('🛑 Raf Bulundu!')
                self.state = "PLACE_OBJECT"
                self.action_done = False

    # --- AKSİYONLAR ---
    def lift_torso(self):
        self.get_logger().info('⬆️ Asansör...')
        self.send_torso_traj(self.torso_h, 4.0)
        time.sleep(4.0)
        self.state = "APPROACH_SHELF"
        self.action_done = False

    def place_sequence(self):
        self.get_logger().info('🦾 Yerleştiriliyor...')
        place_pose = [1.60, 0.0, 0.0, 0.0, 1.57, 0.0, -1.57]
        self.send_arm_traj(place_pose, 4.0)
        time.sleep(4.5)
        
        try:
            self.send_gripper(0.045)
            time.sleep(2.0)
        except: pass

        # Geri kaç
        start = time.time()
        while time.time() - start < 3.0:
            self.drive(-0.2, 0.0)
            time.sleep(0.1)
        self.stop()
        
        # Kapanış
        rest_pose = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]
        self.send_arm_traj(rest_pose, 3.0)
        self.send_torso_traj(0.0, 3.0)
        self.get_logger().info('🏁 BİTTİ.')

    # --- YARDIMCI METODLAR ---
    def drive(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.vel_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def send_arm_traj(self, positions, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start.sec = int(duration)
        goal.trajectory.points = [p]
        self.arm_client.send_goal_async(goal)

    def send_torso_traj(self, height, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['torso_lift_joint']
        p = JointTrajectoryPoint()
        p.positions = [height]
        p.time_from_start.sec = int(duration)
        goal.trajectory.points = [p]
        self.torso_client.send_goal_async(goal)

    def send_gripper(self, pos):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        p = JointTrajectoryPoint()
        p.positions = [pos, pos]
        p.time_from_start.sec = 2
        goal.trajectory.points = [p]
        self.gripper_client.send_goal_async(goal)

def main():
    rclpy.init()
    node = TiagoShelfFinal()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()