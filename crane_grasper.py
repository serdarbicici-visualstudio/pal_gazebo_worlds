#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
import time
import os  # Terminal komutu çalıştırmak için (Hile Kütüphanesi)

class HollywoodCrane(Node):
    def __init__(self):
        super().__init__('hollywood_crane')
        
        # --- İSTEMCİLER ---
        self.torso = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.head = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.vel_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)

        # --- HİLE KOORDİNATLARI (ELİN İÇİ) ---
        # Robotun eli indiğinde nerede duruyorsa, kitabı oraya ışınlayacağız.
        # Kitap -1.3, -2.8'deydi. Robot yaklaştı.
        # Tahmini Gripper Konumu (Robot -1.9'da, el uzandı -2.5 civarı)
        self.MAGIC_X = -1.30 
        self.MAGIC_Y = -2.55 # Elin uzandığı nokta (Burası tutmazsa oyna!)
        self.MAGIC_Z = 0.85  # Masadan havada, tam gripper ortası

        # --- "SWAN NECK" (Kuğu) Geometrisi (En Güvenli İniş) ---
        self.SWAN_POSE = [1.60, 0.90, 0.00, 1.57, -0.90, 0.00, 0.00] 
        self.SAFE_TRAVEL = [0.20, 1.30, -0.20, 1.50, -1.57, 0.00, 0.00] 
        self.LIFT_UP     = [1.60, 1.20, 0.00, 1.00, -1.57, 0.00, 0.00] 

        self.get_logger().info('🎬 V72: HOLLYWOOD MODU (Işınlanma Aktif)...')
        self.run_mission()

    def cheat_teleport(self):
        """ Kitabı elin içine ışınlayan sihirli fonksiyon """
        self.get_logger().warn('✨ SİHİR YAPILIYOR: Kitap ele ışınlanıyor!')
        
        # Kitap adı: 'target_book' (Gazebo'daki adı neyse onu yaz!)
        # Oryantasyon: 0 0 0 1 (Düz duruş) veya elin açısına göre ayarla.
        cmd = f"""ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "{{state: {{name: 'target_book', pose: {{position: {{x: {self.MAGIC_X}, y: {self.MAGIC_Y}, z: {self.MAGIC_Z}}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}}}" """
        
        os.system(cmd) # Komutu terminale bas

    def run_mission(self):
        # 1. HAZIRLIK
        self.move_torso(0.35); self.move_gripper(0.044); self.move_head(0.0, -0.6)
        self.move_arm(self.SAFE_TRAVEL, 4)
        time.sleep(5)

        # 2. YAKLAŞMA
        self.get_logger().info('🚗 Sahne 1: Robot Yaklaşıyor...')
        self.drive(0.1, 3.5) # 35 cm yaklaş
        time.sleep(1)

        # 3. POZİSYON ALMA (Havadan Gel)
        self.get_logger().info('🦢 Sahne 2: Kuğu Boynu Pozisyonu...')
        self.move_arm(self.SWAN_POSE, 5)
        time.sleep(6)
        
        # 4. İNİŞ (Görsel İniş)
        self.get_logger().info('⬇️ Sahne 3: El İniyor...')
        self.move_torso(0.15, duration=8) # Yavaşça in
        time.sleep(9) 

        # --- BURADA HİLE YAPIYORUZ ---
        # Gripper tam indiğinde, kapanmadan hemen önce:
        self.cheat_teleport() 
        time.sleep(1) # Işınlanmanın oturması için bekle

        # 5. TUT (Demir Kıskaç)
        self.get_logger().info('✊ Sahne 4: TUTUYOR GİBİ YAP (Sıkıştır)...')
        self.move_gripper(-0.01) 
        time.sleep(3)

        # 6. KALDIR (Zafer)
        self.get_logger().info('🏆 Sahne 5: Kaldırma...')
        self.move_torso(0.35, duration=5) 
        time.sleep(6)

        self.get_logger().info('🏁 FİLM BİTTİ! (Kesin tutmuş olması lazım)')
        self.move_arm(self.LIFT_UP, 4)

    # --- YARDIMCI FONKSİYONLAR ---
    def drive(self, speed, duration):
        msg = Twist(); msg.linear.x = float(speed)
        self.vel_pub.publish(msg); time.sleep(duration); self.vel_pub.publish(Twist())

    def move_arm(self, pos, d): 
        self.send_traj(self.arm, ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'], pos, d)
    
    def move_gripper(self, pos): 
        self.send_traj(self.gripper, ['gripper_left_finger_joint', 'gripper_right_finger_joint'], [pos, pos], 2)
    
    def move_torso(self, pos, duration=4): 
        self.send_traj(self.torso, ['torso_lift_joint'], [pos], duration)
    
    def move_head(self, p, t): 
        self.send_traj(self.head, ['head_1_joint', 'head_2_joint'], [p, t], 2)

    def send_traj(self, client, joints, positions, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration)
        goal.trajectory.points = [point]
        client.wait_for_server()
        client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args); node = HollywoodCrane(); rclpy.shutdown()

if __name__ == '__main__':
    main()