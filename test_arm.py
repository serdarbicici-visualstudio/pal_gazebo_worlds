import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ArmTester(Node):
    def __init__(self):
        super().__init__('arm_tester_node')
        # Robotun kol kontrolcüsüne bağlanıyoruz
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        time.sleep(1) # Bağlantı için bekle

    def move_arm(self):
        msg = JointTrajectory()
        # TIAGo'nun kol eklemlerinin isimleri
        msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        
        point = JointTrajectoryPoint()
        # Kolu yukarı kaldıracak açılar (Radyan cinsinden)
        point.positions = [1.5, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0] 
        point.time_from_start.sec = 3 # 3 saniyede git
        
        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info("Hareket komutu gönderildi! Gazebo'ya bak!")

def main(args=None):
    rclpy.init(args=args)
    tester = ArmTester()
    tester.move_arm()
    # Komutun gitmesi için biraz bekle
    time.sleep(1)
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()