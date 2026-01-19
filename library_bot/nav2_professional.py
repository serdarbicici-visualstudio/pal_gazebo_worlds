#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()
    
    # Nav2'nin API'ını başlatıyoruz
    navigator = BasicNavigator()

    # --- HEDEF NOKTA (Table position from map) ---
    goal_x = -2.891     # Table x-position in map frame
    goal_y = 2.577      # Table y-position in map frame
    # Yaw: 2.219 rad (127°, facing table) -> Quaternion:
    goal_z = 0.896
    goal_w = 0.445
    # ----------------------------------------

    # Nav2 tamamen açılana kadar bekle
    # (Lifecycle node'ların aktif olması lazım)
    print("⏳ Nav2 bekleniyor (Lütfen RViz'den '2D Pose Estimate' yaptığından emin ol)...")
    navigator.waitUntilNav2Active()

    # Hedef mesajını oluştur
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.z = goal_z
    goal_pose.pose.orientation.w = goal_w

    print(f"📍 Hedef Ayarlandı: X={goal_x}, Y={goal_y}")
    print("🚀 Rota hesaplanıyor ve hareket başlıyor...")
    
    # Git emrini ver
    navigator.goToPose(goal_pose)

    # İlerleme Döngüsü
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"🏃 Hedefe kalan: {feedback.distance_remaining:.2f} metre | Süre: {Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9:.1f}sn", end='\r')

    # Sonuç Kontrolü
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\n✅ BAŞARILI: Robot masaya ulaştı ve park etti!")
    elif result == TaskResult.CANCELED:
        print("\n❌ İPTAL: Hareket iptal edildi.")
    elif result == TaskResult.FAILED:
        print("\n❌ HATA: Navigasyon başarısız! (Yol tıkalı olabilir veya robot sıkıştı)")

    rclpy.shutdown()

if __name__ == '__main__':
    main()