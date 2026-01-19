import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- SENİN ÖLÇTÜĞÜN ALTIN KOORDİNATLAR ---
LOCATIONS = {
    "bookshelf_1": {
        "x": -0.060,
        "y": 0.788,
        "yaw": 1.352
    },
    "bookshelf_2": {
        "x": 1.683, 
        "y": 0.756,
        "yaw": -0.228
    },
    "table": {
        "x": -1.118,
        "y": -1.580,
        "yaw": -1.667
    }
}
# ----------------------------------------

def get_quaternion_from_yaw(yaw):
    """
    Radyan cinsinden açıyı (Yaw) Robotun anlayacağı Quaternion formatına çevirir.
    """
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Nav2'nin tamamen hazır olmasını bekle
    print("⏳ Nav2 sistemi bekleniyor...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 HAZIR! Görev başlıyor.")

    # --- GÖREV SIRALAMASI ---
    # 1. Masa -> 2. Raf 1 -> 3. Masa -> 4. Raf 2
    mission_queue = ["table", "bookshelf_1", "table", "bookshelf_2"]

    for target_name in mission_queue:
        target_data = LOCATIONS[target_name]
        
        print(f"\n🚀 HEDEF: {target_name.upper()} ({target_data['x']}, {target_data['y']})")
        
        # 1. Hedef Pozisyonunu Oluştur
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = target_data['x']
        goal_pose.pose.position.y = target_data['y']
        
        # Açıyı ayarla (Yüzünü dönmesi için)
        q = get_quaternion_from_yaw(target_data['yaw'])
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # 2. Robotu Gönder
        navigator.goToPose(goal_pose)

        # 3. İlerleme Takibi (15cm kuralı)
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
                # print(f'Mesafe: {remaining:.2f}m') # Konsolu kirletmesin diye kapattım

                # 15 cm kala durdur ki tam park etsin
                if remaining < 0.15:
                    print(f"🎯 {target_name} noktasına VARILDI! (Kalan: {remaining:.2f}m)")
                    navigator.cancelTask() # Navigasyonu bitir
                    break
            
            time.sleep(0.1)

        # Durunca biraz bekle (İşlem yapıyormuş gibi)
        print(f"⏳ {target_name} noktasında işlem yapılıyor (3 sn)...")
        time.sleep(3.0)

    # --- BİTİŞ ---
    print("\n🏁 TÜM GÖREVLER TAMAMLANDI! Eve dönüyoruz veya bekliyoruz.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()