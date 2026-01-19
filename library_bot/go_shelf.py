#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class SimpleShelfApproach(Node):
    def __init__(self):
        super().__init__('simple_shelf_approach_node')
        
        # --- AYARLAR ---
        self.target_dist = 1.3  # Hedef Mesafe: 70 cm
        self.tolerance = 0.02    # Tolerans: +/- 2 cm hata payı kabul
        self.kp = 0.8            # Hız kazancı (Ne kadar yüksek, o kadar agresif yaklaşır)
        # ---------------

        # Lidar verisini al
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        # Tekerleklere komut ver
        self.vel_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)
        
        self.front_dist = float('inf') # Başlangıçta mesafe sonsuz

        self.get_logger().info(f'🎯 HEDEF: Rafa {self.target_dist*100:.0f} cm kala durmak.')
        
        # Saniyede 10 kere kontrol döngüsünü çalıştır
        self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        # Robotun tam önündeki dar bir açıyı tara
        mid_idx = len(msg.ranges) // 2
        window = 10 # Merkezden +/- 10 ışın
        
        # Geçerli verileri (sonsuz veya hatalı olmayanları) al
        valid_ranges = [r for r in msg.ranges[mid_idx-window : mid_idx+window] if not np.isinf(r) and not np.isnan(r)]
        
        if len(valid_ranges) > 0:
            self.front_dist = min(valid_ranges) # En yakın noktayı al
        else:
            self.front_dist = float('inf') # Önüm boş

    def control_loop(self):
        if self.front_dist == float('inf'):
            self.get_logger().warn('⚠️ Önümde bir şey görmüyorum, duruyorum.')
            self.stop_robot()
            return

        # Hata = Şu anki mesafe - Hedef mesafe
        error = self.front_dist - self.target_dist
        cmd = Twist()

        if abs(error) <= self.tolerance:
            # Tolerans içindeyiz (Örn: 68cm ile 72cm arası) -> DUR
            cmd.linear.x = 0.0
            self.get_logger().info(f'✅ HEDEFTEYİM! Mesafe: {self.front_dist:.3f}m')
            
        elif error > 0:
            # Uzaktayım (Örn: 1.5m) -> İLERİ GİT
            # P-Kontrol: Hata ne kadar büyükse hız o kadar artar.
            speed = error * self.kp 
            # Hızı sınırla (Çok uçmasın, çok da yavaş kalmasın)
            cmd.linear.x = min(max(speed, 0.05), 0.4) 
            self.get_logger().info(f'➡️ Yaklaşıyorum... (Mesafe: {self.front_dist:.2f}m, Hız: {cmd.linear.x:.2f})')
            
        else:
            # Çok yakınım (Örn: 0.5m) -> GERİ GİT
            speed = error * self.kp # Error negatif olduğu için hız negatif olur
            cmd.linear.x = max(min(speed, -0.05), -0.2) # Geri hız limiti
            self.get_logger().info(f'🔙 Çok girdim, düzeltiyorum... (Mesafe: {self.front_dist:.2f}m)')

        # Komutu gönder
        cmd.angular.z = 0.0 # Sadece düz git
        self.vel_pub.publish(cmd)

    def stop_robot(self):
        self.vel_pub.publish(Twist())

def main():
    rclpy.init()
    node = SimpleShelfApproach()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()