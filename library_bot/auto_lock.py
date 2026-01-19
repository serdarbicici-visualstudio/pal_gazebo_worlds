#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState

class SimpleGlueTuned(Node):
    def __init__(self):
        super().__init__('simple_glue_tuned_node')
        
        self.book_name = 'target_book'
        self.hand_frame = 'tiago::arm_7_link' 
        
        # --- İNCE AYAR BÖLÜMÜ ---
        
        # 1. İLERİ/GERİ AYARI (X)
        # Önceki 0.20 idi. "Uzak olsun" dedin -> 0.23 yaptık.
        # Çok uzaksa azalt, çok yakınsa artır.
        self.offset_x = 0.0
        
        # 2. YUKARI/AŞAĞI AYARI (Z)
        # "Kitap aşağıda" dedin -> Yukarı kaldırmak için 0.05 (5cm) ekledik.
        # Hala aşağıdaysa bunu 0.10 yap. Ters yöne giderse eksi (-) yap.
        self.offset_z = 0.3
        
        # 3. SAĞ/SOL AYARI (Y)
        # Genelde 0.0 iyidir ama kitap sağa/sola kayıksa burayla oyna.
        self.offset_y = 0.0
        # ------------------------

        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info(f'🔧 AYARLAR: X={self.offset_x}, Y={self.offset_y}, Z={self.offset_z}')
        
        self.timer = self.create_timer(0.02, self.glue_loop)

    def glue_loop(self):
        req = SetEntityState.Request()
        req.state.name = self.book_name
        req.state.reference_frame = self.hand_frame
        
        # Ayarları uygula
        req.state.pose.position.x = self.offset_x
        req.state.pose.position.y = self.offset_y
        req.state.pose.position.z = self.offset_z
        
        # Oryantasyon (Kilitli kalsın)
        req.state.pose.orientation.w = 1.0
        
        self.client.call_async(req)

def main():
    rclpy.init()
    node = SimpleGlueTuned()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()