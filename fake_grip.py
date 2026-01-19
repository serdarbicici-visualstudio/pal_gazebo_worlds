#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState

class FakeGrip(Node):
    def __init__(self):
        super().__init__('fake_grip_node')
        
        # --- AYARLAR ---
        self.book_name = 'target_book'      # Kitabın adı
        self.hand_frame = 'gripper_link'    # Elin adı
        
        # OFSET (Elin merkezinden ne kadar önde?)
        self.offset_x = 0.16   # 16 cm ileri
        # ----------------
        
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info('🧲 PROFESYONEL MOD: Kitap (target_book) ele kilitlendi. Hareket edebilirsin!')
        
        # 50 Hz (Daha sıkı takip)
        self.timer = self.create_timer(0.02, self.update_loop)

    def update_loop(self):
        req = SetEntityState.Request()
        req.state.name = self.book_name
        
        # SİHİRLİ SATIR: Referansı direkt EL yapıyoruz.
        # Böylece TF hesaplamaya gerek kalmıyor, Gazebo kendi hallediyor.
        req.state.reference_frame = self.hand_frame
        
        # Sadece X ekseninde (ileri) ötele
        req.state.pose.position.x = self.offset_x
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.0
        
        # Oryantasyon 0,0,0,1 demek "Elin duruşuyla AYNI olsun" demek
        req.state.pose.orientation.w = 1.0
        
        self.client.call_async(req)

def main():
    rclpy.init()
    node = FakeGrip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()