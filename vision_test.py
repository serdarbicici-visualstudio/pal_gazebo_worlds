import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class GreenBookFinder(Node):
    def __init__(self):
        super().__init__('green_finder')
        
        # ROS görüntüsünü OpenCV'ye çeviren köprü
        self.bridge = CvBridge()
        
        # Kameraya abone ol
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10)
        
        # İşlenmiş (Kutucuk çizilmiş) görüntüyü geri yayınla
        self.publisher = self.create_publisher(Image, '/debug/image_processed', 10)
        
        self.get_logger().info('Görüş Sistemi Aktif! Yeşil aranıyor...')

    def image_callback(self, msg):
        try:
            # 1. ROS Mesajını OpenCV resmine çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. Rengi HSV formatına çevir (Renk bulmak için en iyisi)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 3. Gazebo Yeşili için Maske Ayarla
            # Bu aralık Gazebo'daki parlak yeşili yakalar
            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])
            
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # 4. Maskedeki nesnelerin sınırlarını bul (Contours)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                # Sadece belli boyuttan büyük cisimleri al (Gürültüyü engelle)
                if area > 500:
                    # Bounding Box (Sınırlayıcı Kutu) koordinatlarını al
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # 5. Orijinal resme Kırmızı Kutu çiz
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    
                    # 6. Cismin Merkezini Bul
                    cx = int(x + w/2)
                    cy = int(y + h/2)
                    cv2.circle(cv_image, (cx, cy), 5, (255, 0, 0), -1)
                    
                    # Log bas
                    self.get_logger().info(f'Kitap Bulundu! Merkez Piksel: X={cx}, Y={cy}')

            # 7. İşlenmiş resmi ROS topic olarak geri bas (Rviz/Rqt için)
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Hata: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GreenBookFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()