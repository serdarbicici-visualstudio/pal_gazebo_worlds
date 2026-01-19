import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class BookLocator3D(Node):
    def __init__(self):
        super().__init__('book_locator_3d')
        
        self.bridge = CvBridge()
        
        # Son derinlik resmini hafızada tutacağız
        self.current_depth_image = None
        
        # 1. Derinlik Kamerasına Abone Ol
        self.depth_sub = self.create_subscription(
            Image, 
            '/head_front_camera/depth/image_raw', 
            self.depth_callback, 
            10)

        # 2. Renk Kamerasına Abone Ol
        self.rgb_sub = self.create_subscription(
            Image, 
            '/head_front_camera/rgb/image_raw', 
            self.rgb_callback, 
            10)
            
        self.get_logger().info('3D Konumlandırıcı Başladı! Kitap aranıyor...')

    def depth_callback(self, msg):
        # Derinlik resmini kaydet (Daha sonra RGB ile eşleştireceğiz)
        # 32FC1 formatı = Metre cinsinden Float değerler
        self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def rgb_callback(self, msg):
        if self.current_depth_image is None:
            return # Derinlik verisi gelmeden işlem yapma

        try:
            # --- ADIM 1: YEŞİL KİTABI BUL (Önceki kodun aynısı) ---
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                if cv2.contourArea(contour) > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Merkez Noktası (Piksel)
                    cx = int(x + w/2)
                    cy = int(y + h/2)
                    
                    # Görselleştirme için çiz
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.circle(cv_image, (cx, cy), 5, (255, 0, 0), -1)

                    # --- ADIM 2: DERİNLİK DEĞERİNİ OKU ---
                    # Derinlik haritasında aynı pikselin (cy, cx) değerine bak
                    # Not: OpenCV'de resimler [satır, sütun] yani [y, x] diye okunur!
                    depth_val = self.current_depth_image[cy, cx]
                    
                    if np.isnan(depth_val) or depth_val <= 0:
                        continue # Hatalı okuma varsa atla

                    # --- ADIM 3: 3D KOORDİNAT HESAPLA (Basit Kamera Modeli) ---
                    # TIAGo kamera sabitleri (Yaklaşık değerler)
                    fx = 520.0 # Odak uzaklığı X
                    fy = 520.0 # Odak uzaklığı Y
                    cx_cam = 320.0 # Resim merkezi X (640/2)
                    cy_cam = 240.0 # Resim merkezi Y (480/2)

                    # Z = Derinlik (Metre)
                    Z = depth_val
                    # X = (Piksel - Merkez) * Z / Odak
                    X = (cx - cx_cam) * Z / fx
                    # Y = (Piksel - Merkez) * Z / fy
                    Y = (cy - cy_cam) * Z / fy

                    self.get_logger().info(
                        f'✅ KİTAP BULUNDU!\n'
                        f'   Piksel: ({cx}, {cy})\n'
                        f'   Mesafe: {Z:.2f} metre\n'
                        f'   3D Konum (Kameraya Göre): X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}'
                    )
                    
                    # Tek bir kitap bulmak yeterli, döngüden çık
                    break
            
            # (İsteğe bağlı) Görüntüyü ekranda göster
            cv2.imshow("Kamera", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Hata: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BookLocator3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()