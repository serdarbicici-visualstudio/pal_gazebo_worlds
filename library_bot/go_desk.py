#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
import sys # Çıkış için gerekli

# Işınlama için gerekli servisler
from gazebo_msgs.srv import SetEntityState

class VisualNavigatorFinal(Node):
    def __init__(self):
        super().__init__('visual_navigator_final_node')
        
        # --- IŞINLAMA (TELEPORT) ---
        self.teleport_robot(x=-1.3, y=0.0, yaw=-1.6)
        # ---------------------------

        self.camera_topic = '/head_front_camera/rgb/image_raw'
        self.head_topic = '/head_controller/joint_trajectory'
        
        # --- AYARLAR ---
        self.stop_area = 1000 
        self.head_step = 0.01
        self.min_head_angle = -0.9 
        # ---------------

        self.sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)
        self.head_pub = self.create_publisher(JointTrajectory, self.head_topic, 10)
        
        self.bridge = CvBridge()
        self.cmd = Twist()
        
        self.lower_green = np.array([40, 100, 50])
        self.upper_green = np.array([80, 255, 255])

        self.current_pitch = 0.0
        self.force_head_move(self.current_pitch)
        
        self.get_logger().info(f'👁️ AYARLANDI: {self.stop_area} alanını geçince DURACAK ve KAPANACAK.')

    def teleport_robot(self, x, y, yaw):
        self.get_logger().info('🌀 IŞINLAMA BAŞLATILIYOR...')
        client = self.create_client(SetEntityState, '/set_entity_state') # Servis adı bazen /gazebo/set_entity_state olabilir
        
        if not client.wait_for_service(timeout_sec=2.0):
            # Servis yoksa da devam et, kodu kilitleme
            self.get_logger().warn('⚠️ Işınlama servisi yok, manuel devam ediliyor.')
            return

        req = SetEntityState.Request()
        req.state.name = 'tiago' 
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.0 
        
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0

        future = client.call_async(req)
        time.sleep(0.5) 
        self.get_logger().info(f'✨ IŞINLANDI: x={x}, y={y}, yaw={yaw}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            height, width, _ = frame.shape
            center_x = width / 2

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                if area > 200:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        error_x = center_x - cx
                        self.control_robot(error_x, area)
                else:
                    self.search_mode()
            else:
                self.search_mode()

        except SystemExit:
            raise # Çıkış sinyalini yukarı ilet
        except Exception as e:
            self.get_logger().error(f'Hata: {e}')

    def force_head_move(self, pitch):
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        p = JointTrajectoryPoint()
        p.positions = [0.0, pitch] 
        p.velocities = [0.0, 0.0]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = 500000000
        traj.points = [p]
        self.head_pub.publish(traj)

    def control_robot(self, error_x, area):
        self.cmd.angular.z = 0.005 * error_x
        
        if area < self.stop_area:
            speed = 0.3 if area < 40000 else 0.1 
            self.cmd.linear.x = speed
            self.get_logger().info(f'🟢 Yaklaşıyorum... (Anlık Alan: {int(area)} / Hedef: {self.stop_area})')
            self.pub.publish(self.cmd)
        else:
            # --- DURMA VE KAPATMA MANTIĞI ---
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.pub.publish(self.cmd) # Önce son kez DUR komutu gönder
            
            self.get_logger().info(f'🛑 HEDEFE ULAŞILDI! (Alan: {int(area)}) -> Kod Kapatılıyor...')
            
            # Bu komut kodu "Başarıyla Bitti" koduyla kapatır.
            # Manager script bunu görünce "Tamam bu bitti, sıradakine geçeyim" der.
            raise SystemExit 

    def search_mode(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.pub.publish(self.cmd)

        if self.current_pitch > self.min_head_angle:
            self.current_pitch -= self.head_step
            self.force_head_move(self.current_pitch)
            self.get_logger().info(f'⚠️ Yeşil yok! Eğiliyorum... ({self.current_pitch:.2f})')
        else:
            self.get_logger().info('❌ Dipteyim (-0.9), hala yok!')

def main():
    rclpy.init()
    node = VisualNavigatorFinal()
    try:
        rclpy.spin(node)
    except SystemExit:
        # Kodun kendi isteğiyle kapanması (Başarılı)
        rclpy.logging.get_logger("Main").info('✅ Görev bitti, çıkış yapıldı.')
    except KeyboardInterrupt:
        # Kullanıcının Ctrl+C yapması
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()