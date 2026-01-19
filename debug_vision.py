#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionDebugger(Node):
    def __init__(self):
        super().__init__('vision_debugger')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.callback, 1)
        self.get_logger().info('📸 VISION DEBUGGER STARTED. Point robot at the book!')

    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            
            # Get center pixel color
            h, w, _ = hsv.shape
            center_pixel = hsv[int(h/2), int(w/2)]
            
            # Check current detection mask (Standard Green)
            # Range was: [35, 50, 50] to [85, 255, 255]
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            green_pixels = cv2.countNonZero(mask)

            print(f"Center HSV: {center_pixel} | Green Pixels Seen: {green_pixels}")
            
        except Exception as e:
            pass

def main():
    rclpy.init()
    node = VisionDebugger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()