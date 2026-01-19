#!/usr/bin/env python3
"""
Integrated red book detection and MoveIt grasping demo.
Combines vision-based detection with MoveIt arm control.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import time
import sys
import os

# Add library_bot to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_grasper import MoveItGrasper


class RedBookPicker(Node):
    """Vision-based red book detector."""
    
    def __init__(self):
        super().__init__('red_book_picker')
        
        self.bridge = CvBridge()
        self.book_detected = False
        self.book_3d_position = None
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera intrinsics
        self.camera_info = None
        
        # Subscribe to camera topics
        self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self.camera_info_callback, 1)
        
        self.rgb_sub = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.rgb_callback, 1)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/head_front_camera/depth/image_raw',
            self.depth_callback, 1)
        
        self.latest_depth = None
        
        # Publisher for robot rotation
        self.cmd_vel_pub = self.create_publisher(Twist, '/key_vel', 10)
        
        # Red color range (HSV) - two ranges because red wraps around
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])
        
        self.get_logger().info('🔴 Red Book Picker initialized!')

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('✓ Camera calibration received')

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth error: {e}')

    def rgb_callback(self, msg):
        """Detect red book in RGB image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask for red (two ranges)
            mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask = mask1 | mask2
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            best_contour = None
            max_area = 500  # Minimum area threshold
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    best_contour = contour
            
            if best_contour is not None:
                x, y, w, h = cv2.boundingRect(best_contour)
                cx = int(x + w/2)
                cy = int(y + h/2)
                
                self.get_logger().info(f'📕 RED book at pixel ({cx}, {cy}), area={max_area:.0f}')
                
                if self.latest_depth is not None and self.camera_info is not None:
                    self.compute_3d_position(cx, cy)
                    
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def compute_3d_position(self, pixel_x, pixel_y):
        """Convert pixel + depth to 3D point in map frame."""
        try:
            depth = self.latest_depth[pixel_y, pixel_x]
            
            if np.isnan(depth) or depth <= 0 or depth > 10.0:
                return
            
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            z = float(depth)
            x = (pixel_x - cx) * z / fx
            y = (pixel_y - cy) * z / fy
            
            self.get_logger().info(f'📍 Camera frame: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            
            pose_camera = PoseStamped()
            pose_camera.header.frame_id = self.camera_info.header.frame_id
            pose_camera.header.stamp = self.get_clock().now().to_msg()
            pose_camera.pose.position.x = x
            pose_camera.pose.position.y = y
            pose_camera.pose.position.z = z
            pose_camera.pose.orientation.w = 1.0
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', pose_camera.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                
                pose_map = do_transform_pose(pose_camera.pose, transform)
                
                self.book_3d_position = {
                    'x': pose_map.position.x,
                    'y': pose_map.position.y,
                    'z': pose_map.position.z
                }
                
                self.get_logger().info(
                    f'🗺️  Map: x={pose_map.position.x:.3f}, '
                    f'y={pose_map.position.y:.3f}, z={pose_map.position.z:.3f}')
                
                self.book_detected = True
                
            except Exception as e:
                self.get_logger().error(f'TF error: {e}')
                
        except Exception as e:
            self.get_logger().error(f'3D error: {e}')

    def detect_book(self, timeout=30.0, rotate_scan=True):
        """Spin until red book is detected or timeout."""
        self.book_detected = False
        self.book_3d_position = None
        
        self.get_logger().info('🔍 Searching for RED book...')
        
        start_time = time.time()
        angular_speed = 0.3
        rotation_duration = 2 * 3.14159 / angular_speed
        twist = Twist()
        
        while rclpy.ok() and not self.book_detected:
            elapsed = time.time() - start_time
            
            if rotate_scan and elapsed < rotation_duration:
                twist.angular.z = angular_speed
                self.cmd_vel_pub.publish(twist)
            elif rotate_scan and elapsed >= rotation_duration:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rotate_scan = False
                self.get_logger().info('✓ 360° scan complete')
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if time.time() - start_time > timeout:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().warn('⏱️ Detection timeout!')
                return False
            
            time.sleep(0.1)
        
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        if self.book_detected:
            self.get_logger().info('✅ RED book detected!')
        
        return self.book_detected


def main():
    """Full demo: detect red book, navigate, and grasp using MoveIt."""
    rclpy.init()
    
    # Step 1: Detect red book
    picker = RedBookPicker()
    time.sleep(2.0)  # Camera warm-up
    
    if picker.detect_book(timeout=45.0):
        book_pos = picker.book_3d_position
        picker.get_logger().info(
            f'📕 Red book at: x={book_pos["x"]:.3f}, '
            f'y={book_pos["y"]:.3f}, z={book_pos["z"]:.3f}')
        
        # Calculate approach position
        approach_x = book_pos['x'] - 0.5
        approach_y = book_pos['y']
        
        picker.get_logger().info(f'🚶 Approach: x={approach_x:.3f}, y={approach_y:.3f}')
        
        # Step 2: Navigate to book
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = approach_x
        goal_pose.pose.position.y = approach_y
        goal_pose.pose.position.z = 0.0
        
        dx = book_pos['x'] - approach_x
        dy = book_pos['y'] - approach_y
        yaw = np.arctan2(dy, dx)
        goal_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete():
            time.sleep(0.2)
        
        result = navigator.getResult()
        picker.get_logger().info(f'Navigation result: {result}')
        
        # Step 3: Grasp using MoveIt
        picker.destroy_node()
        
        grasper = MoveItGrasper()
        grasper.get_logger().info('🦾 Starting MoveIt grasp...')
        
        # Pick the book (using robot-relative coordinates)
        grasper.pick_object(0.55, book_pos['z'], use_moveit=False)
        grasper.retract()
        grasper.move_to_home()
        
        grasper.get_logger().info('🏁 Red book pick complete!')
        grasper.destroy_node()
        
    else:
        picker.get_logger().error('❌ Red book not found')
        picker.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
