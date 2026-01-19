#!/usr/bin/env python3
"""
Vision-based book detection and picking system.
Detects book by color, calculates 3D position, navigates, and grasps.
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


class BookColorPicker(Node):
    def __init__(self, target_color='green'):
        super().__init__('book_color_picker')
        
        self.bridge = CvBridge()
        self.target_color = target_color
        self.book_detected = False
        self.book_3d_position = None
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera intrinsics (will be set from camera_info)
        self.camera_info = None
        
        # Subscribe to camera topics
        self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self.camera_info_callback,
            1)
        
        self.rgb_sub = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.rgb_callback,
            1)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/head_front_camera/depth/image_raw',
            self.depth_callback,
            1)
        
        # Store latest depth image
        self.latest_depth = None
        
        # Publisher for robot rotation
        # Changed to the global command topic
        # Use the direct controller topic to bypass the multiplexer
        # Use the high-priority keyboard input topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/key_vel', 10)
        
        # Color thresholds (HSV)
        self.color_ranges = {
            'green': ([35, 50, 50], [85, 255, 255]),
            'red': ([0, 100, 100], [10, 255, 255]),  # Red wraps around
            'red2': ([170, 100, 100], [180, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255]),
        }
        
        self.get_logger().info(f'🎯 Book Color Picker initialized! Target: {target_color}')

    def camera_info_callback(self, msg):
        """Store camera intrinsics for 3D projection."""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('✓ Camera calibration received')

    def depth_callback(self, msg):
        """Store latest depth image."""
        try:
            # Depth is in 32FC1 format (meters)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth error: {e}')

    def rgb_callback(self, msg):
        """Detect book by color in RGB image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Get color range
            if self.target_color not in self.color_ranges:
                self.get_logger().warn(f'Unknown color: {self.target_color}')
                return
            
            lower, upper = self.color_ranges[self.target_color]
            lower = np.array(lower)
            upper = np.array(upper)
            
            # Create mask
            mask = cv2.inRange(hsv, lower, upper)
            
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
                # Get bounding box
                x, y, w, h = cv2.boundingRect(best_contour)
                cx = int(x + w/2)
                cy = int(y + h/2)
                
                self.get_logger().info(f'📚 Book found at pixel ({cx}, {cy}), area={max_area:.0f}')
                
                # Get 3D position if depth available
                if self.latest_depth is not None and self.camera_info is not None:
                    self.compute_3d_position(cx, cy)
                
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def compute_3d_position(self, pixel_x, pixel_y):
        """Convert pixel + depth to 3D point in camera frame, then transform to map."""
        try:
            # Get depth at pixel
            depth = self.latest_depth[pixel_y, pixel_x]
            
            if np.isnan(depth) or depth <= 0 or depth > 10.0:
                self.get_logger().warn(f'Invalid depth: {depth}')
                return
            
            # Camera intrinsics
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            # Convert pixel to 3D point in camera frame
            # Camera optical frame: +X right, +Y down, +Z forward
            z = float(depth)
            x = (pixel_x - cx) * z / fx
            y = (pixel_y - cy) * z / fy
            
            self.get_logger().info(f'📍 Camera frame: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            
            # Create PoseStamped in camera optical frame
            pose_camera = PoseStamped()
            # Use the actual frame ID received from the Camera Info topic
            pose_camera.header.frame_id = self.camera_info.header.frame_id
            pose_camera.header.stamp = self.get_clock().now().to_msg()
            pose_camera.pose.position.x = x
            pose_camera.pose.position.y = y
            pose_camera.pose.position.z = z
            pose_camera.pose.orientation.w = 1.0
            
            # Transform to map frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    pose_camera.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                
                pose_map = do_transform_pose(pose_camera.pose, transform)
                
                self.book_3d_position = {
                    'x': pose_map.position.x,
                    'y': pose_map.position.y,
                    'z': pose_map.position.z
                }
                
                self.get_logger().info(
                    f'🗺️  Map frame: x={pose_map.position.x:.3f}, '
                    f'y={pose_map.position.y:.3f}, z={pose_map.position.z:.3f}')
                
                self.book_detected = True
                
            except Exception as e:
                self.get_logger().error(f'TF transform failed: {e}')
                
        except Exception as e:
            self.get_logger().error(f'3D computation error: {e}')

    def detect_book(self, timeout=10.0, rotate_scan=True):
        """Spin until book is detected or timeout. Optionally rotate 360° to scan."""
        self.book_detected = False
        self.book_3d_position = None
        
        self.get_logger().info(f'🔍 Searching for {self.target_color} book...')
        
        if rotate_scan:
            self.get_logger().info('🔄 Starting 360° scan...')
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        # Rotation parameters
        angular_speed = 0.3  # rad/s (slow rotation for better detection)
        rotation_duration = 2 * 3.14159 / angular_speed  # Time for 360° at this speed
        
        twist = Twist()
        
        while rclpy.ok() and not self.book_detected:
            # DEBUG PRINT: Verify loop is alive
            print("Loop running... Publishing to /key_vel")  # <--- ADD THIS LINE

            elapsed = time.time() - start_time
            # ... rest of your code ...
            
            # Rotate if enabled and not exceeded rotation time
            if rotate_scan and elapsed < rotation_duration:
                twist.angular.z = angular_speed
                self.cmd_vel_pub.publish(twist)
            elif rotate_scan and elapsed >= rotation_duration:
                # Stop rotation after 360°
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rotate_scan = False  # Don't rotate anymore
                self.get_logger().info('✓ Full 360° scan complete')
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if time.time() - start_time > timeout:
                # Stop robot
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().warn('⏱️  Detection timeout!')
                return False
            
            # rate.sleep()  <-- Comment this out
            time.sleep(0.1) # <-- Use this instead
            print("Loop finished, restarting...") # Optional debug
        
        # Stop rotation when book found
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        if self.book_detected:
            self.get_logger().info('✅ Book detected! Stopped scanning.')
        
        return self.book_detected


def main():
    """Example: detect green book and navigate to it."""
    rclpy.init()
    
    # Create detector node
    picker = BookColorPicker(target_color='green')
    
    # Let camera warm up
    time.sleep(2.0)
    
    # Detect book
    if picker.detect_book(timeout=55.0):
        book_pos = picker.book_3d_position
        picker.get_logger().info(
            f'✅ Book detected at map coordinates: '
            f'x={book_pos["x"]:.3f}, y={book_pos["y"]:.3f}, z={book_pos["z"]:.3f}')
        
        # Calculate approach position (stand 0.5m back from book)
        # Assuming book is on table, approach from -X direction
        approach_x = book_pos['x'] - 0.5
        approach_y = book_pos['y']
        
        picker.get_logger().info(f'🚶 Approach position: x={approach_x:.3f}, y={approach_y:.3f}')
        
        # Navigate using Nav2
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = approach_x
        goal_pose.pose.position.y = approach_y
        goal_pose.pose.position.z = 0.0
        
        # Calculate orientation to face book
        dx = book_pos['x'] - approach_x
        dy = book_pos['y'] - approach_y
        yaw = np.arctan2(dy, dx)
        
        goal_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        picker.get_logger().info(f'🧭 Navigation goal set, yaw={yaw:.3f} rad')
        
        navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            time.sleep(0.2)
        
        result = navigator.getResult()
        if result == 'TaskResult.SUCCEEDED':
            picker.get_logger().info('✅ Navigation succeeded! Now run grip.py')
        else:
            picker.get_logger().warn(f'Navigation result: {result}')
        
    else:
        picker.get_logger().error('❌ Book detection failed')
    
    picker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
