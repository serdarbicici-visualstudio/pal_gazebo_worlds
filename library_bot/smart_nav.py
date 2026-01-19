#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import math

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    print("🚀 Hedef: Kitap Masası (-3.15, -0.87)")

    # --- HEDEF HESAPLAMA ---
    # Kitap Pozisyonu: x=-3.15, y=-0.87
    # Robotu kitabın 0.5 metre gerisinde durduruyoruz (Standoff)
    goal_x = -2.60   
    goal_y = 0.5
    goal_yaw = 3.14  # Kitaba doğru bakması için 180 derece (Radyan)
    # -----------------------

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    
    # Yaw -> Quaternion
    goal_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

    print(f"📍 Navigasyon başlatılıyor: ({goal_x}, {goal_y})")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Mesafe: {feedback.distance_remaining:.2f}m', end='\r')
        time.sleep(1.0)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('\n✅ Masaya varıldı. Kol kontrolü için hazır!')
    else:
        print('\n❌ Navigasyon başarısız.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()