import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        # Gripper genelde bir Action Server üzerinden çalışır
        self._action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

    def send_goal(self, position):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position, position] # İki parmak aynı anda hareket eder
        point.time_from_start.sec = 2
        
        goal_msg.trajectory.points = [point]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Komut reddedildi :(')
            return
        self.get_logger().info('Komut alındı, el hareket ediyor...')

def main(args=None):
    rclpy.init(args=args)
    action_client = GripperTester()
    
    # 1. Önce Eli Kapat (0.0 kapalı, 0.04 açık gibi düşünebilirsin TIAGo için)
    print("El kapatılıyor...")
    action_client.send_goal(0.0) 
    # Biraz bekle (Basit test olduğu için time.sleep kullanıyoruz)
    import time
    time.sleep(3)
    
    # 2. Sonra Eli Aç
    print("El açılıyor...")
    action_client.send_goal(0.044) # Tam açık
    time.sleep(3)

    rclpy.shutdown()

if __name__ == '__main__':
    main()