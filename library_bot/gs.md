# Agalar bu kod çok önemli terminal açınca bunu yapıştırın direk
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1

# Bunla gazeboyu ve robotu çalıştırcaz
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=galatasaray

# Nav2 slam algosu için ayrı terminalde çalışcak
ros2 launch nav2_bringup slam_launch.py use_sim_time:=True

# Lidar sensor bağlantısı için bunu da ayrı terminalde çalıştır
ros2 run topic_tools relay /scan_raw /scan

# Rviz çalıştır
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
# Keyboardla kontrol için 
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Project map saveleme
cd ~/ros2_ws/src/pal_gazebo_worlds/library_bot/
ros2 run nav2_map_server map_saver_cli -f gs_map


# Nava başlıyoruz
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True map:=$HOME/ros2_ws/src/pal_gazebo_worlds/library_bot/gs_map.yaml

### Batuhan trial
In every new terminal you open, you must run this command to tell ROS where your custom code lives:
source ~/ros2_ws/install/setup.bash

ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=galatasaray
ros2 run topic_tools relay /scan_raw /scan
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True map:=/root/ros2_ws/src/pal_gazebo_worlds/library_bot/gs_map.yaml
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

python3 ~/ros2_ws/src/pal_gazebo_worlds/library_bot/book_color_picker.py
python3 ~/ros2_ws/src/pal_gazebo_worlds/library_bot/grip.py