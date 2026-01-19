#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=galatasaray &
PID=$! # Simülasyonun işlem numarasını al
# 4. Scriptin kapanmasını engelle
wait $PID