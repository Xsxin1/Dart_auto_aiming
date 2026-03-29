#! /bin/zsh
echo "start auto start script"

cd /home/engineer/dart_yolo/

source /opt/ros/humble/setup.zsh
source /home/engineer/dart_yolo/install/setup.zsh

ros2 launch rm_vision_bringup dart_bringup.launch.py 