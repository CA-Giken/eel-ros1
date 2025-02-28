#!/bin/bash

# ROS Build
source /opt/ros/noetic/setup.bash
cd /root/catkin_ws
catkin build
echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# For React
cd /root/catkin_ws/src/eel-ros1/eel-react
npm i

# 最後に終了しないコマンドを実行
exec tail -f /dev/null