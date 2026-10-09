#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Gazebo Classic を入れている場合は、そのモデル/プラグインのパスも通す
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

# CMD もしくは docker run の引数をそのまま実行する
# 例: docker run ... <image> ros2 launch unitree_lidar_ros2 launch.py
exec "$@"
