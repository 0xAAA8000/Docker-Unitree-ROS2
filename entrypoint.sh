#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/ins/setup.bash

ros2 launch unitree_lidar_ros2 launch.py

