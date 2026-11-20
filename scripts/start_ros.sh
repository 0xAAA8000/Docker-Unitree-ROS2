#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS2_TARGET=l1
export ROS_DOMAIN_ID=0

exec ros2 launch robot_bringup robot.launch.py
