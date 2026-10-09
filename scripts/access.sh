#!/bin/bash
# 起動済みのコンテナにアクセス

docker exec -it $(docker ps -q -f "ancestor=ghcr.io/0xaaa8000/docker-unitree-ros2/l1:surface11") \
  bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash"
