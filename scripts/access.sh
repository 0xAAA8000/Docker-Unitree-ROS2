#!/bin/bash
# 起動済みのコンテナにアクセス

docker exec -it $(docker ps -q -f "ancestor=ghcr.io/0xaaa8000/docker-unitree-ros2") bash -l
