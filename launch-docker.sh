#!/bin/bash
# Gazebo / RViz の GUI をホストの X サーバに表示しつつコンテナを起動する。
set -e

IMAGE="${IMAGE:-ghcr.io/0xaaa8000/docker-unitree-ros2:latest}"

# コンテナからホストの X サーバへ接続できるようにする
xhost +local:root >/dev/null 2>&1 || true

DEVICE_ARGS=()
if [ -e /dev/ttyUSB0 ]; then
    DEVICE_ARGS+=(--device=/dev/ttyUSB0)
fi

GPU_ARGS=()
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    GPU_ARGS+=(--gpus all)
fi

docker run -it --rm \
    "${GPU_ARGS[@]}" \
    --network host \
    --ipc host \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${DEVICE_ARGS[@]}" \
    "$IMAGE" "$@"
