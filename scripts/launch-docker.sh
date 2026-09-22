#!/bin/bash
# Gazebo / RViz の GUI をホストの X サーバに表示しつつコンテナを起動する。
set -e

IMAGE="docker-unitree-ros"
TARGET="sim"
DEV_MODE=false

for arg in "$@"; do
    case "$arg" in
        --image=*)
            IMAGE="${arg#*=}"
            ;;
        --target=*)
            TARGET="${arg#*=}"
            ;;
        --dev)
            DEV_MODE=true
            ;;
    esac
done

if [ "$DEV_MODE" = true ]; then
    HOST_SRC_PATH="$(pwd)/src/${TARGET}"
    if [ ! -d "$HOST_SRC_PATH" ]; then
        echo "error: mount target directory is not exist.: ${HOST_SRC_PATH}"
        exit 1
    fi

    CONTAINER_SRC_PATH="/ros2_ws/src"
    MOUNT_OPTION="-v ${HOST_SRC_PATH}:${CONTAINER_SRC_PATH}"
fi

IMAGE="${IMAGE}/${TARGET}:latest"

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
    --device=/dev/dri:/dev/dri \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${MOUNT_OPTION}" \
    "${DEVICE_ARGS[@]}" \
    "$IMAGE" "$@"
