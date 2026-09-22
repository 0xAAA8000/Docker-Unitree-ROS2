GPU_ARGS=()
if docker info --format '{{json  .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    GPU_ARGS+=(--gpus all)
fi

sudo docker run --rm -it \
    "${GPU_ARGS[@]}" \
    -v "/home/okamoto/projects/ros2_ws/gazebo_sim:/ros2_ws/gazebo_sim" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY="$DISPLAY" \
    --device=/dev/dri:/dev/dri \
    --network host \
    docker-unitree-ros2

