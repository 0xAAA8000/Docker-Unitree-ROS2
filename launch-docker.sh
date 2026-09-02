sudo docker run -it --rm \
--gpus all \
--network host \
--ipc host \
--device=/dev/ttyUSB0 \
kuritree-ros2:v1
