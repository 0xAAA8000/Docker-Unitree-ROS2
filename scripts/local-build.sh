docker build -f Dockerfile.local --target sim \
    --build-context wsl-gpu=https://github.com/0xAAA8000/WSL-Qualcomm-GPU.git \
    -t ghcr.io/0xaaa8000/docker-unitree-ros2/sim:surface11 .
