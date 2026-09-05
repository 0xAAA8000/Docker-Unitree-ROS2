FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# humble-desktop-full には Ignition Gazebo Fortress (ros_ign_*) しか入っていないので、
# Gazebo Classic 11 一式と ros2_control、および Point-LIO のビルド依存を追加する。
RUN apt-get update && apt-get install -y --no-install-recommends \
    # --- Gazebo Classic 11 + ROS 2 連携 ---
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-dev \
    # --- ロボットモデル / 制御 ---
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    # --- 3D LiDAR をシミュレートするための Gazebo プラグイン ---
    #     ring / intensity 付きの PointCloud2 を出せるので Point-LIO に入力できる
    ros-humble-velodyne-simulator \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-velodyne-description \
    # --- 操作系 ---
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt-robot-steering \
    # --- Point-LIO (point_lio_unilidar) のビルド依存 ---
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    libpcl-dev \
    libeigen3-dev \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# hostのros2_ws配下をcontainerにコピー
COPY . .
RUN chmod +x /ros2_ws/entrypoint.sh

# コンテナ内でビルド
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build/ log/

# Gazebo Classic の標準モデルを見つけられるようにする
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["bash"]
