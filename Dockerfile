# ========== base：全イメージ共通 ==========
FROM osrf/ros:humble-desktop-full AS base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-tf2-ros ros-humble-tf2-eigen \
    ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    libpcl-dev libeigen3-dev libomp-dev python3-dev \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ========== 共通ワークスペースを1回だけビルド ==========
FROM base AS common-build
COPY src/common /opt/common_ws/src
RUN . /opt/ros/humble/setup.sh && \
    cd /opt/common_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

# ========== l1 ==========
FROM base AS l1
ENV ROS2_TARGET=l1
COPY --from=common-build /opt/common_ws /opt/common_ws
COPY src/l1 /ros2_ws/src
RUN . /opt/ros/humble/setup.sh && . /opt/common_ws/install/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && rm -rf build log

# ========== l2 ==========
FROM base AS l2
ENV ROS2_TARGET=l2
COPY --from=common-build /opt/common_ws /opt/common_ws
COPY src/l2 /ros2_ws/src
RUN ... (l1 と同じ)

# ========== sim ==========
FROM base AS sim
ENV ROS2_TARGET=sim
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz ros-humble-sdformat-urdf ros-humble-ign-ros2-control \
 && rm -rf /var/lib/apt/lists/*
COPY --from=common-build /opt/common_ws /opt/common_ws
COPY src/sim /ros2_ws/src
RUN ... (同上)