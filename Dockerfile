FROM osrf/ros:humble-desktop-full

WORKDIR /ros2_ws

# hostのros2_ws配下をcontainerにコピー
COPY . .

# コンテナ内でビルド
RUN . /opt/ros/humble/setup.sh && \
colcon build && \
rm -rf build/ log/

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash"]

