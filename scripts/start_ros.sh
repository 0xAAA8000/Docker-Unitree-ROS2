#!/bin/bash
# ttyacm0-action.service から起動する. スマホで http://<IP>:8080 を開いてモードを選ぶ.
#
# systemd の停止 (SIGINT / SIGTERM) は launcher が受け取り, 動いているモードを順に止めてから終わる.
# モードの停止に最大 30 秒ほどかかるので, サービスの TimeoutStopSec は 60 秒程度にする.

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS2_TARGET=l1
export ROS_DOMAIN_ID=0
# 設定 (src/common/robot_config/config/robots/<名前>.yaml) を使うロボット. ポートなどはそのファイルに書く
export ROBOT_NAME=rc_car
export PYTHONUNBUFFERED=1

# 全モードの ros2 launch に足す引数 (例 "lidar_port:=/dev/serial/by-id/...") と, スマホの画面の key.
# key を設定すると http://<IP>:8080/?key=<token> でしか操作できない
LAUNCH_ARGS=""
TOKEN=""

# ros2 launch を挟まず実行ファイルを直接起動する (ros2 launch は停止の合図から数秒で強制終了するため).
# 空のパラメータ (-p token:=) は ROS 2 が解釈できず起動に失敗するので, 値があるものだけ渡す
params=()
[[ -n "$LAUNCH_ARGS" ]] && params+=(-p "launch_args:=$LAUNCH_ARGS")
[[ -n "$TOKEN" ]] && params+=(-p "token:=$TOKEN")
exec "$(ros2 pkg prefix robot_launcher)/lib/robot_launcher/launcher" --ros-args "${params[@]}"
