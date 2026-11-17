#!/bin/bash
# systemd (robot-bringup.service) から呼ばれて robot_bringup を起動する.
#
# systemd から起動したプロセスは ~/.bashrc を読まないので, ROS の環境 (ros2 コマンド,
# ビルドしたワークスペース, ROS2_TARGET, ROS_DOMAIN_ID) をここで読み込む.
# 設定は /etc/default/robot-bringup (install-autostart.sh が作る) に書く.
set -eo pipefail

CONF=${ROBOT_BRINGUP_CONF:-/etc/default/robot-bringup}
if [[ -f "$CONF" ]]; then
    # shellcheck disable=SC1090
    . "$CONF"
fi
ROS_WS=${ROS_WS:-$HOME/ros2_ws}
ARDUINO_PORT=${ARDUINO_PORT:-/dev/ttyACM0}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "/opt/ros/humble/setup.bash が無い (ROS 2 Humble が入っていない)" >&2
    exit 1
fi
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
    echo "$ROS_WS/install/setup.bash が無い. ROS_WS ($CONF) を確認するか setup-raspi.sh でビルドする" >&2
    exit 1
fi
# ROS の setup.bash は未定義の変数を参照するので set -u は付けない
. /opt/ros/humble/setup.bash
. "$ROS_WS/install/setup.bash"

export ROS2_TARGET=${ROS2_TARGET:-l2}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export PYTHONUNBUFFERED=1           # journalctl にログがすぐ出るように
export RCUTILS_COLORIZED_OUTPUT=0   # journalctl に色のエスケープ文字が混ざらないように

# ROBOT_LAUNCH_ARGS は空白区切りで launch 引数に渡す (例: "dry_run:=true web_token:=abcd")
# shellcheck disable=SC2206
args=(arduino_port:="$ARDUINO_PORT" ${ROBOT_LAUNCH_ARGS:-})
echo "起動: ros2 launch robot_bringup robot.launch.py ${args[*]} (ROS2_TARGET=$ROS2_TARGET, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, ユーザー $(id -un))"
exec ros2 launch robot_bringup robot.launch.py "${args[@]}"
