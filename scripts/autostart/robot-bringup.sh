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

# スマホで http://<IP>:8080 を開いてモードを選ぶ (robot_launcher). ROS 2 launch を挟まず直接起動する
# (ros2 launch は停止の合図から数秒で強制終了し, モードを順に止める時間が足りないため).
# ROBOT_LAUNCH_ARGS: 全モードの ros2 launch に足す引数 (空白区切り. 例 "dry_run:=true")
# ROBOT_WEB_TOKEN:  設定すると http://<IP>:8080/?key=<token> でしか操作できない
# ROBOT_START_MODE: 起動時に自動で始めるモード (goal / calibration. 空ならスマホで選ぶのを待つ)
echo "起動: robot_launcher (ROS2_TARGET=$ROS2_TARGET, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, ユーザー $(id -un))"
# 空のパラメータ (-p token:=) は ROS 2 が解釈できず起動に失敗するので, 値があるものだけ渡す
params=(-p "arduino_port:=$ARDUINO_PORT")
[[ -n "${ROBOT_LAUNCH_ARGS:-}" ]] && params+=(-p "launch_args:=$ROBOT_LAUNCH_ARGS")
[[ -n "${ROBOT_WEB_TOKEN:-}" ]] && params+=(-p "token:=$ROBOT_WEB_TOKEN")
[[ -n "${ROBOT_START_MODE:-}" ]] && params+=(-p "start_mode:=$ROBOT_START_MODE")
exec "$(ros2 pkg prefix robot_launcher)/lib/robot_launcher/launcher" --ros-args "${params[@]}"
