#!/bin/bash
# Arduino (/dev/ttyACM0) が刺さったら robot_bringup を自動で起動するように設定する.
#
#   ./scripts/install-autostart.sh l2          # 設定する (L1 なら l1)
#   ./scripts/install-autostart.sh uninstall   # 元に戻す
#
# 作るもの
#   /etc/udev/rules.d/99-robot-bringup.rules     ttyACM0 が現れたら robot-bringup.service を起動
#   /etc/systemd/system/robot-bringup.service    ttyACM0 が抜けたら止まる. 実行ユーザーはこのスクリプトを実行した人
#   /etc/default/robot-bringup                   ワークスペースや launch 引数の設定 (既にあれば上書きしない)
#
# 環境変数: WS=~/ros2_ws (setup-raspi.sh と同じビルド先), PORT=/dev/ttyACM0
set -euo pipefail

REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SERVICE=robot-bringup.service
RULE=/etc/udev/rules.d/99-robot-bringup.rules
UNIT=/etc/systemd/system/$SERVICE
CONF=/etc/default/robot-bringup

if [[ "${1:-}" == "uninstall" ]]; then
    sudo systemctl stop "$SERVICE" 2>/dev/null || true
    sudo rm -f "$RULE" "$UNIT"
    sudo systemctl daemon-reload
    sudo udevadm control --reload-rules
    echo "削除した ($CONF は残している. 不要なら sudo rm $CONF)"
    exit 0
fi

TARGET=${1:-}
if [[ "$TARGET" != "l1" && "$TARGET" != "l2" ]]; then
    echo "usage: ${BASH_SOURCE[0]} l1|l2|uninstall" >&2
    exit 1
fi
if [[ $EUID -eq 0 ]]; then
    echo "sudo を付けずに, ロボットを動かすユーザーで実行する (必要なところだけ sudo する)" >&2
    exit 1
fi
WS=${WS:-$HOME/ros2_ws}
PORT=${PORT:-/dev/ttyACM0}
KERNEL=$(basename "$PORT")
DEV_UNIT=$(systemd-escape --path --suffix=device "$PORT")
if [[ ! -f "$WS/install/setup.bash" ]]; then
    echo "警告: $WS/install/setup.bash が無い. 先に ./scripts/setup-raspi.sh $TARGET でビルドする" >&2
fi
if ! id -nG "$USER" | grep -qw dialout; then
    echo "警告: $USER が dialout グループに入っていない. シリアルを開けない (setup-raspi.sh が追加する)" >&2
fi

# ---------- 設定ファイル (既にあれば残す) ----------
if [[ ! -f "$CONF" ]]; then
    sudo tee "$CONF" >/dev/null <<CONFEOF
# robot-bringup.service の設定 (scripts/autostart/robot-bringup.sh が読む). 変更後は
#   sudo systemctl restart robot-bringup   (または Arduino を挿し直す)
ROS_WS=$WS
ROS2_TARGET=$TARGET
ROS_DOMAIN_ID=0
ARDUINO_PORT=$PORT
# robot.launch.py に渡す引数 (空白区切り). 例:
#   ROBOT_LAUNCH_ARGS="calibration_file:=$HOME/drive_calib/<日時>/summary.yaml web_token:=abcd"
ROBOT_LAUNCH_ARGS=""
CONFEOF
    echo "作成: $CONF"
else
    echo "既存の $CONF をそのまま使う"
fi

# ---------- systemd ----------
sudo tee "$UNIT" >/dev/null <<UNITEOF
# scripts/install-autostart.sh が作成. $PORT が現れると udev から起動され, 抜けると止まる
[Unit]
Description=robot_bringup ($PORT の Arduino が刺さったら起動)
BindsTo=$DEV_UNIT
After=$DEV_UNIT network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
SupplementaryGroups=dialout
WorkingDirectory=$HOME
ExecStart=$REPO/scripts/autostart/robot-bringup.sh
# 止めるときは ros2 launch にだけ Ctrl-C (SIGINT) を送り, launch が各ノードを順に止める.
# 全プロセスに同時に送ると, 走行中のゴールの取り消しや地図の保存の途中で割り込まれる
KillMode=mixed
KillSignal=SIGINT
TimeoutStopSec=30
# 起動直後に失敗したら (LiDAR やネットワークがまだ準備できていない等) 少し待ってやり直す
Restart=on-failure
RestartSec=10
UNITEOF
echo "作成: $UNIT"

# ---------- udev ----------
sudo tee "$RULE" >/dev/null <<RULEEOF
# scripts/install-autostart.sh が作成. $PORT が現れたら $SERVICE を起動する
SUBSYSTEM=="tty", KERNEL=="$KERNEL", TAG+="systemd", ENV{SYSTEMD_WANTS}+="$SERVICE"
RULEEOF
echo "作成: $RULE"

sudo systemctl daemon-reload
sudo udevadm control --reload-rules
if [[ -e "$PORT" ]]; then
    # 既に刺さっているときは, 今の分にもルールを当てて起動する
    sudo udevadm trigger --action=add "$PORT"
    echo "$PORT は刺さっているので起動した"
fi
cat <<MSG

設定した. ログ:   journalctl -u $SERVICE -f
          状態:   systemctl status $SERVICE
          止める: sudo systemctl stop $SERVICE  (Arduino を抜いても止まる)
MSG
