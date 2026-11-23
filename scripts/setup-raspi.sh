#!/bin/bash
# Raspberry Pi (Ubuntu 22.04 / arm64) に Docker なしで直接 ROS 2 Humble と本リポジトリを入れる.
#
#   ./scripts/setup-raspi.sh l1        # L1 用 (src/common + src/l1)
#   ./scripts/setup-raspi.sh l2        # L2 用 (src/common + src/l2)
#
# Dockerfile の base / common-build / l1 / l2 と同じ内容を 1 つのワークスペースにビルドする.
# l1 と l2 は同じ名前のパッケージ (unitree_lidar_ros2) を持つので, どちらか一方だけを選ぶ.
# 何度実行してもよい (2 回目以降は不足分のインストールと再ビルドだけ行う).
#
# 環境変数
#   WS=~/ros2_ws        ビルド先 (build / install / log がここにできる)
#   JOBS=2              同時コンパイル数. メモリ 4GB 以下で Point-LIO がメモリ不足で落ちるなら 1 にする
#   WITH_RVIZ=0         1 ならラズパイにも rviz2 を入れる (通常は PC 側の rviz2 で見る)
#
# ./ で実行しても source (.) で読み込んでもよい. 本体は ( ) のサブシェルで動かすので,
# set -e や exit, cd が呼び出し元のシェルに残らない
(
set -euo pipefail

TARGET=${1:-}
if [[ "$TARGET" != "l1" && "$TARGET" != "l2" ]]; then
    echo "usage: ${BASH_SOURCE[0]} l1|l2" >&2
    exit 1
fi

REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WS=${WS:-$HOME/ros2_ws}
JOBS=${JOBS:-2}
WITH_RVIZ=${WITH_RVIZ:-0}

. /etc/os-release
if [[ "$VERSION_CODENAME" != "jammy" ]]; then
    echo "Ubuntu 22.04 (jammy) 用. この OS は $PRETTY_NAME" >&2
    exit 1
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "警告: aarch64 ではない ($(uname -m)). LiDAR SDK のライブラリは aarch64 / x86_64 のみ" >&2
fi

# ---------- ROS 2 Humble の apt リポジトリ ----------
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    sudo add-apt-repository -y universe
    if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
        ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
              | grep -F '"tag_name"' | awk -F'"' '{print $4}')
        tmp=$(mktemp -d)
        curl -fsSL -o "$tmp/ros2-apt-source.deb" \
            "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"
        sudo dpkg -i "$tmp/ros2-apt-source.deb"
        rm -rf "$tmp"
    fi
fi

# ---------- パッケージ (Dockerfile の base から GUI と Gazebo を除いたもの) ----------
pkgs=(
    ros-humble-ros-base
    ros-humble-pcl-conversions ros-humble-pcl-ros
    ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs
    ros-humble-xacro ros-humble-robot-state-publisher
    ros-humble-navigation2 ros-humble-nav2-bringup
    ros-humble-sensor-msgs-py
    python3-colcon-common-extensions python3-rosdep
    libpcl-dev libeigen3-dev libomp-dev python3-dev
    python3-serial python3-yaml python3-numpy
)
if [[ "$WITH_RVIZ" == "1" ]]; then pkgs+=(ros-humble-rviz2); fi
sudo apt-get update
sudo apt-get install -y --no-install-recommends "${pkgs[@]}"

# シリアル (L1 の /dev/ttyUSB*, Arduino の /dev/ttyACM*) を sudo なしで開けるようにする
if ! id -nG "$USER" | grep -qw dialout; then
    sudo usermod -aG dialout "$USER"
    echo "dialout グループに追加した. 一度ログアウトしてから入り直すこと"
fi

# ---------- ビルド ----------
# ソースはリポジトリに置いたまま --base-paths で指定する. --symlink-install なので
# yaml / launch / Python の変更は再ビルドなしで反映される (C++ は再ビルドが必要)
mkdir -p "$WS"
cd "$WS"
set +u
. /opt/ros/humble/setup.bash
set -u
MAKEFLAGS="-j$JOBS" colcon build \
    --base-paths "$REPO/src/common" "$REPO/src/$TARGET" \
    --symlink-install \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ---------- ~/.bashrc ----------
marker="# >>> docker-unitree-ros2 >>>"
if ! grep -qF "$marker" "$HOME/.bashrc"; then
    cat >> "$HOME/.bashrc" <<EOF

$marker
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export ROS2_TARGET=$TARGET
# PC の rviz2 から見るときは PC 側も同じ値にする
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
# <<< docker-unitree-ros2 <<<
EOF
    echo "~/.bashrc に ROS の設定を追記した"
fi

echo "完了: $WS にビルドした ($TARGET). 新しいターミナルを開くか 'source ~/.bashrc' すること"
)
