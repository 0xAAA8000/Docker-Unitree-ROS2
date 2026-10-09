#!/bin/bash
# WSL (Surface Pro 11 / Snapdragon X) 向けエントリポイント。
# entrypoint.sh の処理に加えて、Adreno GPU (D3D12) 描画を有効にする。
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Gazebo Classic を入れている場合は、そのモデル/プラグインのパスも通す
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

# ホストの /usr/lib/wsl を /usr/lib/wsl-host にマウントしていれば、
# ドライバをパッチして LD_LIBRARY_PATH と GALLIUM_DRIVER を設定する (source が必要)
if [ -f /opt/wsl-gpu/setup.sh ]; then
    source /opt/wsl-gpu/setup.sh
fi

# CMD もしくは docker run の引数をそのまま実行する
exec "$@"
