# l2_bringup

Unitree L2 ドライバ + Point-LIO + RViz2 を 1 つの launch でまとめて起動するパッケージ。

```bash
ros2 launch l2_bringup mapping.launch.py                      # Ethernet、地図は保存しない
ros2 launch l2_bringup mapping.launch.py save:=true           # Ctrl+C で終了したときに地図を保存
ros2 launch l2_bringup mapping.launch.py connection:=serial   # USB シリアル (/dev/ttyACM0)
ros2 launch l2_bringup mapping.launch.py rviz:=false
```

起動後、最初の数秒は L2 を動かさない (IMU 初期化)。

| 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `connection` | `ethernet` | `ethernet` または `serial` |
| `save` | `false` | `true` で終了時に `point_lio_ros2/PCD/scans.pcd` に保存 (毎回上書き) |
| `rviz` | `true` | RViz2 を起動する |

## ドライバ単体 (`unitree_lidar_ros2`)

`unilidar_sdk2` の `launch.py` に引数を追加してある。

```bash
ros2 launch unitree_lidar_ros2 launch.py connection:=serial serial_port:=/dev/ttyACM0
```

| 引数 | 既定値 |
| --- | --- |
| `connection` | `ethernet` |
| `serial_port` | `/dev/ttyACM0` |
| `lidar_ip` | `192.168.1.62` |
| `local_ip` | `192.168.1.2` |
| `rviz` | `true` |

## 接続メモ

- L2 は工場出荷時 Ethernet モード。LiDAR `192.168.1.62`、PC 側を `192.168.1.2/24` に固定する。
- ドライバは起動時に `work_mode` を LiDAR に書き込む (ethernet → 0、serial → 8)。
  モード変更は L2 の電源を入れ直すと反映される。
- L2 付属の 12V 1A アダプタでは回転が安定せず点群が出ないことがあった (L1 のアダプタでは正常)。
  点群が来ない／IMU が数秒おきに途切れるときは電源を疑う。
