# l2_bringup

Unitree L2 を 1 つの launch でまとめて起動するパッケージ。

- `mapping.launch.py`: ドライバ + Point-LIO + RViz2 (地図作成)
- `localization.launch.py`: ドライバ + lidar_localization_ros2 + RViz2 (保存した地図の上で自己位置推定)

## 地図作成

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

## 自己位置推定 (`localization.launch.py`)

2025 年度に L1 で使っていた方式 (lidar_localization_ros2 の NDT で保存地図に照合) を L2 用にしたもの。
`lidar_localization_ros2` は当時と同じ upstream の版 (cd5e27a)、`ndt_omp_ros2` はその依存。どちらも `src/l2` に同梱。

```bash
# 地図は事前に間引いておく (0.05m)
pcl_voxel_grid scans.pcd map_v0.05.pcd -leaf 0.05,0.05,0.05

ros2 launch l2_bringup localization.launch.py map:=/path/to/map_v0.05.pcd
ros2 launch l2_bringup localization.launch.py map:=... x:=1.5 y:=2.0 yaw:=90   # 初期位置 [m, m, 度]
```

| 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `map` | (必須) | PCD 地図 |
| `x` `y` `z` `yaw` | `0` | 初期位置。地図の原点 = 地図を作り始めた地点・向き |
| `connection` | `ethernet` | `ethernet` または `serial` |
| `rviz` | `true` | RViz2 を起動する |

- 初期位置がずれていたら RViz の **2D Pose Estimate** で指定する (赤いスキャンが白い地図に重なれば OK)。
- 出力: `/pcl_pose`、`/path`、TF `map -> base_link` (`base_link` = `unilidar_lidar`)。
- パラメータは `config/localization.yaml`。2025 年度の設定からの変更点:
  初期姿勢 `qw` 0 → 1 (0 は回転として不正)、`use_imu` true → false (当時も IMU は未接続で無効だった)、
  スキャン間引き 0.5 → 0.2m・スレッド 6 → 12 (L2 の 1 スキャンの照合は約 1.5ms で、83ms 間隔に対して余裕が大きいため)。

## RViz2 の描画 GPU

NVIDIA ドライバ (`libGLX_nvidia.so.0`) がある環境では、RViz2 を NVIDIA GPU で描画する
(`__NV_PRIME_RENDER_OFFLOAD=1`)。ノート PC の PRIME on-demand では指定しないと内蔵 GPU で描画されるため。
ドライバがない環境では何もしない。

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
