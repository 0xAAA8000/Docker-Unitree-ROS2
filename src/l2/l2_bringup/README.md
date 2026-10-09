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
- `auto_init:=true`: 起動直後に約3秒静止しているあいだのスキャンから、開始位置を地図上で自動推定する
  (`scripts/l2_initial_pose.py`。IMU で水平化 → 床から 0.5〜2.0m の構造物の上面図を全方位で照合 →
  上位 300 候補を ICP で絞り込み → `/initialpose`)。約2〜4秒。別の場所の候補が僅差なら警告を出す。
  地図から作った模擬スキャンでは 50/50 だったが、**実機では 4 回中 1 回しか当たらなかった** (2026-10-02)。
  必ず RViz で確認すること。`init_map` は非圧縮 PCD (pcl_voxel_grid の出力は圧縮なので不可)。
- `relocalize:=true`(既定): 推定位置でスキャンを地図に重ねた一致率を常に監視し、直近 2 秒の中央値が 0.6 未満
  (または推定位置が出なくなった)なら「位置を見失いました」と表示。センサが 2 秒静止すると、最後に正しかった
  位置の周辺を探し直して `/initialpose` に出し、3 秒間の一致率で確認して「復帰しました」と表示する
  (`scripts/l2_initial_pose.py`)。10/7 の屋外の記録では、見失った区間の一致率が 0.34 → 0.99 に回復した。
- `l2_data_watch.py`(mapping / localization 共通): 点群・IMU が 1 秒以上届かないと警告、戻ると途切れた秒数を表示。
  USB-LAN アダプタが切れると数〜十数秒データが止まり、Point-LIO の地図が崩れたり位置を見失ったりするため。
- `record:=true`: 生データ (`/unilidar/cloud`, `/unilidar/imu`) と推定結果を `~/bags/l2loc_日時` に ros2 bag で記録する
  (`mapping.launch.py` も同様)。
- ドライバは `unilidar_imu -> unilidar_lidar` の TF を常に出していて、`base_link -> unilidar_lidar` と衝突する
  (NDT がほとんど動かなくなる)。`localization.launch.py` ではドライバの `/tf` を `/unilidar/tf` に付け替えている。
- 出力: `/pcl_pose`、`/path`、TF `map -> base_link` (`base_link` = `unilidar_lidar`)。
- パラメータは `config/localization.yaml`。2025 年度の設定からの変更点:
  初期姿勢 `qw` 0 → 1 (0 は回転として不正)、`use_imu` true → false (当時も IMU は未接続で無効だった)、
  スキャン間引き 0.5 → 0.2m・スレッド 6 → 12 (L2 の 1 スキャンの照合は約 1.5ms で、83ms 間隔に対して余裕が大きいため)。

## 記録の評価 (`tools/l2eval`)

記録した bag を自己位置推定に再生し、同じデータを Point-LIO で処理した軌跡を基準に採点する
(位置飛び・ずれ・不採用率。結果は `記録_eval/results.md`)。パラメータや地図を変えて何度でも比べられる。

```bash
tools/l2eval BAG MAP -x X -y Y -yaw DEG -n 名前 [-- -p voxel_leaf_size:=0.5 ...]
python3 tools/l2eval_dump.py BAG   # 実験中の /pcl_pose を live.csv に出し、開始姿勢を表示
```
実機テストの手順は `docs/実機テスト手順_自己位置推定.md`。

## 地図の後処理 (`tools/pcd_*`)

Point-LIO が破綻して点が遠くへ飛んだ部分(ほうき星)の除去、(任意で)ループ補正、外れ点除去(CloudCompare)。
手順は `docs/地図の後処理_手順書.md`。

```bash
python3 tools/pcd_trim.py MAP.pcd -o MAP_trim.pcd
python3 tools/pcd_loopfix.py MAP_trim.pcd -o MAP_loop.pcd   # RESULT が "Do not use" なら使わない
tools/pcd_sor.sh MAP_trim.pcd MAP_clean.pcd
```

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
