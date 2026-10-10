# ROS2 LiDAR Docker Image
**TL;DR:**  
1. Unitree L1/L2 LiDARでROS2, PointLioを動かすためのDockerイメージ
2. ROS2上でGazebo(Ign)のシミュレーションをするためのDockerイメージ

> ROS 2 のコマンドの使い方は **[ROS 2 コマンド チートシート](docs/ros2-cheatsheet.md)**
> にまとめてあります。`ros2 topic` / `ros2 launch` / `ros2 bag` / `colcon` の使い方と、
> このリポジトリでの定番コマンド、トラブルシュート早見表つきです。

## Usage
### 1. イメージの取得  

**L1**
```bash
docker pull ghcr.io/0xaaa8000/docker-unitree-ros2/l1:latest
```
**l2**
```bash
docker pull ghcr.io/0xaaa8000/docker-unitree-ros2/l2:latest
```
**sim**
```bash
docker pull ghcr.io/0xaaa8000/docker-unitree-ros2/sim:latest
```
### 2. 起動
`--target`でバージョンを選択(l1, l2, sim)  
`--dev`で作業ディレクトリ(src/{l1, l2, sim})をマウント
```bash
./scripts/launch-docker.sh --target=sim --dev
```

## 含まれるパッケージ
### 共通部
| パッケージ | 内容 |
| --- | --- |
| `point_lio` | Point-LIO (LiDAR-IMU オドメトリ)。[dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) を `src/point_lio_ros2/` に取り込んだもの |

### L1, L2用
| パッケージ | 内容 |
| --- | --- |
| `unitree_lidar_ros2` | Unitree L1 / L2 LiDAR の ROS 2 ドライバ (l1:`src/unilidar_sdk/`, l2:`src/unilidar_sdk2`) |

### Gazebo Sim用
| パッケージ | 内容 |
| --- | --- |
| ` ` ||

## Point-LIO を動かす

実機 (Unitree L2) の場合:

```bash
# ターミナル1: LiDAR ドライバ
ros2 launch unitree_lidar_ros2 launch.py

# ターミナル2: Point-LIO
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

L1 の場合は `mapping_unilidar_l1.launch.py` を使ってください。
`avia` / `horizon` / `mid360` / `ouster64` / `velody16` 用の launch と、
オドメトリのみを出す `correct_odom_unilidar_l1(l2).launch.py` も同梱しています。

パッケージ名は `point_lio`、実行ファイルは `pointlio_mapping` です
(ディレクトリ名の `point_lio_ros2` とは異なります)。

### launch 引数

| 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `rviz` | `true` | RViz2 を同時に起動する |
| `use_sim_time` | `false` | bag 再生や Gazebo で回すときは `true` (unilidar 用 launch のみ、下記「取り込みにあたっての変更点」参照) |

### トピック

* 購読: `common.lid_topic` (既定 `/unilidar/cloud`), `common.imu_topic` (既定 `/unilidar/imu`)
* 配信: `/pointlio/odom`, `/pointlio/path`, `/pointlio/cloud_registered`,
  `/pointlio/cloud_registered_body`, `/pointlio/laser_map`
* TF: `camera_init` → `aft_mapped`
  (`odom_header_frame_id` / `odom_child_frame_id` パラメータで変更可能)

パラメータは `src/point_lio_ros2/config/*.yaml` にあります。

動作確認や、うまく動かないときの切り分けは
[ROS 2 コマンド チートシート](docs/ros2-cheatsheet.md#14-トラブルシュート早見表)
を参照してください。


## 実機で Nav2 を動かす (`lidar_bridge` / `robot_operation`)

sim と同じトピック・TF にそろえて、Point-LIO → Nav2 → Arduino の順につなぎます。

```
unitree_lidar_ros2 ──/unilidar/cloud, /unilidar/imu──▶ lidar_bridge ──/lidar, /imu──▶ Point-LIO
                                                                    └─/lidar, /lidar_obstacles──▶ Nav2 ──/cmd_vel──▶ cmd_vel_converter ──serial──▶ Arduino
```

```bash
# 1. LiDAR ドライバ
ros2 launch unitree_lidar_ros2 launch.py
# 2. ブリッジ + 静的 TF + Point-LIO (L1 なら lidar:=l1)
ros2 launch lidar_bridge real.launch.py lidar:=l2
# 3. Nav2 (aft_mapped -> base_link は 2 が出すので publish_base_tf:=false)
ros2 launch robot_navigation navigation.launch.py use_sim_time:=false publish_base_tf:=false cmd_vel_topic:=/cmd_vel
# 4. Twist -> Arduino (最初は dry_run:=true で指令値だけ確認するとよい)
ros2 launch robot_operation operation.launch.py calibration_file:=$HOME/drive_calib/<日時>/summary.yaml
```

* LiDAR の取付位置・傾き (前に 30 度 → `lidar_pitch_deg: 30`)、地面の高さ、自車体の箱は
  `src/common/lidar_bridge/config/robot.yaml` で設定します。障害物の高さ判定は、この取付姿勢で
  base_link 座標に直してから行います (配信する点の座標は LiDAR 座標のまま)。
* `cmd_vel_converter` は Twist を `tan(δ) = L·ω / v` で舵角に、`sin(δ) = K·sin(φ − φ0)` でサーボ角に、
  `throttle = deadband + v / gain` でスロットルに変換します。値は `calibration_file`
  (calibrate_drive の `summary.yaml`) か `src/common/robot_operation/config/operation.yaml` から読みます。
* `/cmd_vel` が 0.5 秒途絶えるとスロットル 0 を送ります。
* LiDAR (L1 は `/dev/ttyUSB0`) と Arduino の USB シリアルが同じ名前にならないよう、
  `port:=/dev/serial/by-id/...` で指定するのが確実です。

### 決めた秒数だけ前進して点群を記録する (`drive_forward`)

上の 1, 2, 4 を起動した状態 (Nav2 は不要) で実行します。`/cmd_vel` に一定速度を送り、
`--bag` を付けると走り出す前から止まった後までの生データと Point-LIO の出力を記録します。

```bash
ros2 run robot_operation drive_forward --speed 0.3 --duration 5 --bag /opt/common_ws/src/bags/forward1
```

* 記録先はコンテナを終了しても消えない、マウントしている場所 (`/opt/common_ws/src` など) にします。
* Ctrl-C で中止すると速度 0 を送ってから終了します。
* Point-LIO が作った地図は、real.launch.py を Ctrl-C で終了したときに
  `src/common/point_lio_ros2/PCD/scans.pcd` に保存されます。

## 実機の走行キャリブレーション (`robot_operation calibrate_drive`)

実機を自動で走らせて、次の2つの対応を Point-LIO のオドメトリから計測します。

* スロットル (operator.ino の -100..100) と速度 [m/s]
* サーボ角 [deg] と前輪の舵角 [deg]

```bash
# ターミナル1, 2: LiDAR ドライバと Point-LIO (車体は静止させておく)
ros2 launch unitree_lidar_ros2 launch.py
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false

# ターミナル3: 計測 (半径 3 m ほどの空きが必要)
ros2 run robot_operation calibrate_drive --wheelbase <ホイールベース[m]> --lidar-x <後輪軸からLiDARまでの前方向距離[m]>
```

* 手順は2段階です。まずステアリングを中央にして各スロットルで直進し、同じだけ後退して戻ります。次に各サーボ角で、一定のスロットルのまま1周（または時間切れ）するまで旋回します。
* LiDAR は前に 30 度傾いていても、X 軸が前を向いていれば問題ありません。機体 X 軸を水平面に投影した向きをヨー角として使います。
* 舵角は自転車モデル `tan(δ) = L / R` で求めます。旋回半径 R は、LiDAR の位置から後輪軸中心の値に換算します (`--lidar-x`)。
* 結果は `~/drive_calib/<日時>/` に保存されます。生データ `raw.csv`, `segments.csv`、区間ごとの結果 `throttle_result.csv`, `steering_result.csv`、フィット結果 `summary.yaml` の5つです。
* `--analyze <ディレクトリ>` を付けると走行せず、記録済みのデータを別の `--wheelbase` / `--lidar-x` で解析し直せます。
* Ctrl-C、またはオドメトリが 0.5 秒途絶えた場合は、スロットル 0・サーボ中央を送って止めます。ただし Arduino 側にはタイムアウトがないので、USB が抜けたときに備えてすぐ止められる状態で使ってください。
* 主なオプション: `--throttles 15,20,30`, `--servos 40,55,70,85,100,115,130`, `--steer-throttle 25`, `--max-distance 3.0`, `--pause` (区間ごとに Enter 待ち)。一覧は `--help` で確認できます。

## Point-LIO のバージョンについて

Unitree 公式の [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar)
は catkin / roscpp ベースの **ROS 1 専用パッケージ**で、そのままでは本イメージ (Humble) では
ビルドできません。

そこで、ROS 2 に移植済みの
[**dfloreaa/point_lio_ros2**](https://github.com/dfloreaa/point_lio_ros2)
を `src/point_lio_ros2/` に取り込んでいます。これは hku-mars の Point-LIO を ROS 2 に
移植したうえで、Unitree Unilidar L1 / L2 対応 (上流 `point_lio_unilidar` 由来) を
取り込んだものです。Livox MID-360 にも対応しています。

* 取り込み元コミット: `a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7` (2025-08-14)
* 更新するときは、上記リポジトリの新しいコミットで `src/point_lio_ros2/` を
  置き換えてください

### 取り込みにあたっての変更点

上流をほぼそのまま入れていますが、次の3点だけ手を入れています。

1. `image/` ディレクトリ (README 用のデモ GIF、約 246 MB) を除外
2. 実体のないサブモジュール定義が残っていた `.gitmodules` を削除
   (`include/IKFoM` と `include/ikd-Tree` は上流でも実ファイルとしてコミット済み)
3. `mapping_unilidar_l1.launch.py` / `mapping_unilidar_l2.launch.py` に
   `use_sim_time` 引数を追加 (Gazebo / bag 再生用。上流には無い)

Livox AVIA の `CustomMsg` 入力経路は上流でもコメントアウトされており
(`livox_ros_driver2` を含めていないため)、標準の `sensor_msgs/PointCloud2` 経路のみが
有効です。

## Licenses & Acknowledgments

This project utilizes and depends on the following open-source software:

* **ROS 2 (desktop-full)**
  * **License:** [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) (Some individual packages within the distribution may use BSD or other permissive licenses)
  * **Copyright:** Copyright Open Source Robotics Foundation, Inc. and ROS 2 contributors
  * **Source:** [ROS 2 Documentation - Governance and Policies](https://docs.ros.org/en/humble/Governance/ROS2-Open-Source-Policy.html)

* **Unitree_Lidar_ROS2**
  * **License:** [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause)
  * **Copyright:** Copyright (c) Unitree Robotics
  * **Source:** [unitreerobotics/unitree_lidar_ros2 (GitHub)](https://github.com/unitreerobotics/unitree_lidar_ros2)

* **Point-LIO / point_lio_ros2**
  * **License:** [GNU General Public License v2.0](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html)
    (`src/point_lio_ros2/LICENSE`)。上流の `package.xml` には `BSD` と書かれていますが、
    同梱の `LICENSE` は GPL-2.0 です。**このパッケージを取り込んだことで、配布物全体に
    GPL-2.0 の条件がかかる点に注意してください。**
  * **Copyright:** Copyright (c) Dongjiao He, Wei Xu (HKU MARS Lab) / Daniel Florea (ROS 2 port & Unilidar support)
  * **Source:** [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO),
    [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2),
    [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar)

* **Gazebo Classic / gazebo_ros_pkgs**
  * **License:** [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
  * **Copyright:** Copyright Open Source Robotics Foundation, Inc.
  * **Source:** [ros-simulation/gazebo_ros_pkgs (GitHub)](https://github.com/ros-simulation/gazebo_ros_pkgs)
