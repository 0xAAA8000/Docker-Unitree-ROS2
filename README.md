# Unitree LiDAR with ROS2 docker image
**TL;DR:** Unitree L2 LiDARでROS2を動かすためのDockerイメージ
Gazeboシミュレーションと Point-LIO (`point_lio_unilidar`) にも対応しています。

## Usage
```
docker pull ghcr.io/0xaaa8000/docker-unitree-ros2:latest
```

GUI（Gazebo / RViz）を使う場合は同梱の起動スクリプトを使ってください。
`DISPLAY` と `/tmp/.X11-unix` をコンテナへ渡し、NVIDIA runtime と `/dev/ttyUSB0` が
あるときだけそれらを追加します。

```bash
./launch-docker.sh                # bash が起動する
./launch-docker.sh ros2 launch unitree_lidar_ros2 launch.py
```

## 含まれるパッケージ

### ワークスペース (`src/`)

| パッケージ | 内容 |
| --- | --- |
| `unitree_lidar_ros2` | Unitree L1 / L2 LiDAR の ROS 2 ドライバ (`src/unilidar_sdk/`) |
| `point_lio_unilidar` | Point-LIO (LiDAR-IMU オドメトリ)。**本リポジトリで ROS 2 に移植したもの** |

### ベースイメージに追加した apt パッケージ

`osrf/ros:humble-desktop-full` には Ignition Gazebo Fortress (`ros_ign_*`) しか
含まれていないため、Gazebo Classic 11 系を明示的に追加しています。

* Gazebo Classic: `ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-ros2-control`,
  `ros-humble-gazebo-dev`
* ロボットモデル / 制御: `ros-humble-ros2-control`, `ros-humble-ros2-controllers`,
  `ros-humble-controller-manager`, `ros-humble-xacro`,
  `ros-humble-joint-state-publisher(-gui)`, `ros-humble-robot-state-publisher`
* シミュレーション用 3D LiDAR: `ros-humble-velodyne-simulator`,
  `ros-humble-velodyne-gazebo-plugins`, `ros-humble-velodyne-description`
* 操作: `ros-humble-teleop-twist-keyboard`, `ros-humble-rqt-robot-steering`
* Point-LIO のビルド依存: `ros-humble-pcl-conversions`, `ros-humble-pcl-ros`,
  `ros-humble-tf2-ros`, `ros-humble-tf2-eigen`, `libpcl-dev`, `libeigen3-dev`,
  `libomp-dev`

Ignition Fortress 側 (`ros_ign_gazebo` / `ros_ign_bridge`) はベースイメージのまま
使えるので、Classic と Fortress のどちらでも動かせます。

## Point-LIO を動かす

実機 (Unitree L2) の場合:

```bash
# ターミナル1: LiDAR ドライバ
ros2 launch unitree_lidar_ros2 launch.py

# ターミナル2: Point-LIO
ros2 launch point_lio_unilidar mapping_unilidar_l2.launch.py
```

L1 の場合は `mapping_unilidar_l1.launch.py` を使ってください。
`avia` / `horizon` / `ouster64` / `velody16` 用の launch も同梱しています。

### launch 引数

| 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `rviz` | `true` | RViz2 を同時に起動する |
| `use_sim_time` | `false` | bag 再生や Gazebo で回すときは `true` |

### トピック

* 購読: `common.lid_topic` (既定 `/unilidar/cloud`), `common.imu_topic` (既定 `/unilidar/imu`)
* 配信: `/pointlio/odom`, `/pointlio/path`, `/pointlio/cloud_registered`,
  `/pointlio/cloud_registered_body`, `/pointlio/laser_map`
* TF: `camera_init` → `aft_mapped`

パラメータは `src/point_lio_unilidar/config/*.yaml` にあります。ROS 2 のパラメータは
型が厳密なので、`satu_gyro: 35` のような整数リテラルは `35.0` に直してあります。
名前空間の区切りも ROS 1 の `/` から ROS 2 の `.` に変わっています
(例: `mapping/det_range` → `mapping.det_range`)。

## Gazebo で Point-LIO を動かす場合の注意

Point-LIO は「点ごとのタイムスタンプ」を前提にしたアルゴリズムです。

* Gazebo Classic の `gazebo_ros_ray_sensor` が出す `PointCloud2` には
  `ring` も `time` フィールドも入りません。そのままでは Point-LIO に入力できません。
* `velodyne_gazebo_plugins` の `GazeboRosVelodyneLaser` は `ring` と `intensity` を
  出力します。この場合 `preprocess.lidar_type: 2` (Velodyne) を指定してください。
  `time` フィールドが無いときは、Point-LIO 側が方位角から点ごとの時刻オフセットを
  推定するフォールバック経路に入ります (`Preprocess::velodyne_handler`)。
* シミュレーションで回すときは IMU プラグイン (`gazebo_ros_imu_sensor`) を
  `mapping.imu_time_inte` に合わせた周期で回し、launch には
  `use_sim_time:=true` を付けてください。

## Point-LIO の ROS 2 移植について

上流の [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar)
は catkin / roscpp ベースの ROS 1 パッケージで、そのままでは本イメージ (Humble) で
ビルドできません。`src/point_lio_unilidar/` は次の変更を加えた ROS 2 版です。
推定アルゴリズム本体 (`Estimator`, `IKFoM`, `ikd-Tree`, `preprocess` の点群処理) には
手を入れていません。

* ビルドを catkin から `ament_cmake` に変更 (`CMakeLists.txt`, `package.xml`)
* `roscpp` → `rclcpp`。`ros::NodeHandle` を `rclcpp::Node` に、
  publisher / subscriber を `create_publisher` / `create_subscription` に置き換え
* メッセージ型を `sensor_msgs::PointCloud2` から `sensor_msgs::msg::PointCloud2` へ、
  `ConstPtr` を `ConstSharedPtr` へ
* `ros::Time::toSec()` / `fromSec()` が無くなったため、`common_lib.h` に
  `get_time_sec()` / `get_ros_time()` ヘルパを追加
* `nh.param<T>()` を、宣言と取得をまとめた `declare_and_get<T>()` に置き換え
  (ROS 2 はパラメータの事前宣言が必須)。ROS 2 に `float` 型パラメータが無いため
  `plane_thr` と `det_range` は `double` で読んでキャストしています
* `tf::TransformBroadcaster` を `tf2_ros::TransformBroadcaster` +
  `geometry_msgs::msg::TransformStamped` に置き換え
* `ROS_WARN` / `ROS_ERROR` / `ROS_INFO` を `RCLCPP_*` に置き換え
* `.launch` (XML) を `.launch.py` に書き直し、設定ファイルを ROS 2 の
  `/**: ros__parameters:` 形式に変換
* RViz 設定は ROS 1 形式では rviz2 が読めないため、
  `rviz_cfg/point_lio.rviz` を新規に追加（上流の `.rviz` も残してあります）
* 未使用だった `Python.h` / matplotlib-cpp と `eigen_conversions` への依存、および
  デッドコードの `Preprocess::pub_func()` を削除

Livox AVIA の `CustomMsg` 入力経路は上流でもコメントアウトされており、
`livox_ros_driver2` を含めていないため、標準の `sensor_msgs/PointCloud2` 経路のみを
配線しています。

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

* **point_lio_unilidar (Point-LIO)**
  * **License:** [GNU General Public License v2.0](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html) — 上流の `LICENSE` ファイルが GPL-2.0 です
    (上流の `package.xml` には `BSD` と書かれていますが、同梱の `LICENSE` に合わせて
    `GPL-2.0` としています)。**このパッケージを取り込んだことで、配布物全体に GPL-2.0 の
    条件がかかる点に注意してください。**
  * **Copyright:** Copyright (c) Unitree Robotics / Point-LIO contributors
  * **Source:** [unitreerobotics/point_lio_unilidar (GitHub)](https://github.com/unitreerobotics/point_lio_unilidar)

* **Gazebo Classic / gazebo_ros_pkgs**
  * **License:** [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
  * **Copyright:** Copyright Open Source Robotics Foundation, Inc.
  * **Source:** [ros-simulation/gazebo_ros_pkgs (GitHub)](https://github.com/ros-simulation/gazebo_ros_pkgs)
