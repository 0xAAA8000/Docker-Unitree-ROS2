# 試験走行の実験手順

## 1. キャリブレーション
### 準備
半径 3 m ほどの空いた平らな場所を用意し、車体は静止させておく。
次の 2 つの値を測っておく。
--wheelbase: ホイールベース [m]（前輪軸から後輪軸まで）
--lidar-x: 後輪軸から LiDAR までの、前方向の距離 [m]
calibrate_drive は Arduino を自分で直接操作します。operation.launch.py（cmd_vel_converter）は起動しないでください。同じシリアルポートを取り合ってしまいます。

```bash
# 端末1: LiDAR ドライバ
ros2 launch unitree_lidar_ros2 launch.py

# 端末2: Point-LIO（L1 用。real.launch.py lidar:=l1 rviz:=false でもよい）
ros2 launch point_lio mapping_unilidar_l1.launch.py rviz:=false

# 端末3: 計測
ros2 run robot_operation calibrate_drive --wheelbase <ホイールベース[m]> --lidar-x <後輪軸からLiDARまで[m]>
```

## 2. n秒間前進して点群を記録
```bash
# 端末1〜3: LiDAR ドライバ、lidar_bridge（Point-LIO を含む）、Arduino への変換
ros2 launch unitree_lidar_ros2 launch.py
ros2 launch lidar_bridge real.launch.py lidar:=l1 gap_fill_timeout:=0.3
ros2 launch robot_operation operation.launch.py calibration_file:=<保存先>/summary.yaml
# 端末4: 0.3 m/s で 5 秒前進して記録
ros2 run robot_operation drive_forward --speed 0.3 --duration 5 --bag /opt/common_ws/src/bags/forward1

```

## 3. RVizから目標を送って自律走行。その間に地図を作る。
```bash
# 1 の端末1〜3 に加えて
ros2 launch robot_navigation navigation.launch.py use_sim_time:=false publish_base_tf:=false cmd_vel_topic:=/cmd_vel
ros2 bag record -o /opt/common_ws/src/bags/nav1 /unilidar/cloud /unilidar/imu /aft_mapped_to_init /tf /tf_static
```