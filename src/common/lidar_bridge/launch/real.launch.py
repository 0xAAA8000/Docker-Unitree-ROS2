"""実機 (Unitree L1/L2) 用: ドライバの出力を sim と同じトピック・TF にそろえて Point-LIO を起動する.

起動するもの
  * lidar_bridge     : /unilidar/cloud, /unilidar/imu -> /lidar, /lidar_obstacles, /imu
  * 静的 TF          : map -> camera_init, aft_mapped -> base_link,
                       base_link -> lidar_link, base_link -> imu_link
  * Point-LIO        : /lidar, /imu を購読 (point_lio:=false で起動しない)
  * RViz             : Point-LIO の結果を表示 (rviz:=false で起動しない)

LiDAR ドライバ (unitree_lidar_ros2) は別に起動しておくこと.
Nav2 は robot_navigation の navigation.launch.py を
  use_sim_time:=false publish_base_tf:=false cmd_vel_topic:=/cmd_vel
で起動する (aft_mapped -> base_link はこの launch が出すため).
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np
import yaml


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rpy(r):
    """回転行列 -> (roll, pitch, yaw)."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def _static_tf(name, parent, child, xyz, rpy):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', f'{xyz[0]:.6f}', '--y', f'{xyz[1]:.6f}', '--z', f'{xyz[2]:.6f}',
            '--roll', f'{rpy[0]:.6f}', '--pitch', f'{rpy[1]:.6f}', '--yaw', f'{rpy[2]:.6f}',
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def _setup(context):
    config = LaunchConfiguration('config').perform(context)
    lidar = LaunchConfiguration('lidar').perform(context)
    with open(config) as f:
        prm = yaml.safe_load(f)['lidar_bridge']['ros__parameters']

    # base_link -> LiDAR (= Point-LIO の aft_mapped と同じ位置・姿勢とみなす)
    t = np.array([prm['lidar_x'], prm['lidar_y'], prm['lidar_z']], dtype=float)
    rpy = [math.radians(prm[f'lidar_{k}_deg']) for k in ('roll', 'pitch', 'yaw')]
    r = _rotation(*rpy)
    # 逆変換 aft_mapped -> base_link
    r_inv = r.T
    t_inv = -r_inv @ t

    lidar_frame = prm.get('lidar_frame', 'lidar_link')
    imu_frame = prm.get('imu_frame', 'imu_link')
    actions = [
        Node(
            package='lidar_bridge',
            executable='lidar_bridge',
            name='lidar_bridge',
            output='screen',
            parameters=[config],
        ),
        _static_tf('map_to_camera_init', 'map', 'camera_init', (0, 0, 0), (0, 0, 0)),
        _static_tf('aft_mapped_to_base_link', 'aft_mapped', 'base_link', t_inv, _rpy(r_inv)),
        _static_tf('base_link_to_lidar', 'base_link', lidar_frame, t, rpy),
        _static_tf('base_link_to_imu', 'base_link', imu_frame, t, rpy),
    ]

    # Point-LIO: 機種ごとの yaml を使い, 入力トピックだけ /lidar, /imu に差し替える
    point_lio_params = [
        os.path.join(get_package_share_directory('point_lio'), 'config',
                     f'unilidar_{lidar}.yaml'),
        {
            'common.lid_topic': prm.get('cloud_out', '/lidar'),
            'common.imu_topic': prm.get('imu_out', '/imu'),
            'use_sim_time': False,
            # 以下は mapping_unilidar_l1/l2.launch.py と同じ値
            'use_imu_as_input': False,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            # mapping_unilidar_l1/l2.launch.py は 10. 少ない点で最初の地図を作ると
            # 静止中もドリフトしやすいので増やす (hku-mars/Point-LIO issue #86)
            'init_map_size': int(LaunchConfiguration('init_map_size').perform(context)),
            'point_filter_num': 1,
            'space_down_sample': True,
            'filter_size_surf': 0.1,
            'filter_size_map': 0.1,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
            # 実機は LiDAR の回転で車体ごと振動し, その加速度ノイズを積分して静止中も
            # 動いていると推定してしまう. IMU の加速度を信用しすぎないようにする
            # (yaml の値はそれぞれ 0.1, 500.0)
            'mapping.imu_meas_acc_cov': float(LaunchConfiguration('imu_acc_cov').perform(context)),
            'mapping.acc_cov_output': float(LaunchConfiguration('acc_cov_output').perform(context)),
        },
    ]
    actions.append(Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=point_lio_params,
        condition=IfCondition(LaunchConfiguration('point_lio')),
    ))
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('point_lio'),
                                      'rviz_cfg', 'loam_livox.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('lidar_bridge')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='l2', choices=['l1', 'l2'],
                              description='LiDAR の機種 (Point-LIO の設定ファイルの選択)'),
        DeclareLaunchArgument('config', default_value=os.path.join(pkg, 'config', 'robot.yaml'),
                              description='取付位置・フィルタの設定'),
        DeclareLaunchArgument('point_lio', default_value='true',
                              description='Point-LIO を起動するか'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz を起動するか'),
        DeclareLaunchArgument('imu_acc_cov', default_value='2.0',
                              description='Point-LIO の IMU 加速度の観測分散 (大きいほど IMU を信用しない)'),
        DeclareLaunchArgument('acc_cov_output', default_value='10.0',
                              description='Point-LIO の加速度のプロセスノイズ (小さいほど加速度の推定が滑らか)'),
        DeclareLaunchArgument('init_map_size', default_value='1000',
                              description='Point-LIO が最初の地図を作るのに使う点の数の下限'),
        OpaqueFunction(function=_setup),
    ])
