"""実機 (Unitree L1/L2) 用: ドライバの出力を sim と同じトピック・TF にそろえて Point-LIO を起動する.

起動するもの
  * lidar_bridge     : /unilidar/cloud, /unilidar/imu -> /lidar, /lidar_full, /imu
  * obstacle_detector: /lidar_full -> /lidar_obstacles, /lidar_clear (Nav2 の障害物とクリア用.
                       obstacles:=false で起動しない)
  * 静的 TF          : map -> camera_init, aft_mapped -> base_link,
                       base_link -> lidar_link, base_link -> imu_link
  * Point-LIO        : /lidar, /imu を購読 (point_lio:=false で起動しない)
  * RViz             : Point-LIO の結果を表示 (rviz:=false で起動しない)
  * stationary_detector: /cmd_vel と /imu から停止を判定し, Point-LIO の ZUPT に渡す (zupt:=false で無効)

設定はすべて robot_config (common.yaml と robots/<robot>.yaml) から配る. 実際の値は
  ros2 run robot_config show lidar_bridge obstacle_detector point_lio
LiDAR ドライバ (unitree_lidar_ros2) は別に起動しておくこと.
Nav2 は robot_navigation の navigation.launch.py で起動する (aft_mapped -> base_link はこの launch が出す).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from robot_config.launch_helpers import config_of, flag, override, robot_argument


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
    cfg = config_of(context)
    lidar = override(context, 'lidar') or cfg.lidar['model']
    bridge = cfg.node('lidar_bridge')

    # base_link -> LiDAR (= Point-LIO の aft_mapped と同じ位置・姿勢とみなす) と, その逆
    t, rpy = cfg.mount_tf()
    t_inv, rpy_inv = cfg.inverse_mount_tf()
    lidar_frame = bridge.get('lidar_frame', 'lidar_link')
    imu_frame = bridge.get('imu_frame', 'imu_link')
    actions = [
        Node(
            package='lidar_bridge',
            executable='lidar_bridge',
            name='lidar_bridge',
            output='screen',
            parameters=[bridge],
        ),
        _static_tf('map_to_camera_init', 'map', 'camera_init', (0, 0, 0), (0, 0, 0)),
        _static_tf('aft_mapped_to_base_link', 'aft_mapped', 'base_link', t_inv, rpy_inv),
        _static_tf('base_link_to_lidar', 'base_link', lidar_frame, t, rpy),
        _static_tf('base_link_to_imu', 'base_link', imu_frame, t, rpy),
    ]
    if flag(context, 'obstacles'):
        actions.append(Node(
            package='obstacle_detector',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[cfg.node('obstacle_detector')],
        ))

    zupt = flag(context, 'zupt')
    if zupt:
        actions.append(Node(
            package='lidar_bridge',
            executable='stationary_detector',
            name='stationary_detector',
            output='screen',
            parameters=[cfg.node('stationary_detector')],
        ))

    if flag(context, 'point_lio'):
        # 機種ごとの yaml (point_lio_ros2/config/unilidar_<l1|l2>.yaml) に robot_config の値を重ねる
        point_lio = cfg.node('point_lio')
        point_lio.setdefault('zupt', {})['enable'] = zupt
        actions.append(Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('point_lio'), 'config',
                             f'unilidar_{lidar}.yaml'),
                point_lio,
            ],
        ))
    if flag(context, 'rviz'):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(get_package_share_directory('point_lio'),
                                          'rviz_cfg', 'loam_livox.rviz')],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        robot_argument(),
        DeclareLaunchArgument('lidar', default_value='',
                              description='LiDAR の機種 l1 / l2 (空ならロボットの設定の値)'),
        DeclareLaunchArgument('point_lio', default_value='true',
                              description='Point-LIO を起動するか'),
        DeclareLaunchArgument('obstacles', default_value='true',
                              description='Nav2 の障害物を作る obstacle_detector を起動するか'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz を起動するか'),
        DeclareLaunchArgument('zupt', default_value='true',
                              description='止まっている間は Point-LIO に速度 0 を入れる (手で持って動かす試験では false)'),
        OpaqueFunction(function=_setup),
    ])
