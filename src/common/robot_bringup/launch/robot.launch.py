"""実機を一括起動する: LiDAR ドライバ -> lidar_bridge + Point-LIO -> Nav2 -> Arduino 変換.

  ros2 launch robot_bringup robot.launch.py                 # 全部 (ロボットは $ROBOT_NAME, 無ければ rc_car)
  ros2 launch robot_bringup robot.launch.py nav:=false operation:=false   # 地図作り・確認だけ
  ros2 launch robot_bringup robot.launch.py dry_run:=true   # Arduino に送らずログに出すだけ
  ros2 launch robot_bringup robot.launch.py web_token:=abcd # スマホは http://<IP>:8080/?key=abcd

設定 (LiDAR の機種・ポート, Arduino のポート, Web のポートなど) は robot_config から配る
(src/common/robot_config/config/). ポートなどは引数で一時的に上書きもできる (lidar_port:= など).

起動するもの (それぞれ false にすると起動しない)
  driver    : unitree_lidar_ros2 (L1 はノードを直接, L2 はドライバの launch.py を起動)
  point_lio : lidar_bridge の real.launch.py (ブリッジ, 障害物の検知, 静的 TF, Point-LIO)
  nav       : robot_navigation の navigation.launch.py (bringup.nav_delay 秒後に起動)
  operation : robot_operation の operation.launch.py (/cmd_vel -> Arduino)
  map       : robot_map の map.launch.py (Point-LIO の点群 -> 2D 地図 /map)
  web       : robot_goal の web (スマホから地図をタップして目的地を送る)
目的地はスマホの http://<IP>:8080 か, 別のターミナルから `ros2 run robot_goal goal X Y [YAW_DEG]` で送る.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_config.launch_helpers import config_of, flag, override, robot_argument


def _launch(pkg, name):
    """share/<pkg>/launch/<name>. 無ければ share/<pkg>/<name>.

    unitree_lidar_ros2 (L1 / L2) は launch.py を launch/ ではなく share/<pkg>/ の直下に入れる.
    """
    share = get_package_share_directory(pkg)
    path = os.path.join(share, 'launch', name)
    if not os.path.exists(path) and os.path.exists(os.path.join(share, name)):
        path = os.path.join(share, name)
    return PythonLaunchDescriptionSource(path)


def _driver(context):
    if not flag(context, 'driver'):
        return []   # driver:=false ならドライバのパッケージを探さない
    cfg = config_of(context)
    lidar = override(context, 'lidar') or cfg.lidar['model']
    if lidar == 'l1':
        # L1 の launch.py はポートが固定なので, 同じ設定でノードを直接起動する
        params = cfg.node('unitree_lidar_l1')
        if override(context, 'lidar_port'):
            params['port'] = override(context, 'lidar_port')
        return [Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters=[params],
        )]
    args = {k: str(v) for k, v in cfg.node('unitree_lidar_l2').items()}
    if override(context, 'lidar_port'):
        args['serial_port'] = override(context, 'lidar_port')
    args['rviz'] = 'false'
    return [IncludeLaunchDescription(_launch('unitree_lidar_ros2', 'launch.py'),
                                     launch_arguments=args.items())]


def _rest(context):
    cfg = config_of(context)
    robot = cfg.name
    actions = []
    if flag(context, 'point_lio'):
        actions.append(IncludeLaunchDescription(
            _launch('lidar_bridge', 'real.launch.py'),
            launch_arguments={
                'robot': robot,
                'lidar': LaunchConfiguration('lidar').perform(context),
                'rviz': LaunchConfiguration('rviz').perform(context),
                'zupt': LaunchConfiguration('zupt').perform(context),
            }.items(),
        ))
    if flag(context, 'nav'):
        # Point-LIO が TF (map -> base_link) を出し始めてから Nav2 を起動する
        actions.append(TimerAction(
            period=float(cfg.node('bringup').get('nav_delay', 5.0)),
            actions=[IncludeLaunchDescription(
                _launch('robot_navigation', 'navigation.launch.py'),
                launch_arguments={'robot': robot}.items(),
            )],
        ))
    if flag(context, 'operation'):
        actions.append(IncludeLaunchDescription(
            _launch('robot_operation', 'operation.launch.py'),
            launch_arguments={
                'robot': robot,
                'port': LaunchConfiguration('arduino_port').perform(context),
                'dry_run': LaunchConfiguration('dry_run').perform(context),
                'calibration_file': LaunchConfiguration('calibration_file').perform(context),
            }.items(),
        ))
    if flag(context, 'map'):
        actions.append(IncludeLaunchDescription(
            _launch('robot_map', 'map.launch.py'),
            launch_arguments={
                'robot': robot,
                'pcd_file': LaunchConfiguration('map_pcd').perform(context),
            }.items(),
        ))
    if flag(context, 'web'):
        web = cfg.node('robot_goal_web')
        web['token'] = LaunchConfiguration('web_token').perform(context)
        web['launcher_port'] = int(LaunchConfiguration('launcher_port').perform(context) or 0)
        if override(context, 'web_port'):
            web['port'] = int(override(context, 'web_port'))
        actions.append(Node(
            package='robot_goal',
            executable='web',
            name='robot_goal_web',
            output='screen',
            parameters=[web],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        robot_argument(),
        DeclareLaunchArgument('driver', default_value='true', description='LiDAR ドライバを起動するか'),
        DeclareLaunchArgument('point_lio', default_value='true',
                              description='lidar_bridge + 障害物の検知 + Point-LIO を起動するか'),
        DeclareLaunchArgument('nav', default_value='true', description='Nav2 を起動するか'),
        DeclareLaunchArgument('operation', default_value='true',
                              description='/cmd_vel -> Arduino 変換を起動するか'),
        DeclareLaunchArgument('map', default_value='true',
                              description='2D 地図 (/map) を作るか'),
        DeclareLaunchArgument('web', default_value='true',
                              description='スマホから目的地を送る Web サーバーを起動するか'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz を起動するか (ラズパイでは PC 側で開く)'),
        DeclareLaunchArgument('zupt', default_value='true',
                              description='止まっている間は Point-LIO に速度 0 を入れる (手で動かす試験では false)'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true なら Arduino に送らずログに出すだけ'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='calibrate_drive の summary.yaml (空ならロボットの設定の仮の値)'),
        DeclareLaunchArgument('map_pcd', default_value='',
                              description='2D 地図の作成を始める前に読み込む保存済み地図 (.pcd)'),
        DeclareLaunchArgument('web_token', default_value='',
                              description='設定すると URL に ?key=<token> が無いと操作できない'),
        DeclareLaunchArgument('launcher_port', default_value='0',
                              description='モード選択の画面のポート (robot_launcher から起動したとき. 0 なら無し)'),
        # 以下は設定の一時的な上書き (空なら robot_config の値)
        DeclareLaunchArgument('lidar', default_value='', description='LiDAR の機種 l1 / l2'),
        DeclareLaunchArgument('lidar_port', default_value='', description='LiDAR のシリアルポート'),
        DeclareLaunchArgument('arduino_port', default_value='', description='Arduino のシリアルポート'),
        DeclareLaunchArgument('web_port', default_value='', description='目的地の画面のポート'),
        OpaqueFunction(function=_driver),
        OpaqueFunction(function=_rest),
    ])
