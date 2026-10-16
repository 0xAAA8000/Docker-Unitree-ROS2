"""実機を一括起動する: LiDAR ドライバ -> lidar_bridge + Point-LIO -> Nav2 -> Arduino 変換.

  ros2 launch robot_bringup robot.launch.py                 # 全部 (機種は $ROS2_TARGET, 無ければ l2)
  ros2 launch robot_bringup robot.launch.py lidar:=l1 lidar_port:=/dev/serial/by-id/...
  ros2 launch robot_bringup robot.launch.py nav:=false operation:=false   # 地図作り・確認だけ
  ros2 launch robot_bringup robot.launch.py dry_run:=true   # Arduino に送らずログに出すだけ

起動するもの (それぞれ false にすると起動しない)
  driver    : unitree_lidar_ros2 (L1 はノードを直接, L2 はドライバの launch.py を起動)
  point_lio : lidar_bridge の real.launch.py (ブリッジ, 静的 TF, Point-LIO)
  nav       : robot_navigation の navigation.launch.py (実機用の引数で, nav_delay 秒後に起動)
  operation : robot_operation の operation.launch.py (/cmd_vel -> Arduino)
目的地は別のターミナルから `ros2 run robot_goal goal X Y [YAW_DEG]` で送る.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(pkg, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), 'launch', name))


def _driver(context):
    lidar = LaunchConfiguration('lidar').perform(context)
    enabled = IfCondition(LaunchConfiguration('driver'))
    if lidar == 'l1':
        # L1 の launch.py はポートが固定なので, 同じ設定でノードを直接起動する
        return [Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('lidar_port'),
                'rotate_yaw_bias': 0.0,
                'range_scale': 0.001,
                'range_bias': 0.0,
                'range_max': 50.0,
                'range_min': 0.0,
                'cloud_frame': 'unilidar_lidar',
                'cloud_topic': 'unilidar/cloud',
                'cloud_scan_num': 18,
                'imu_frame': 'unilidar_imu',
                'imu_topic': 'unilidar/imu',
            }],
            condition=enabled,
        )]
    return [IncludeLaunchDescription(
        _launch('unitree_lidar_ros2', 'launch.py'),
        launch_arguments={
            'connection': LaunchConfiguration('l2_connection'),
            'serial_port': LaunchConfiguration('lidar_port'),
            'lidar_ip': LaunchConfiguration('l2_lidar_ip'),
            'local_ip': LaunchConfiguration('l2_local_ip'),
            'rviz': 'false',
        }.items(),
        condition=enabled,
    )]


def generate_launch_description():
    nav_params = os.path.join(get_package_share_directory('robot_navigation'),
                              'config', 'nav2_params.yaml')

    point_lio = IncludeLaunchDescription(
        _launch('lidar_bridge', 'real.launch.py'),
        launch_arguments={
            'lidar': LaunchConfiguration('lidar'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('point_lio')),
    )

    # Point-LIO が TF (map -> base_link) を出し始めてから Nav2 を起動する
    nav = TimerAction(
        period=LaunchConfiguration('nav_delay'),
        actions=[IncludeLaunchDescription(
            _launch('robot_navigation', 'navigation.launch.py'),
            launch_arguments={
                'use_sim_time': 'false',
                'publish_base_tf': 'false',     # aft_mapped -> base_link は real.launch.py が出す
                'cmd_vel_topic': '/cmd_vel',
                'params_file': LaunchConfiguration('nav_params'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('nav')),
        )],
    )

    operation = IncludeLaunchDescription(
        _launch('robot_operation', 'operation.launch.py'),
        launch_arguments={
            'port': LaunchConfiguration('arduino_port'),
            'dry_run': LaunchConfiguration('dry_run'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('operation')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value=os.environ.get('ROS2_TARGET', 'l2'),
                              choices=['l1', 'l2'], description='LiDAR の機種'),
        DeclareLaunchArgument('driver', default_value='true', description='LiDAR ドライバを起動するか'),
        DeclareLaunchArgument('point_lio', default_value='true',
                              description='lidar_bridge + Point-LIO を起動するか'),
        DeclareLaunchArgument('nav', default_value='true', description='Nav2 を起動するか'),
        DeclareLaunchArgument('operation', default_value='true',
                              description='/cmd_vel -> Arduino 変換を起動するか'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz を起動するか (ラズパイでは PC 側で開く)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='LiDAR のシリアルポート (L1, または L2 の serial 接続)'),
        DeclareLaunchArgument('l2_connection', default_value='ethernet',
                              description="L2 の接続方式 'ethernet' or 'serial'"),
        DeclareLaunchArgument('l2_lidar_ip', default_value='192.168.1.62'),
        DeclareLaunchArgument('l2_local_ip', default_value='192.168.1.2'),
        DeclareLaunchArgument('nav_params', default_value=nav_params,
                              description='Nav2 のパラメータファイル'),
        DeclareLaunchArgument('nav_delay', default_value='5.0',
                              description='Point-LIO の起動から Nav2 を起動するまでの秒数'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0',
                              description='Arduino のシリアルポート'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true なら Arduino に送らずログに出すだけ'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='calibrate_drive の summary.yaml (空なら operation.yaml の値)'),
        OpaqueFunction(function=_driver),
        point_lio,
        nav,
        operation,
    ])
