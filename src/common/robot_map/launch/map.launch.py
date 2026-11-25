"""2D 地図ノード (robot_map) を起動する. 高さの判定には robot.yaml (obstacle_detector の項) の値を使う.

  ros2 launch robot_map map.launch.py
  ros2 launch robot_map map.launch.py pcd_file:=$HOME/maps/scans.pcd   # 保存済みの地図から始める
  ros2 launch robot_map map.launch.py resolution:=0.1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# robot.yaml から引き継ぐパラメータ (obstacle_detector の項. 古い robot.yaml なら lidar_bridge の項)
_FROM_ROBOT = ('ground_z', 'obstacle_min_height', 'obstacle_max_height')


def _setup(context):
    config = LaunchConfiguration('config').perform(context)
    with open(config) as f:
        doc = yaml.safe_load(f)
    prm = dict((doc.get('lidar_bridge') or {}).get('ros__parameters') or {})
    prm.update((doc.get('obstacle_detector') or {}).get('ros__parameters') or {})
    params = {k: float(prm[k]) for k in _FROM_ROBOT if k in prm}
    params.update({
        'resolution': float(LaunchConfiguration('resolution').perform(context)),
        'pcd_file': LaunchConfiguration('pcd_file').perform(context),
        'live': LaunchConfiguration('live').perform(context).lower() == 'true',
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
    })
    return [Node(
        package='robot_map',
        executable='map',
        name='robot_map',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    robot_yaml = os.path.join(get_package_share_directory('lidar_bridge'), 'config', 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=robot_yaml,
                              description='床・障害物の高さを読む robot.yaml'),
        DeclareLaunchArgument('resolution', default_value='0.05', description='地図の 1 セル [m]'),
        DeclareLaunchArgument('pcd_file', default_value='',
                              description='最初に読み込む保存済み地図 (.pcd). 空なら読み込まない'),
        DeclareLaunchArgument('live', default_value='true',
                              description='Point-LIO の点群で地図を更新し続けるか'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
