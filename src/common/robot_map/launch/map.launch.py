"""2D 地図ノード (robot_map) を起動する. パラメータは robot_config (nodes.robot_map) から配る.
床と障害物の高さ (ground_z, obstacle_min/max_height) は obstacle_detector と同じ値が入る.

  ros2 launch robot_map map.launch.py
  ros2 launch robot_map map.launch.py pcd_file:=$HOME/robot_maps/<日時>_goal.pcd   # 保存済みの地図から始める
  ros2 launch robot_map map.launch.py save_file:=$HOME/robot_maps/test.pcd       # 保存先 (空なら <日時>_map.pcd)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_config.launch_helpers import config_of, flag, robot_argument


def _setup(context):
    params = config_of(context).node('robot_map')
    params['pcd_file'] = LaunchConfiguration('pcd_file').perform(context)
    params['live'] = flag(context, 'live')
    save_file = LaunchConfiguration('save_file').perform(context)
    if save_file:
        params['save_file'] = save_file
    return [Node(
        package='robot_map',
        executable='map',
        name='robot_map',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        robot_argument(),
        DeclareLaunchArgument('pcd_file', default_value='',
                              description='最初に読み込む保存済み地図 (.pcd). 空なら読み込まない'),
        DeclareLaunchArgument('save_file', default_value='',
                              description='ボクセル地図の保存先 (.pcd). 空なら save_dir/<起動日時>_map.pcd'),
        DeclareLaunchArgument('live', default_value='true',
                              description='Point-LIO の点群で地図を更新し続けるか'),
        OpaqueFunction(function=_setup),
    ])
