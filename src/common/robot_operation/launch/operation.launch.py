"""Nav2 の速度指令を Arduino に送る cmd_vel_converter を起動する.

  ros2 launch robot_operation operation.launch.py
  ros2 launch robot_operation operation.launch.py dry_run:=true calibration_file:=$HOME/drive_calib/<日時>/summary.yaml

パラメータは robot_config (common.yaml と robots/<robot>.yaml の cmd_vel_converter) から配る.
キャリブレーションの結果 (summary.yaml) は calibration_file:= で渡し, 設定の仮の値を上書きする.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_config.launch_helpers import config_of, flag, override, robot_argument


def _setup(context):
    params = config_of(context).node('cmd_vel_converter')
    params['calibration_file'] = LaunchConfiguration('calibration_file').perform(context)
    params['dry_run'] = flag(context, 'dry_run')
    if override(context, 'port'):
        params['port'] = override(context, 'port')
    return [Node(
        package='robot_operation',
        executable='cmd_vel_converter',
        name='cmd_vel_converter',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        robot_argument(),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='calibrate_drive が出力した summary.yaml (空ならロボットの設定の仮の値)'),
        DeclareLaunchArgument('port', default_value='',
                              description='Arduino のシリアルポート (空ならロボットの設定の値)'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true なら Arduino に送らずログに出すだけ'),
        OpaqueFunction(function=_setup),
    ])
