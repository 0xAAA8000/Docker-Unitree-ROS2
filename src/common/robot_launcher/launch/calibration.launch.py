"""キャリブレーションのモード: LiDAR ドライバと Point-LIO (ZUPT なし) と, キャリブレーションの画面.

  ros2 launch robot_launcher calibration.launch.py

Nav2 と cmd_vel_converter は起動しない (calibrate_drive が Arduino を直接使うため).
ZUPT は切る (calibrate_drive は /cmd_vel を出さないので, 一定の速さで走ると停止と誤判定する).
画面のパラメータ (ポート, ホイールベースと LiDAR の位置の初期値, Arduino のポート) は robot_config から配る.
robot.launch.py の他の引数 (lidar_port:= など) はそのまま渡る.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_config.launch_helpers import config_of, override, robot_argument


def _setup(context):
    cfg = config_of(context)
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={
            'robot': cfg.name,
            'nav': 'false',
            'operation': 'false',
            'map': 'false',
            'web': 'false',
            'zupt': 'false',
        }.items(),
    )
    params = cfg.node('calib_web')
    params['calib_dir'] = os.path.expanduser(params.get('calib_dir', '~/drive_calib'))
    params['token'] = LaunchConfiguration('token').perform(context)
    params['launcher_port'] = int(LaunchConfiguration('launcher_port').perform(context) or 0)
    for key in ('port', 'arduino_port', 'calib_dir'):
        if override(context, key):
            params[key] = int(override(context, key)) if key == 'port' else override(context, key)
    return [robot, Node(
        package='robot_launcher',
        executable='calib_web',
        name='calib_web',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        robot_argument(),
        DeclareLaunchArgument('token', default_value='', description='?key=<token> を必須にする'),
        DeclareLaunchArgument('launcher_port', default_value='0',
                              description='モード選択の画面のポート (0 なら戻るボタンを出さない)'),
        # 以下は設定の一時的な上書き (空なら robot_config の値)
        DeclareLaunchArgument('port', default_value='', description='キャリブレーションの画面のポート'),
        DeclareLaunchArgument('arduino_port', default_value='', description='Arduino のシリアルポート'),
        DeclareLaunchArgument('calib_dir', default_value='', description='計測結果を置くディレクトリ'),
        OpaqueFunction(function=_setup),
    ])
