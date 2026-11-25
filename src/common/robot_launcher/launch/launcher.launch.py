"""モード選択の Web (ポート 8080) を起動する.

  ros2 launch robot_launcher launcher.launch.py token:=abcd launch_args:="dry_run:=true"

ポートなどの既定値は robot_config (nodes.launcher) にあり, launcher ノードが自分で読む.
systemd から起動するときは, ros2 launch を挟まず実行ファイルを直接起動する
(scripts/start_ros.sh, scripts/autostart/robot-bringup.sh). ros2 launch は停止の合図から
数秒で強制終了するため, モードを順に止める時間が足りなくなる.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_config.launch_helpers import robot_argument


def _setup(context):
    params = {'robot': LaunchConfiguration('robot').perform(context)}
    for name in ('token', 'launch_args', 'start_mode', 'arduino_port'):
        value = LaunchConfiguration(name).perform(context)
        if value:   # 空なら launcher が robot_config の値を使う
            params[name] = value
    return [Node(
        package='robot_launcher',
        executable='launcher',
        name='robot_launcher',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        robot_argument(),
        DeclareLaunchArgument('token', default_value=''),
        DeclareLaunchArgument('launch_args', default_value='',
                              description='全モードの ros2 launch に足す引数 (例 "dry_run:=true")'),
        DeclareLaunchArgument('start_mode', default_value='',
                              description='起動時に自動で始めるモード (goal / calibration, 空なら待つ)'),
        DeclareLaunchArgument('arduino_port', default_value='',
                              description='Arduino のシリアルポートの一時的な上書き (空ならロボットの設定の値)'),
        OpaqueFunction(function=_setup),
    ])
