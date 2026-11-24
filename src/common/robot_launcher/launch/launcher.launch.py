"""モード選択の Web (ポート 8080) を起動する.

  ros2 launch robot_launcher launcher.launch.py token:=abcd launch_args:="lidar:=l1"

systemd から起動するときは, ros2 launch を挟まず実行ファイルを直接起動する
(scripts/start_ros.sh, scripts/autostart/robot-bringup.sh). ros2 launch は停止の合図から
数秒で強制終了するため, モードを順に止める時間が足りなくなる.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('mode_port', default_value='8081'),
        DeclareLaunchArgument('token', default_value=''),
        DeclareLaunchArgument('launch_args', default_value='',
                              description='全モードの ros2 launch に足す引数 (例 "lidar:=l1")'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('start_mode', default_value='',
                              description='起動時に自動で始めるモード (goal / calibration, 空なら待つ)'),
        Node(
            package='robot_launcher',
            executable='launcher',
            name='robot_launcher',
            output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'mode_port': ParameterValue(LaunchConfiguration('mode_port'), value_type=int),
                'token': ParameterValue(LaunchConfiguration('token'), value_type=str),
                'launch_args': ParameterValue(LaunchConfiguration('launch_args'), value_type=str),
                'arduino_port': LaunchConfiguration('arduino_port'),
                'start_mode': ParameterValue(LaunchConfiguration('start_mode'), value_type=str),
            }],
        ),
    ])
