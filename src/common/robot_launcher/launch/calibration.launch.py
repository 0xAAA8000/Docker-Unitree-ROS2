"""キャリブレーションのモード: LiDAR ドライバと Point-LIO (ZUPT なし) と, キャリブレーションの画面.

  ros2 launch robot_launcher calibration.launch.py port:=8081

Nav2 と cmd_vel_converter は起動しない (calibrate_drive が Arduino を直接使うため).
ZUPT は切る (calibrate_drive は /cmd_vel を出さないので, 一定の速さで走ると停止と誤判定する).
robot.launch.py の他の引数 (lidar:=, lidar_port:= など) はそのまま渡る.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={
            'nav': 'false',
            'operation': 'false',
            'map': 'false',
            'web': 'false',
            'zupt': 'false',
        }.items(),
    )
    calib_web = Node(
        package='robot_launcher',
        executable='calib_web',
        name='calib_web',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'token': ParameterValue(LaunchConfiguration('token'), value_type=str),
            'launcher_port': ParameterValue(LaunchConfiguration('launcher_port'), value_type=int),
            'arduino_port': LaunchConfiguration('arduino_port'),
            'calib_dir': LaunchConfiguration('calib_dir'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8081', description='キャリブレーションの画面のポート'),
        DeclareLaunchArgument('token', default_value='', description='?key=<token> を必須にする'),
        DeclareLaunchArgument('launcher_port', default_value='0',
                              description='モード選択の画面のポート (0 なら戻るボタンを出さない)'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('calib_dir', default_value=os.path.expanduser('~/drive_calib'),
                              description='計測結果を置くディレクトリ'),
        robot,
        calib_web,
    ])
