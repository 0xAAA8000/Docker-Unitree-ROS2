import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_operation')

    params_file = LaunchConfiguration('params_file')
    calibration_file = LaunchConfiguration('calibration_file')
    port = LaunchConfiguration('port')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg, 'config', 'operation.yaml'),
            description='cmd_vel_converter のパラメータファイル'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='calibrate_drive が出力した summary.yaml (空なら params_file の値)'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Arduino のシリアルポート'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true なら Arduino に送らずログに出すだけ'),
        Node(
            package='robot_operation',
            executable='cmd_vel_converter',
            name='cmd_vel_converter',
            output='screen',
            parameters=[params_file, {
                'calibration_file': calibration_file,
                'port': port,
                'dry_run': dry_run,
            }],
        ),
    ])
