import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Usage:
#   ros2 launch unitree_lidar_ros2 launch.py                         # Ethernet (factory default)
#   ros2 launch unitree_lidar_ros2 launch.py connection:=serial      # USB serial (/dev/ttyACM0)
#   ros2 launch unitree_lidar_ros2 launch.py rviz:=false
#
# NOTE: the node writes work_mode to the LiDAR at startup.
#   ethernet -> work_mode 0, serial -> work_mode 8 (bit3 = serial).
#   A mode change takes effect after power-cycling the LiDAR.


def launch_setup(context, *args, **kwargs):
    connection = LaunchConfiguration('connection').perform(context)
    if connection not in ('ethernet', 'serial'):
        raise RuntimeError("connection must be 'ethernet' or 'serial', got: " + connection)
    serial = connection == 'serial'

    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                {'initialize_type': 1 if serial else 2},
                {'work_mode': 8 if serial else 0},
                {'use_system_timestamp': True},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': LaunchConfiguration('serial_port').perform(context)},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': LaunchConfiguration('lidar_ip').perform(context)},
                {'local_port': 6201},
                {'local_ip': LaunchConfiguration('local_ip').perform(context)},

                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},
                ]
    )

    # Run Rviz
    package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
    rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return [node1, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('connection', default_value='ethernet',
                              description="'ethernet' or 'serial'"),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.62'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.2'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
