"""Unitree L2 + Point-LIO live mapping with RViz.

  ros2 launch l2_bringup mapping.launch.py                    # Ethernet, map not saved
  ros2 launch l2_bringup mapping.launch.py save:=true         # save map on Ctrl+C
  ros2 launch l2_bringup mapping.launch.py connection:=serial
  ros2 launch l2_bringup mapping.launch.py rviz:=false
  ros2 launch l2_bringup mapping.launch.py record:=true      # ros2 bag to ~/bags/l2map_<date>

Saved map goes to <point_lio source>/PCD/scans.pcd (overwritten every run; move it if you want to keep it).
"""
from datetime import datetime
import glob
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Render RViz on the NVIDIA GPU instead of the Intel iGPU (PRIME on-demand).
# Only when the NVIDIA GLX library exists; forcing it without one makes RViz fail to start.
NVIDIA_RENDER_ENV = (
    {'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'}
    if glob.glob('/usr/lib/*/libGLX_nvidia.so.0') else {})

# Raw sensor data, enough to replay and re-tune offline (~7 GB/hour with L2). Point-LIO's
# /aft_mapped_to_init (~12 kHz) and /path (whole path every scan) would bloat it; l2eval regenerates them.
RECORD_TOPICS = ['/unilidar/cloud', '/unilidar/imu']


def default_bag_path():
    return os.path.expanduser(datetime.now().strftime('~/bags/l2map_%Y%m%d_%H%M%S'))



def generate_launch_description():
    connection = LaunchConfiguration('connection')
    save = LaunchConfiguration('save')
    rviz = LaunchConfiguration('rviz')

    # GroupAction scopes the driver's rviz:=false so it does not override ours
    driver = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unitree_lidar_ros2'), 'launch.py'])),
        launch_arguments={'connection': connection, 'rviz': 'false'}.items(),
    )])

    # Same parameters as point_lio/mapping_unilidar_l2.launch.py, plus save switch
    point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('point_lio'), 'config', 'unilidar_l2.yaml']),
            {
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 1,
                'space_down_sample': True,
                'filter_size_surf': 0.1,
                'filter_size_map': 0.1,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                'pcd_save.pcd_save_en': ParameterValue(save, value_type=bool),
            },
        ],
    )

    data_watch = Node(
        package='l2_bringup',
        executable='l2_data_watch.py',
        name='l2_data_watch',
        remappings=[('cloud', '/unilidar/cloud'), ('imu', '/unilidar/imu')],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        additional_env=NVIDIA_RENDER_ENV,
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'), 'rviz_cfg', 'loam_livox.rviz'])],
        condition=IfCondition(rviz),
        output='log',
    )

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag')] + RECORD_TOPICS,
        condition=IfCondition(LaunchConfiguration('record')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('connection', default_value='ethernet',
                              description="'ethernet' or 'serial'"),
        DeclareLaunchArgument('save', default_value='false',
                              description='save the map (PCD) when stopped with Ctrl+C'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='false',
                              description='record sensor data and estimates with ros2 bag'),
        DeclareLaunchArgument('bag', default_value=default_bag_path(),
                              description='output directory of the recording'),
        driver,
        point_lio,
        data_watch,
        rviz_node,
        recorder,
    ])
