"""Point-LIO mapping for avia.

Node parameters below mirror the values of the upstream ROS 1 launch file; the
rest come from config/avia.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('point_lio_unilidar')
    config_file = os.path.join(share_dir, 'config', 'avia.yaml')
    rviz_config = os.path.join(share_dir, 'rviz_cfg', 'point_lio.rviz')

    use_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz alongside the mapping node.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Set true when replaying a bag or running in Gazebo.'),
        Node(
            package='point_lio_unilidar',
            executable='pointlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    # change to True to use the IMU as input of Point-LIO
                    'use_imu_as_input': False,
                    'prop_at_freq_of_imu': True,
                    'check_satu': True,
                    'init_map_size': 10,
                    'point_filter_num': 1,
                    'space_down_sample': False,
                    'filter_size_surf': 0.3,
                    'filter_size_map': 0.2,
                    'cube_side_length': 2000.0,
                    'runtime_pos_log_enable': False,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
