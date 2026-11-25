"""Nav2 を起動する. パラメータは robot_config (common.yaml の nodes.nav2 + robots/<robot>.yaml) から作る.

  ros2 launch robot_navigation navigation.launch.py               # 実機 (ロボットは $ROBOT_NAME か rc_car)
  ros2 launch robot_navigation navigation.launch.py robot:=sim    # Gazebo

  * Nav2 のパラメータは起動のたびに一時ファイル (/tmp/nav2_params_<robot>_*.yaml) に書き出して渡す.
    中身は `ros2 run robot_config show nav2 --robot <robot>` でも見られる
  * use_sim_time, /cmd_vel の行き先 (cmd_vel_topic), aft_mapped -> base_link の静的 TF を出すか
    (navigation.publish_base_tf) もロボットの設定から決まる
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

from robot_config.launch_helpers import config_of, robot_argument


def _setup(context):
    cfg = config_of(context)
    nav = cfg.node('navigation')
    use_sim_time = 'true' if nav.get('use_sim_time') else 'false'
    cmd_vel_topic = nav.get('cmd_vel_topic', '/cmd_vel')

    bt_xml = os.path.join(get_package_share_directory('robot_navigation'),
                          'behavior_trees', 'navigate_keep_path.xml')
    params_file = cfg.write_nav2_params(bt_xml)

    actions = []
    if nav.get('publish_base_tf'):
        # aft_mapped (LiDAR) -> base_link. 実機では lidar_bridge の real.launch.py が出すので出さない
        t, rpy = cfg.inverse_mount_tf()
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='aft_mapping_to_base_link_publisher',
            arguments=['--x', f'{t[0]:.6f}', '--y', f'{t[1]:.6f}', '--z', f'{t[2]:.6f}',
                       '--roll', f'{rpy[0]:.6f}', '--pitch', f'{rpy[1]:.6f}', '--yaw', f'{rpy[2]:.6f}',
                       '--frame-id', 'aft_mapped', '--child-frame-id', 'base_link'],
            parameters=[{'use_sim_time': nav.get('use_sim_time', False)}],
        ))

    group = []
    # /cmd_vel の付け替え. 実機 (cmd_vel_topic == /cmd_vel) では付け替えない。/cmd_vel -> /cmd_vel の付け替えは
    # Nav2 自身の付け替え (cmd_vel -> cmd_vel_nav) より優先されるため, velocity_smoother が
    # /cmd_vel を読んで /cmd_vel に書くループになり, 動き出さない・止まらない原因になる
    if cmd_vel_topic != '/cmd_vel':
        group.append(SetRemap(src='/cmd_vel', dst=cmd_vel_topic))
    group.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true' if nav.get('autostart', True) else 'false',
        }.items(),
    ))
    actions.append(GroupAction(actions=group))
    return actions


def generate_launch_description():
    return LaunchDescription([
        robot_argument(),
        OpaqueFunction(function=_setup),
    ])
