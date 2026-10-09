"""Unitree L2 localization on a saved PCD map (lidar_localization_ros2, NDT_OMP).

  ros2 launch l2_bringup localization.launch.py map:=/home/okamoto/maps/xxx.pcd
  ros2 launch l2_bringup localization.launch.py map:=... x:=1.0 y:=2.0 yaw:=90
  ros2 launch l2_bringup localization.launch.py map:=... auto_init:=true  # find the start pose
  ros2 launch l2_bringup localization.launch.py map:=... relocalize:=false  # no re-localization
  ros2 launch l2_bringup localization.launch.py map:=... connection:=serial rviz:=false
  ros2 launch l2_bringup localization.launch.py map:=... record:=true   # ros2 bag to ~/bags/l2loc_<date>

The map origin is where Point-LIO started when the map was made, so starting at
that spot needs no initial pose. Otherwise give x/y/yaw[deg], use auto_init:=true
(l2_initial_pose matches ~3 s of scans against the map), or "2D Pose Estimate" in RViz.
"""
from datetime import datetime
import glob
import math
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node, SetRemap
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


# Render RViz on the NVIDIA GPU instead of the Intel iGPU (PRIME on-demand).
# Only when the NVIDIA GLX library exists; forcing it without one makes RViz fail to start.
NVIDIA_RENDER_ENV = (
    {'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'}
    if glob.glob('/usr/lib/*/libGLX_nvidia.so.0') else {})

# Raw sensor data + estimates, enough to replay and re-tune offline (~7 GB/hour with L2).
# /path is left out: it republishes the whole path every scan.
RECORD_TOPICS = ['/unilidar/cloud', '/unilidar/imu', '/pcl_pose', '/tf', '/tf_static']


def default_bag_path():
    return os.path.expanduser(datetime.now().strftime('~/bags/l2loc_%Y%m%d_%H%M%S'))


def localization_node(context):
    yaw = math.radians(float(LaunchConfiguration('yaw').perform(context)))
    auto = LaunchConfiguration('auto_init').perform(context).lower() == 'true'
    node = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[
            PathJoinSubstitution([FindPackageShare('l2_bringup'), 'config', 'localization.yaml']),
            {
                'map_path': LaunchConfiguration('map').perform(context),
                'initial_pose_x': float(LaunchConfiguration('x').perform(context)),
                'initial_pose_y': float(LaunchConfiguration('y').perform(context)),
                'initial_pose_z': float(LaunchConfiguration('z').perform(context)),
                'initial_pose_qx': 0.0,
                'initial_pose_qy': 0.0,
                'initial_pose_qz': math.sin(yaw / 2),
                'initial_pose_qw': math.cos(yaw / 2),
                # with auto_init, wait for l2_initial_pose instead of starting at x/y/yaw
                'set_initial_pose': not auto,
            },
        ],
        remappings=[('/cloud', '/unilidar/cloud')],
        output='screen',
    )

    def change_state(transition):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node), transition_id=transition))

    # unconfigured -> (configure: load map) -> inactive -> (activate) -> active
    activate_when_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[LogInfo(msg='-- map loaded, activating --'),
                  change_state(Transition.TRANSITION_ACTIVATE)],
    ))
    actions = [activate_when_configured, node, change_state(Transition.TRANSITION_CONFIGURE)]
    reloc = LaunchConfiguration('relocalize').perform(context).lower() == 'true'
    if auto or reloc:
        init_map = LaunchConfiguration('init_map').perform(context) or \
            LaunchConfiguration('map').perform(context)
        actions.append(Node(
            package='l2_bringup',
            executable='l2_initial_pose.py',
            name='l2_initial_pose',
            parameters=[{'map_path': init_map, 'auto_init': auto, 'relocalize': reloc}],
            remappings=[('cloud', '/unilidar/cloud'), ('imu', '/unilidar/imu'),
                        ('pcl_pose', '/pcl_pose'), ('initialpose', '/initialpose')],
            output='screen',
        ))
    return actions


def generate_launch_description():
    # GroupAction scopes the driver's rviz:=false so it does not override ours. The driver
    # also broadcasts unilidar_imu -> unilidar_lidar on /tf, which gives unilidar_lidar a second
    # parent next to our base_link and breaks the lookup; move it out of the way.
    driver = GroupAction([SetRemap('/tf', '/unilidar/tf'), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unitree_lidar_ros2'), 'launch.py'])),
        launch_arguments={'connection': LaunchConfiguration('connection'),
                          'rviz': 'false'}.items(),
    )])

    # L2 is the robot body: base_link == unilidar_lidar
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'unilidar_lidar'],
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
            FindPackageShare('l2_bringup'), 'rviz', 'localization.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag')] + RECORD_TOPICS,
        condition=IfCondition(LaunchConfiguration('record')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='PCD map (downsampled recommended)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='initial yaw [deg]'),
        DeclareLaunchArgument('auto_init', default_value='false',
                              description='estimate the start pose on the map (keep still ~3 s)'),
        DeclareLaunchArgument('relocalize', default_value='true',
                              description='find the pose on the map again when tracking is lost'),
        DeclareLaunchArgument('init_map', default_value='',
                              description='map for auto_init / relocalize (binary PCD); default: map'),
        DeclareLaunchArgument('connection', default_value='ethernet',
                              description="'ethernet' or 'serial'"),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='false',
                              description='record sensor data and estimates with ros2 bag'),
        DeclareLaunchArgument('bag', default_value=default_bag_path(),
                              description='output directory of the recording'),
        driver,
        lidar_tf,
        OpaqueFunction(function=localization_node),
        data_watch,
        rviz_node,
        recorder,
    ])
