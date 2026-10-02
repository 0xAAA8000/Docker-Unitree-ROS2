"""Unitree L2 localization on a saved PCD map (lidar_localization_ros2, NDT_OMP).

  ros2 launch l2_bringup localization.launch.py map:=/home/okamoto/maps/xxx.pcd
  ros2 launch l2_bringup localization.launch.py map:=... x:=1.0 y:=2.0 yaw:=90
  ros2 launch l2_bringup localization.launch.py map:=... connection:=serial rviz:=false

The map origin is where Point-LIO started when the map was made, so starting at
that spot needs no initial pose. Otherwise give x/y/yaw[deg] or use "2D Pose
Estimate" in RViz.
"""
import glob
import math

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


# Render RViz on the NVIDIA GPU instead of the Intel iGPU (PRIME on-demand).
# Only when the NVIDIA GLX library exists; forcing it without one makes RViz fail to start.
NVIDIA_RENDER_ENV = (
    {'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'}
    if glob.glob('/usr/lib/*/libGLX_nvidia.so.0') else {})


def localization_node(context):
    yaw = math.radians(float(LaunchConfiguration('yaw').perform(context)))
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
    return [activate_when_configured, node, change_state(Transition.TRANSITION_CONFIGURE)]


def generate_launch_description():
    # GroupAction scopes the driver's rviz:=false so it does not override ours
    driver = GroupAction([IncludeLaunchDescription(
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

    return LaunchDescription([
        DeclareLaunchArgument('map', description='PCD map (downsampled recommended)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='initial yaw [deg]'),
        DeclareLaunchArgument('connection', default_value='ethernet',
                              description="'ethernet' or 'serial'"),
        DeclareLaunchArgument('rviz', default_value='true'),
        driver,
        lidar_tf,
        OpaqueFunction(function=localization_node),
        rviz_node,
    ])
