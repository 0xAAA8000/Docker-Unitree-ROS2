import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. パッケージやファイルのパス設定
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_sim_gazebo = get_package_share_directory('sim_gazebo')

    config_file = os.path.join(pkg_sim_gazebo, 'config', 'bridge_config.yaml')
    default_world_path = os.path.join(pkg_sim_gazebo, 'world', 'flat_kurifarm.world')

    # 名称'world'のLaunch引数を定義
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description="Path to the SDF world file"
    )
    # LaunchConfiguration経由で引数を取得
    world_path = LaunchConfiguration('world')
    #world_path = default_world_path

    # 2. Gazebo (Ignition) シミュレータ本体の起動設定
    # gz_args: '-r' は自動再生(run)、'empty.sdf' は標準の空ワールド指定
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [world_path, ' -r']}.items()
        #launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. parameter_bridge ノードの起動設定
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': config_file, 'use_sim_time': True}],
        output='screen'
    )

    urdf_path = os.path.join(pkg_sim_gazebo, 'urdf/four_wheel_car.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace('CONTROLLER_YAML_PATH', pkg_sim_gazebo)

    # ros2 controlにURDFを渡す
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'four_wheel_car',
            '-x', '0.0',
            '-y', '2.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
               '--controller-manager', '/controller_manager'],
        output='screen',
    )

    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
               '--controller-manager', '/controller_manager'],
        output='screen',
    )

    static_tf_pub = Node( # 名前変換
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            '0', '0', '0', '0', '0', '0', # xyz rpy 位置差
            'lidar_link',                 # 親フレーム
            'four_wheel_car/base_link/gpu_lidar' # 子フレーム
        ]
    )

    add_timestamp_field = Node(
        package='sim_gazebo',
        executable='add_time_field.py',
        name='pointcloud_time_adder',
        output='screen'
    )

    # 4. まとめて起動
    return LaunchDescription([
        world_arg,
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot,
        # spawn 完了後に順番に起動する（先に走らせると controller_manager がまだ無い）
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_spawner])),
        static_tf_pub,
        add_timestamp_field,
    ])

