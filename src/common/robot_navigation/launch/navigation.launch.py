import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # apt install された nav2_bringup のパスを取得
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_robot_navigation = get_package_share_directory('robot_navigation')

    # Launch引数の定義
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    publish_base_tf = LaunchConfiguration('publish_base_tf')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # 既定は実機. sim:=true にすると, 下の 4 つの既定値がまとめて sim (Gazebo) 用になる.
    # 個別に指定した引数は, sim の値より優先される.
    sim = LaunchConfiguration('sim')
    declare_sim_cmd = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true なら Gazebo シミュレーション用の既定値で起動する'
    )

    def _pick(sim_value, real_value):
        """sim:=true なら sim_value, それ以外は real_value を返す substitution."""
        return PythonExpression(
            ["'", sim_value, "' if '", sim, "'.lower() == 'true' else '", real_value, "'"])

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=sim,
        description='シミュレーション時間を使うか (既定: sim と同じ)'
    )

    # 実機では lidar_bridge の real.launch.py が LiDAR の傾きを含めて出すので false にする
    declare_publish_base_tf_cmd = DeclareLaunchArgument(
        'publish_base_tf',
        default_value=sim,
        description='aft_mapped -> base_link の静的 TF (sim 用の値) を出すか (既定: sim と同じ)'
    )

    # sim: ackermann_steering_controller へ / 実機: /cmd_vel (robot_operation の cmd_vel_converter)
    declare_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value=_pick('/ackermann_steering_controller/reference_unstamped', '/cmd_vel'),
        description='Nav2 の /cmd_vel のリマップ先'
    )

    # 実機: nav2_params_real.yaml (車体 0.50 x 0.30 m) / sim: nav2_params.yaml
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=_pick(os.path.join(pkg_robot_navigation, 'config', 'nav2_params.yaml'),
                            os.path.join(pkg_robot_navigation, 'config', 'nav2_params_real.yaml')),
        description='Nav2パラメータファイル(YAML)のフルパス'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Nav2ノードを自動的にActivate状態にするか'
    )

    # ----------------------------------------------------------------------
    # 1. Static TF: map -> camera_init (固定ワールドフレームの統合)
    # ----------------------------------------------------------------------
    tf_map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ----------------------------------------------------------------------
    # 2. Static TF: aft_mapping -> base_link (LiDAR位置から車両中心へのオフセット)
    # ※ 実機の設置位置に合わせて x, y, z の値を修正してください
    # ----------------------------------------------------------------------
    tf_aft_mapping_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aft_mapping_to_base_link_publisher',
        arguments=['-0.2', '0.0', '-0.25', '0.0', '0.0', '0.0', 'aft_mapped', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_base_tf)
    )

    # ----------------------------------------------------------------------
    # 3. Nav2 Navigation Launch
    # /cmd_vel を /ackermann_steering_control にリマップして一括起動
    # ----------------------------------------------------------------------
    bt_xml = os.path.join(pkg_robot_navigation, 'behavior_trees', 'navigate_keep_path.xml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_xml},
        convert_types=True
    )
    nav2_group = GroupAction(
        actions=[
            # トピックのリマップ(経路)を設定。型変換はしてない。
            # 実機 (cmd_vel_topic:=/cmd_vel) では付け替えない。/cmd_vel -> /cmd_vel の付け替えは
            # Nav2 自身の付け替え (cmd_vel -> cmd_vel_nav) より優先されるため, velocity_smoother が
            # /cmd_vel を読んで /cmd_vel に書くループになり, 動き出さない・止まらない原因になる
            SetRemap(src='/cmd_vel', dst=cmd_vel_topic,
                     condition=IfCondition(PythonExpression(
                         ["'", cmd_vel_topic, "' != '/cmd_vel'"]))),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_params,
                    'autostart': autostart
                }.items()
            )
        ]
    )

    return LaunchDescription([
        declare_sim_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_publish_base_tf_cmd,
        declare_cmd_vel_topic_cmd,
        #tf_map_to_camera_init,
        tf_aft_mapping_to_base_link,
        nav2_group
    ])
