"""launch ファイルから robot_config を使うための補助.

  from robot_config.launch_helpers import robot_argument, config_of
  DeclareLaunchArgument の代わりに robot_argument() を並べ, OpaqueFunction の中で
  cfg = config_of(context) として cfg.node('<ノード>') をパラメータに渡す.
"""
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from robot_config.config import default_robot, load, robots

_cache = {}


def robot_argument():
    return DeclareLaunchArgument(
        'robot', default_value=default_robot(),
        description=f'設定を使うロボット (robot_config/config/robots/<名前>.yaml. 今あるもの: {", ".join(robots())})')


def config_of(context):
    """launch 引数 robot のロボットの設定 (1 回の起動の中では同じものを使う)."""
    name = LaunchConfiguration('robot').perform(context) or default_robot()
    if name not in _cache:
        _cache[name] = load(name)
    return _cache[name]


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def override(context, name):
    """launch 引数 name が空でなければその文字列, 空なら None (設定の値を使う)."""
    value = LaunchConfiguration(name).perform(context)
    return value if value != '' else None
