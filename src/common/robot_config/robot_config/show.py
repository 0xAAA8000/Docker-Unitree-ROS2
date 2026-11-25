"""各ノードに実際に渡る設定を表示する.

  ros2 run robot_config show                     # ノードの一覧とロボットの事実
  ros2 run robot_config show obstacle_detector   # そのノードに渡る値
  ros2 run robot_config show nav2 --robot sim    # sim の Nav2 のパラメータ
"""
import argparse
import sys

import yaml

from robot_config.config import default_robot, load, robots


def main(argv=None):
    ap = argparse.ArgumentParser(description='robot_config から各ノードに渡る値を表示する')
    ap.add_argument('nodes', nargs='*', help='ノード名 (省略すると一覧)')
    ap.add_argument('--robot', default=default_robot(),
                    help=f'ロボット ({", ".join(robots())}. 既定は $ROBOT_NAME か rc_car)')
    args = ap.parse_args(argv)
    cfg = load(args.robot)
    if not args.nodes:
        print(f'# ロボット: {cfg.name}')
        yaml.safe_dump({'robot': cfg.robot}, sys.stdout, allow_unicode=True, sort_keys=False)
        print('# ノード (ros2 run robot_config show <ノード> で値を表示):')
        for n in cfg.node_names():
            print(f'#   {n}')
        return
    for n in args.nodes:
        print(f'# {n} ({cfg.name})')
        yaml.safe_dump({n: cfg.node(n)}, sys.stdout, allow_unicode=True, sort_keys=False)


if __name__ == '__main__':
    main()
