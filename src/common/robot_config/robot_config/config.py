"""設定の一元管理: config/common.yaml と config/robots/<ロボット>.yaml を読み, 各ノードのパラメータを作る.

ファイル
  common.yaml          ロボットに依らない設定. nodes: <ノード>: {パラメータ}
  robots/<名前>.yaml   ロボットごとの設定
                         robot: ロボットの事実 (寸法, LiDAR の取付, 地面の高さ, ポートなど)
                         nodes: このロボットだけの上書き (書き方は common.yaml と同じ)
  キャリブレーションの結果 (calibrate_drive の summary.yaml) はここに含めない. 起動時に別に渡す.

各ノードに渡す値 (後のものほど優先)
  1. common.yaml の nodes.<ノード>
  2. robot: の事実から計算した値 (derive(). 例: 自車体の箱, Nav2 の footprint, 後輪軸から LiDAR まで)
  3. robots/<名前>.yaml の nodes.<ノード>

ロボットの選び方: launch 引数 robot:=<名前> > 環境変数 ROBOT_NAME > rc_car
実際に渡る値は `ros2 run robot_config show [ノード...] [--robot <名前>]` で見られる.
"""
import copy
import math
import os
import tempfile

import yaml

DEFAULT_ROBOT = 'rc_car'


def default_robot():
    return os.environ.get('ROBOT_NAME') or DEFAULT_ROBOT


def config_dir():
    """インストール先 (share/robot_config/config). 見つからなければソースの config."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('robot_config'), 'config')
        if os.path.isdir(path):
            return path
    except Exception:   # noqa: BLE001 (ament が無いときやパッケージ外から使うとき)
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


def robots():
    """選べるロボットの名前."""
    d = os.path.join(config_dir(), 'robots')
    return sorted(f[:-5] for f in os.listdir(d) if f.endswith('.yaml'))


def merge(base, over):
    """辞書を深く重ねる (over が優先). リストや値は置き換え."""
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _read(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def footprint_string(points):
    """[[x, y], ...] -> Nav2 の footprint の文字列."""
    return '[' + ', '.join(f'[{float(x):g}, {float(y):g}]' for x, y in points) + ']'


def rotation_rpy(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll) (ROS / static_transform_publisher と同じ順)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class Config:
    """1 台のロボットの設定."""

    def __init__(self, robot=None):
        self.name = robot or default_robot()
        d = config_dir()
        path = os.path.join(d, 'robots', f'{self.name}.yaml')
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f'ロボット {self.name!r} の設定 {path} が無い (選べるもの: {", ".join(robots())})')
        self.common = _read(os.path.join(d, 'common.yaml'))
        self.robot_file = _read(path)
        self.robot = self.robot_file.get('robot') or {}

    # ---------- ロボットの事実 ----------
    @property
    def sim(self):
        return bool(self.robot.get('sim', False))

    @property
    def lidar(self):
        return self.robot['lidar']

    @property
    def mount(self):
        """base_link から見た LiDAR の取付 {x, y, z, roll_deg, pitch_deg, yaw_deg}."""
        m = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0}
        m.update(self.lidar.get('mount') or {})
        return m

    def mount_tf(self):
        """base_link -> LiDAR の (平行移動 [x, y, z], 姿勢 [roll, pitch, yaw] [rad])."""
        m = self.mount
        return ([m['x'], m['y'], m['z']],
                [math.radians(m[f'{k}_deg']) for k in ('roll', 'pitch', 'yaw')])

    def inverse_mount_tf(self):
        """LiDAR -> base_link (aft_mapped -> base_link の静的 TF)."""
        t, rpy = self.mount_tf()
        r = rotation_rpy(*rpy)
        rt = [[r[j][i] for j in range(3)] for i in range(3)]                  # 転置 = 逆回転
        t_inv = [-sum(rt[i][j] * t[j] for j in range(3)) for i in range(3)]
        pitch = math.asin(max(-1.0, min(1.0, -rt[2][0])))
        roll = math.atan2(rt[2][1], rt[2][2])
        yaw = math.atan2(rt[1][0], rt[0][0])
        clean = [0.0 if abs(v) < 1e-12 else v for v in t_inv]   # -0.0 を 0.0 に
        return clean, [0.0 if abs(v) < 1e-12 else v for v in (roll, pitch, yaw)]

    @property
    def body(self):
        return self.robot['body']

    # ---------- 事実から計算する値 ----------
    def derived(self):
        """ノードごとの, robot: の事実から計算した値."""
        m, body = self.mount, self.body
        sim_time = {'use_sim_time': self.sim}
        mount = {f'lidar_{k}': float(v) for k, v in m.items()}
        self_box = {'self_box_min': [float(v) for v in body['self_box_min']],
                    'self_box_max': [float(v) for v in body['self_box_max']]}
        ground = {'ground_z': float(body['ground_z'])}
        arduino = self.robot.get('arduino_port', '/dev/ttyACM0')
        cmd_vel = self.robot.get('cmd_vel_topic', '/cmd_vel')
        wheelbase = float(body['wheelbase'])
        rear_axle_x = float(body.get('rear_axle_x', -wheelbase / 2.0))
        footprint = footprint_string(body['footprint'])

        bridge = self._raw('lidar_bridge')
        launcher = self._raw('launcher')
        detector = merge(self._raw('obstacle_detector'), ground)
        d = {
            'lidar_bridge': dict(sim_time, **mount, **self_box),
            'obstacle_detector': dict(sim_time, **mount, **self_box, **ground),
            'stationary_detector': dict(sim_time, imu_topic=bridge.get('imu_out', '/imu'),
                                        cmd_vel_topic=cmd_vel),
            'point_lio': {
                'use_sim_time': self.sim,
                'common': {'lid_topic': bridge.get('cloud_out', '/lidar'),
                           'imu_topic': bridge.get('imu_out', '/imu')},
            },
            'unitree_lidar_l1': {'port': self.lidar.get('port', '/dev/ttyUSB0')},
            'unitree_lidar_l2': {
                'connection': self.lidar.get('l2_connection', 'ethernet'),
                'serial_port': self.lidar.get('port', '/dev/ttyACM0'),
                'lidar_ip': self.lidar.get('l2_lidar_ip', '192.168.1.62'),
                'local_ip': self.lidar.get('l2_local_ip', '192.168.1.2'),
            },
            'robot_map': dict(sim_time, **ground,
                              obstacle_min_height=detector.get('obstacle_min_height'),
                              obstacle_max_height=detector.get('obstacle_max_height'),
                              # ボクセルは Point-LIO の地図の粗さに合わせる. 保存先は launcher と同じ
                              voxel_size=float(self._raw('point_lio').get('filter_size_map', 0.1)),
                              save_dir=launcher.get('map_dir', '~/robot_maps'),
                              labeled_topic=detector.get('labeled_out', '/lidar_labeled')),
            'cmd_vel_converter': dict(sim_time, port=arduino, cmd_vel_topic=cmd_vel,
                                      wheelbase=wheelbase),
            'robot_goal_web': dict(sim_time),
            'launcher': {},
            'calib_web': {
                'port': launcher.get('mode_port', 8081),
                'calib_dir': launcher.get('calib_dir', '~/drive_calib'),
                'arduino_port': arduino,
                'wheelbase': wheelbase,
                'lidar_x': round(m['x'] - rear_axle_x, 6),   # 後輪軸から LiDAR まで
            },
            'navigation': {
                'use_sim_time': self.sim,
                'cmd_vel_topic': cmd_vel,
            },
            'filter_invalid_points': dict(sim_time, **self._lidar_frame_box()),
            'nav2': self._nav2_derived(footprint),
        }
        return d

    def _lidar_frame_box(self):
        """自車体の箱を LiDAR 座標に直したもの (LiDAR が傾いていないときだけ正しい. sim 用)."""
        t, rpy = self.mount_tf()
        if any(abs(a) > 1e-9 for a in rpy):
            return {}
        lo = [float(v) - t[i] for i, v in enumerate(self.body['self_box_min'])]
        hi = [float(v) - t[i] for i, v in enumerate(self.body['self_box_max'])]
        return {'self_box_min': [round(v, 6) for v in lo], 'self_box_max': [round(v, 6) for v in hi]}

    def _nav2_derived(self, footprint):
        out = {}
        for costmap in ('local_costmap', 'global_costmap'):
            out[costmap] = {costmap: {'ros__parameters': {'footprint': footprint,
                                                           'use_sim_time': self.sim}}}
        for server in ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server'):
            out[server] = {'ros__parameters': {'use_sim_time': self.sim}}
        return out

    # ---------- ノードに渡す値 ----------
    def _raw(self, node):
        """common と robot の nodes だけを重ねたもの (計算した値を含まない)."""
        return merge((self.common.get('nodes') or {}).get(node) or {},
                     (self.robot_file.get('nodes') or {}).get(node) or {})

    def node(self, node):
        """ノード (または launch の設定のまとまり) に渡すパラメータ."""
        common = (self.common.get('nodes') or {}).get(node) or {}
        robot = (self.robot_file.get('nodes') or {}).get(node) or {}
        return merge(merge(common, self.derived().get(node) or {}), robot)

    def node_names(self):
        names = set((self.common.get('nodes') or {}).keys())
        names |= set((self.robot_file.get('nodes') or {}).keys())
        names |= set(self.derived().keys())
        return sorted(names)

    def nav2_params(self, bt_xml=None):
        """Nav2 のパラメータの木 (nav2_bringup にそのまま渡せる形)."""
        tree = self.node('nav2')
        if bt_xml:
            tree = merge(tree, {'bt_navigator': {'ros__parameters': {
                'default_nav_to_pose_bt_xml': bt_xml}}})
        return tree

    def write_nav2_params(self, bt_xml=None):
        """Nav2 のパラメータを一時ファイルに書き出してパスを返す (nav2_bringup は params_file を読むため)."""
        fd, path = tempfile.mkstemp(prefix=f'nav2_params_{self.name}_', suffix='.yaml')
        with os.fdopen(fd, 'w') as f:
            f.write(f'# robot_config ({self.name}) から作った Nav2 のパラメータ. 編集しても次の起動で作り直される\n')
            yaml.safe_dump(self.nav2_params(bt_xml), f, allow_unicode=True, sort_keys=False)
        return path


def load(robot=None):
    return Config(robot)
