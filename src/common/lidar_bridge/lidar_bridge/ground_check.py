"""床の点群から LiDAR の取付角度と地面の高さを推定し, robot.yaml の値と比べる.

  ros2 run lidar_bridge ground_check                 # LiDAR ドライバを起動した状態で, 平らな床に止めておく
  ros2 run lidar_bridge ground_check --ros-args -p config:=/path/to/robot.yaml

robot.yaml の取付位置・姿勢で点を base_link 座標に直し, 床の平面を当てはめる.
設定が正しければ床は水平 (傾き 0) で, 高さは ground_z になる.
傾いていれば lidar_pitch_deg / lidar_roll_deg が, 高さがずれていれば ground_z が実機と合っていない.
床が障害物 (地面から obstacle_min_height 以上) と判定される割合も表示する.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import yaml

from lidar_bridge.lidar_bridge_node import cloud_to_array, rotation_rpy

PERIOD = 3.0        # 集計の周期 [s]
INLIER = 0.03       # 平面からこの距離 [m] 以内を床の点とする


def fit_ground(base, lidar_z, iterations=300, rng=None):
    """base_link 座標の点から床の平面を探す. (法線 n (z>0), 平面の式 n.x = d, 床の点の bool) を返す.

    床の候補は LiDAR より下の, 水平距離 0.5 - 6 m の点. 法線が鉛直から 45 度以内の平面だけを採る.
    """
    rng = rng or np.random.default_rng(0)
    r = np.hypot(base[:, 0], base[:, 1])
    cand = base[(r > 0.5) & (r < 6.0) & (base[:, 2] < lidar_z)]
    if len(cand) < 50:
        return None
    best = None
    for _ in range(iterations):
        p = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if n[2] < 0:
            n = -n
        if n[2] < math.cos(math.radians(45)):
            continue
        count = np.count_nonzero(np.abs(cand @ n - n @ p[0]) < INLIER)
        if best is None or count > best[0]:
            best = (count, n, n @ p[0])
    if best is None:
        return None
    _, n, d = best
    # 床の点で最小二乗の平面に直す
    for _ in range(2):
        pts = cand[np.abs(cand @ n - d) < INLIER]
        c = pts.mean(axis=0)
        n = np.linalg.eigh(np.cov((pts - c).T))[1][:, 0]   # 分散が最小の向き = 法線 (3x3 で計算)
        if n[2] < 0:
            n = -n
        d = n @ c
    return n, d, np.abs(base @ n - d) < INLIER


def suggest(xyz_lidar, mount, rng=None):
    """LiDAR 座標の点と取付設定 (dict) から推定結果を返す. 床が見つからなければ None."""
    rot = rotation_rpy(*[math.radians(mount[f'lidar_{k}_deg']) for k in ('roll', 'pitch', 'yaw')])
    trans = np.array([mount['lidar_x'], mount['lidar_y'], mount['lidar_z']])
    base = xyz_lidar @ rot.T + trans
    fit = fit_ground(base, mount['lidar_z'], rng=rng)
    if fit is None:
        return None
    n, d, ground = fit
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    # 床が水平になる roll と pitch を求める. 床の傾きからは yaw は決まらないので今の値のまま.
    # R = Rz(yaw) Ry(pitch) Rx(roll) の 3 行目 (-sin p, cos p sin r, cos p cos r) が
    # LiDAR 座標での床の法線になればよい
    n_lidar = rot.T @ n
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -n_lidar[0]))))
    roll = math.degrees(math.atan2(n_lidar[1], n_lidar[2]))
    yaw = mount['lidar_yaw_deg']
    rot_new = rotation_rpy(math.radians(roll), math.radians(pitch), math.radians(yaw))
    base_new = xyz_lidar @ rot_new.T + trans
    ground_z = float(np.median(base_new[ground, 2]))
    # 今の設定で, 床の点が障害物と判定される割合
    h = base[ground, 2] - mount['ground_z']
    obs = (h > mount['obstacle_min_height']) & (h < mount['obstacle_max_height'])
    # 床の点のうち, 今の設定で床から何 m 浮いて見えるか (遠くほど大きいなら角度のずれ)
    r = np.hypot(base[ground, 0], base[ground, 1])
    rise = {}
    for lo, hi in ((0.5, 1.5), (1.5, 3.0), (3.0, 6.0)):
        m = (r >= lo) & (r < hi)
        if np.count_nonzero(m) >= 10:
            rise[(lo, hi)] = float(np.median(h[m]))
    return {
        'tilt_deg': tilt, 'roll_deg': roll, 'pitch_deg': pitch, 'yaw_deg': yaw,
        'ground_z': ground_z, 'ground_points': int(np.count_nonzero(ground)),
        'obstacle_ratio': float(np.mean(obs)) if len(obs) else 0.0, 'rise': rise,
    }


def near_obstacles(xyz_lidar, mount, self_min, self_max, radius=1.0):
    """lidar_bridge が障害物とする点のうち, base_link から水平 radius [m] 以内の点 (base_link 座標).

    車体や LiDAR の取付台が写っていると, ここに出る. self_box_min / self_box_max で除く.
    """
    rot = rotation_rpy(*[math.radians(mount[f'lidar_{k}_deg']) for k in ('roll', 'pitch', 'yaw')])
    base = xyz_lidar @ rot.T + np.array([mount['lidar_x'], mount['lidar_y'], mount['lidar_z']])
    in_self = np.all((base > self_min) & (base < self_max), axis=1)
    h = base[:, 2] - mount['ground_z']
    obs = ~in_self & (h > mount['obstacle_min_height']) & (h < mount['obstacle_max_height'])
    near = obs & (np.hypot(base[:, 0], base[:, 1]) < radius)
    return base[near]


class GroundCheck(Node):

    def __init__(self):
        super().__init__('ground_check')
        default = ''
        try:
            from ament_index_python.packages import get_package_share_directory
            default = os.path.join(get_package_share_directory('lidar_bridge'), 'config',
                                   'robot.yaml')
        except Exception:   # noqa: BLE001 (パッケージ外から実行したとき)
            pass
        config = self.declare_parameter('config', default).value
        topic = self.declare_parameter('cloud_topic', '/unilidar/cloud').value
        with open(config) as f:
            doc = yaml.safe_load(f)
        prm = dict(doc['lidar_bridge']['ros__parameters'])
        # 地面の高さと障害物の高さは obstacle_detector の項にある
        prm.update((doc.get('obstacle_detector') or {}).get('ros__parameters') or {})
        self.mount = {k: float(prm.get(k, v)) for k, v in (
            ('lidar_x', 0.2), ('lidar_y', 0.0), ('lidar_z', 0.25), ('lidar_roll_deg', 0.0),
            ('lidar_pitch_deg', 0.0), ('lidar_yaw_deg', 0.0), ('ground_z', -0.1),
            ('obstacle_min_height', 0.15), ('obstacle_max_height', 0.5))}
        self.range_min = float(prm.get('obstacle_range_min', 0.08))
        self.self_min = np.array(prm.get('self_box_min', [-0.4, -0.25, -0.2]), dtype=float)
        self.self_max = np.array(prm.get('self_box_max', [0.4, 0.25, 0.4]), dtype=float)
        self.buf = []
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(PERIOD, self.report)
        m = self.mount
        self.get_logger().info(
            f'{config} の設定: roll={m["lidar_roll_deg"]:.1f} pitch={m["lidar_pitch_deg"]:.1f} '
            f'yaw={m["lidar_yaw_deg"]:.1f} deg, lidar_z={m["lidar_z"]:.2f}, '
            f'ground_z={m["ground_z"]:.2f} m. {topic} を待っています (平らな床に止めておく)')

    def on_cloud(self, msg):
        pts = cloud_to_array(msg)
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[np.einsum('ij,ij->i', xyz, xyz) >= self.range_min ** 2]
        self.buf.append(xyz)

    def report(self):
        if not self.buf:
            self.get_logger().warn('点群が届いていない')
            return
        xyz = np.concatenate(self.buf)
        self.buf = []
        if len(xyz) > 60000:
            xyz = xyz[np.random.default_rng().choice(len(xyz), 60000, replace=False)]
        self.report_near(xyz)
        s = suggest(xyz, self.mount)
        if s is None:
            self.get_logger().warn('床の平面が見つからない (LiDAR の前 0.5-6 m に床が見えている?)')
            return
        m = self.mount
        rise = '  '.join(f'{lo:.1f}-{hi:.1f}m: {v:+.2f}' for (lo, hi), v in s['rise'].items())
        print(
            f'\n床の点 {s["ground_points"]} 点. 今の設定で床が障害物になる割合 '
            f'{100 * s["obstacle_ratio"]:.0f}%\n'
            f'  今の設定で床が地面 (ground_z) から浮いて見える高さ [m]: {rise}\n'
            f'  今の設定で見た床の傾き {s["tilt_deg"]:.1f} deg (0 なら取付角度は合っている)\n'
            f'  推定値: lidar_roll_deg {s["roll_deg"]:.1f} (今 {m["lidar_roll_deg"]:.1f})  '
            f'lidar_pitch_deg {s["pitch_deg"]:.1f} (今 {m["lidar_pitch_deg"]:.1f})  '
            f'ground_z {s["ground_z"]:.3f} (今 {m["ground_z"]:.3f})', flush=True)

    def report_near(self, xyz):
        near = near_obstacles(xyz, self.mount, self.self_min, self.self_max)
        fmt = lambda v: '[' + ', '.join(f'{x:.2f}' for x in v) + ']'  # noqa: E731
        if len(near) < 5:
            print(f'\nロボットから 1 m 以内の障害物の点: なし (自車体を除く箱 {fmt(self.self_min)}'
                  f' - {fmt(self.self_max)})', flush=True)
            return
        lo, hi = near.min(axis=0), near.max(axis=0)
        print(
            f'\nロボットから 1 m 以内の障害物の点: {len(near)} 点, 範囲 (base_link) '
            f'{fmt(lo)} - {fmt(hi)}\n'
            f'  周りに本当の障害物が無いのにここに点があるなら, 車体や取付台が写っている.\n'
            f'  self_box_min / self_box_max (今 {fmt(self.self_min)} - {fmt(self.self_max)}) を\n'
            f'  この範囲を含むよう広げる: 例 self_box_min {fmt(np.minimum(self.self_min, lo - 0.03))}'
            f'  self_box_max {fmt(np.maximum(self.self_max, hi + 0.03))}', flush=True)


def main():
    rclpy.init()
    node = GroundCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
