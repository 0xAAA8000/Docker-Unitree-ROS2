"""Unitree L1/L2 ドライバのトピックを, sim と同じ名前・形で Point-LIO と Nav2 に配信する.

  /unilidar/cloud --(自車体・近すぎる点を除去)--> /lidar            (Point-LIO)
                  --(地面からの高さで抽出)------> /lidar_obstacles  (Nav2 の障害物)
                  --(地面の点を少し奥へずらす)--> /lidar_clear      (Nav2 のクリア用)

/lidar_clear について
  Nav2 の costmap は LiDAR から点までの線上のセルを空きにするが, 点のあるセル自体は空きにしない.
  床の上に一度残った誤検知 (自己位置のずれなどで付いた障害物) は, その床の点が届いても消えない.
  そこで地面の点 (obstacle_min_height 未満) だけを, LiDAR から見た水平方向に
  ground_clear_extend [m] 奥へずらして配信する. 床が見えているセルは毎回空きに戻る.
  本当の障害物は同じ更新で付け直されるので消えない (Nav2 はクリアのあとに障害物を付ける)
  /unilidar/imu   --(frame_id だけ付け替え)-----> /imu              (Point-LIO)

sim (sim_gazebo/filter_invalid_points.py) との違い
  * 実機の点群には time フィールドがあり, 無効点も来ないので time の付加はしない
  * LiDAR を前に傾けて付けているので, 高さと自車体の判定は LiDAR 座標ではなく
    base_link 座標 (取付位置・姿勢で変換) で行う. 配信する点の座標は LiDAR 座標のまま
  * frame_id を lidar_link / imu_link に付け替える. ドライバが出す
    unilidar_imu_initial -> unilidar_imu の TF (IMU の姿勢) とぶつからないようにするため

坂の判定 (slope_filter が true のとき)
  高さだけで判定すると, 緩い坂も地面から obstacle_min_height 以上になった所から障害物になる.
  そこで車体の周り slope_range [m] の点を, 上から見た slope_cell [m] のマスに分けて
  マスごとの高さ (平均) と厚み (最高 - 最低) を求め, 「床から坂を伝ってたどり着ける」マスを床から広げる.
    * 出発点ははっきり床と言えるマス (点が slope_min_points 個以上, 高さ < obstacle_min_height の半分,
      厚みが小さい). 点が 1 つのマスは, ばらつきで低く出ると段の上でも床に見えるため使わない
    * 進めるのは隣 (8 方向) のマスで, 高さの差が step_max 以下 (段差の上限)
    * 進んだ先の床以外のマスは, 点が slope_min_points 個以上, 厚みが小さく (壁や箱の側面は厚い),
      周り 5 x 5 マスの高さに当てはめた平面の傾きが slope_max_deg 以下
  たどり着けるマスの障害物の点は坂として障害物から外す. 箱や台は上面が平らでも, 縁の段差で
  たどり着けないので障害物のまま. 点がまばらで途切れた坂も (安全側に) 障害物のまま. 範囲の外は高さだけで判定する.
  高さは base_link 基準 (車体が傾くと一緒に傾く) なので, 傾きは車体に対する値になる.

センサの途切れの補完 (gap_fill_timeout > 0 のとき)
  Point-LIO は点群が途切れると, その間を IMU だけで積分する. 再開時には位置が飛んで地図と
  照合できなくなり, 一方向に流れ続けたり, 別の場所に新しい地図を作り始めたりする.
  そこで点群が gap_fill_timeout [s] 以上届かないと, 途切れの間を
  「その場で回転だけしていた (回転はジャイロのとおり, 平行移動はしない)」とみなして補完する.
    * IMU   : 角速度は実際の値. 加速度は「途切れる直前の重力の向きを, ジャイロで求めた
              姿勢の変化で回したもの」に置き換える (並進の加速度を 0 にする)
    * 点群  : 途切れる直前の点群を, その後の姿勢の変化で回して時刻を進めながら配信する.
              Point-LIO からは同じ場所で向きだけ変わったように見え, 位置が保たれる
  IMU も途切れたときは角速度 0 (静止) として IMU を補完する.
  途切れている間に平行移動した分は再開後の照合で戻す (約 2 m を越えると戻れない).
  出力の時刻は実時間のまま. 補完した時刻より古い実データは捨てる
  (Point-LIO が時刻の巻き戻りとして扱うため)
"""
import array
from collections import deque
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Header

_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def rotation_rpy(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll) (ROS / static_transform_publisher と同じ順)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def cloud_to_array(msg):
    """PointCloud2 -> numpy の構造化配列 (全フィールドを保持)."""
    dtype = np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [_DTYPES[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    if msg.is_bigendian:
        dtype = dtype.newbyteorder('>')
    n_row = msg.width * msg.point_step
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.row_step)[:, :n_row]
    return np.frombuffer(buf.tobytes(), dtype=dtype)


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def to_stamp(t):
    sec = math.floor(t)
    return Time(sec=int(sec), nanosec=min(int(round((t - sec) * 1e9)), 999999999))


def so3_exp(w):
    """回転ベクトル -> 回転行列 (ロドリゲスの式)."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(th) * kx + (1.0 - math.cos(th)) * kx @ kx


class Attitude:
    """ジャイロを積分した姿勢の履歴 (基準時刻の姿勢を単位行列とする)."""

    def __init__(self, t0, r0=None):
        self.t = [t0]
        self.r = [np.eye(3) if r0 is None else r0]

    def add(self, t, gyro):
        """時刻 t までを, 前回から角速度 gyro 一定として積分する."""
        dt = t - self.t[-1]
        if dt > 0.0:
            self.t.append(t)
            self.r.append(self.r[-1] @ so3_exp(np.asarray(gyro) * dt))

    def at(self, ts):
        """時刻 ts (配列可) の姿勢. 直前の積分結果を使う (IMU の周期 4 ms 程度の精度)."""
        if np.isscalar(ts) and ts >= self.t[-1]:
            return self.r[-1]
        i = np.searchsorted(self.t, ts, side='right') - 1
        return np.asarray(self.r)[np.clip(i, 0, len(self.t) - 1)]

    def trim(self, t_keep):
        """時刻 t_keep より前の履歴を, その時刻の姿勢を求めるのに要る 1 つを残して捨てる."""
        k = int(np.searchsorted(self.t, t_keep, side='right')) - 1
        if k > 0:
            del self.t[:k]
            del self.r[:k]

    def rebase(self, t_ref):
        """基準を時刻 t_ref の姿勢に変え, それより前の履歴を捨てる."""
        r_ref_inv = self.at(t_ref).T
        k = max(int(np.searchsorted(self.t, t_ref, side='right')) - 1, 0)
        self.t = [t_ref] + self.t[k + 1:]
        self.r = [np.eye(3)] + [r_ref_inv @ r for r in self.r[k + 1:]]


def make_cloud(src, pts, frame_id):
    out = PointCloud2()
    out.header = src.header
    out.header.frame_id = frame_id
    out.fields = src.fields
    out.is_bigendian = src.is_bigendian
    out.point_step = src.point_step
    out.height = 1
    out.width = len(pts)
    out.row_step = out.width * src.point_step
    out.data = array.array('B', pts.tobytes())   # bytes を直接渡すと setter が遅い
    out.is_dense = True
    return out


class LidarBridge(Node):

    def __init__(self):
        super().__init__('lidar_bridge')
        p = self.declare_parameter
        cloud_in = p('cloud_in', '/unilidar/cloud').value
        imu_in = p('imu_in', '/unilidar/imu').value
        cloud_out = p('cloud_out', '/lidar').value
        obstacles_out = p('obstacles_out', '/lidar_obstacles').value
        clear_out = p('clear_out', '/lidar_clear').value
        imu_out = p('imu_out', '/imu').value
        self.lidar_frame = p('lidar_frame', 'lidar_link').value
        self.imu_frame = p('imu_frame', 'imu_link').value

        # base_link から見た LiDAR の取付位置 [m] と姿勢 [deg] (前に傾けたら pitch が正)
        t = [p('lidar_x', 0.2).value, p('lidar_y', 0.0).value, p('lidar_z', 0.25).value]
        rpy = [math.radians(p(f'lidar_{k}_deg', 0.0).value) for k in ('roll', 'pitch', 'yaw')]
        self.rot = rotation_rpy(*rpy)
        self.trans = np.array(t)

        # 以下はすべて base_link 座標
        self.ground_z = p('ground_z', -0.1).value                    # 地面の z [m]
        self.obstacle_min = p('obstacle_min_height', 0.15).value    # 地面からの高さ [m]
        self.obstacle_max = p('obstacle_max_height', 0.5).value
        self.self_min = np.array(p('self_box_min', [-0.4, -0.25, -0.2]).value, dtype=float)
        self.self_max = np.array(p('self_box_max', [0.4, 0.25, 0.4]).value, dtype=float)
        self.range_min = p('range_min', 0.3).value                  # LiDAR からの距離 [m]
        # 障害物とクリア用の点の最小距離 [m]. 負なら range_min と同じ. Point-LIO 用 (range_min) とは別に
        # 小さくして, 車体のすぐ近くの障害物も拾う (車体の点は self_box で除く)
        self.obs_range_min = p('obstacle_range_min', -1.0).value
        if self.obs_range_min < 0.0:
            self.obs_range_min = self.range_min
        self.ground_extend = p('ground_clear_extend', 0.1).value   # 地面の点をずらす距離 [m] (0 で無効)

        # 坂の判定 (ファイル先頭の説明)
        self.slope_filter = p('slope_filter', True).value
        self.slope_tan = math.tan(math.radians(p('slope_max_deg', 15.0).value))
        self.step_max = p('step_max', 0.06).value               # 隣のマスとの高さの差の上限 (段差) [m]
        self.slope_cell = p('slope_cell', 0.15).value            # マスの大きさ [m]
        self.slope_range = p('slope_range', 4.0).value           # 判定する範囲 (base_link から ±) [m]
        self.slope_min_pts = p('slope_min_points', 3).value
        traversable_out = p('traversable_out', '/lidar_slope').value   # 坂として外した点 (確認用)

        # IMU 加速度の 1 次ローパスのカットオフ [Hz] (0 以下で無効).
        # LiDAR の回転で車体が振動し, そのノイズを Point-LIO が積分して静止中も動いてしまうため
        self.acc_lpf_hz = p('imu_acc_lpf_hz', 0.0).value
        self.acc_filt = None        # フィルタ後の加速度
        self.imu_last_t = None      # 前回の IMU の時刻 [s]

        # センサの途切れの補完 (0 以下で無効)
        self.gap_timeout = p('gap_fill_timeout', 0.0).value     # これ以上届かなければ補完 [s]
        self.cloud_period = 0.1     # 点群の周期 [s] (実データから推定)
        self.imu_period = 0.004     # IMU の周期 [s] (実データから推定)
        self.imu_hist = deque(maxlen=1000)  # (時刻, 角速度, 加速度) 実際の IMU の直近 約 4 s
        self.cloud_last = None      # (時刻, 点の構造化配列, 元メッセージ) 最後に配信した実際の点群
        self.cloud_out_t = None     # 最後に配信した点群の時刻 [s] (補完分を含む)
        self.imu_out_t = None       # 最後に配信した IMU の時刻 [s] (補完分を含む)
        self.cloud_rx = None        # (センサ時刻, 受信時刻) 最後に受信した点群
        self.imu_rx = None          # (センサ時刻, 受信時刻) 最後に受信した IMU
        self.gap = None             # 点群の途切れ中の状態 (start_cloud_gap)
        self.imu_gap_t = None       # IMU の補完を始めたセンサ時刻

        self.pub = self.create_publisher(PointCloud2, cloud_out, 10)
        self.pub_obs = self.create_publisher(PointCloud2, obstacles_out, 10)
        self.pub_clear = self.create_publisher(PointCloud2, clear_out, 10)
        self.pub_slope = self.create_publisher(PointCloud2, traversable_out, 10)
        self.pub_imu = self.create_publisher(Imu, imu_out, 50)
        self.create_subscription(PointCloud2, cloud_in, self.on_cloud, 10)
        self.create_subscription(Imu, imu_in, self.on_imu, 50)
        if self.gap_timeout > 0.0:
            self.create_timer(0.02, self.on_gap_timer)

        self.get_logger().info(
            f'{cloud_in} -> {cloud_out}, {obstacles_out} / {imu_in} -> {imu_out}  '
            f'mount xyz={t} rpy[deg]={[round(math.degrees(a), 1) for a in rpy]}  '
            f'imu_acc_lpf_hz={self.acc_lpf_hz}  gap_fill_timeout={self.gap_timeout}  '
            f'slope_filter={self.slope_filter} (max {math.degrees(math.atan(self.slope_tan)):.0f}deg)')

    def on_imu(self, msg):
        t = to_sec(msg.header.stamp)
        if self.imu_rx is not None and 0.0 < t - self.imu_rx[0] < 0.1:
            self.imu_period += 0.01 * (t - self.imu_rx[0] - self.imu_period)
        self.imu_rx = (t, time.monotonic())
        msg.header.frame_id = self.imu_frame
        if self.acc_lpf_hz > 0.0:
            self.filter_acc(msg)
        w, a = msg.angular_velocity, msg.linear_acceleration
        gyro, acc = np.array([w.x, w.y, w.z]), np.array([a.x, a.y, a.z])
        self.imu_hist.append((t, gyro, acc))
        if self.imu_out_t is not None and t <= self.imu_out_t:
            return      # 補完済みの時刻
        if self.imu_gap_t is not None:
            self.get_logger().warn(f'imu: 再開. {t - self.imu_gap_t:.2f} s 分を補完した')
            self.imu_gap_t = None
        if self.gap is not None:
            # 点群の途切れ中: 角速度はそのまま, 加速度は重力だけにする
            self.gap['att'].add(t, gyro)
            a.x, a.y, a.z = (float(v) for v in self.gap_gravity(t))
        self.imu_out_t = t
        self.pub_imu.publish(msg)

    def gap_gravity(self, t):
        """点群の途切れ中, 時刻 t に静止していれば IMU が測るはずの加速度."""
        return self.gap['att'].at(t).T @ self.gap['g_ref']

    def on_gap_timer(self):
        """途切れているセンサの分を補完する."""
        now = time.monotonic()
        rx = [r for r in (self.cloud_rx, self.imu_rx) if r is not None]
        if not rx:
            return
        ref_t, ref_rx = max(rx, key=lambda r: r[1])
        sensor_now = ref_t + (now - ref_rx)     # 最後に届いたデータから推定したセンサ時刻

        if self.gap is None and self.cloud_last is not None \
                and now - self.cloud_rx[1] > self.gap_timeout:
            self.start_cloud_gap()

        if self.imu_out_t is not None and self.imu_hist \
                and now - self.imu_rx[1] > self.gap_timeout:
            if self.imu_gap_t is None:
                self.imu_gap_t = self.imu_out_t
                self.get_logger().warn(f'imu: {self.gap_timeout} s 以上届かないので静止として補完を開始')
            acc = np.mean([h[2] for h in list(self.imu_hist)[-50:]], axis=0)
            t = self.imu_out_t + self.imu_period
            while t <= sensor_now:
                if self.gap is not None:
                    self.gap['att'].add(t, np.zeros(3))
                    acc = self.gap_gravity(t)
                msg = Imu()
                msg.header = Header(stamp=to_stamp(t), frame_id=self.imu_frame)
                msg.orientation_covariance[0] = -1.0
                (msg.linear_acceleration.x, msg.linear_acceleration.y,
                 msg.linear_acceleration.z) = (float(v) for v in acc)
                self.pub_imu.publish(msg)
                self.imu_out_t = t
                t += self.imu_period

        if self.gap is not None:
            # 1 スキャン分の姿勢が IMU でそろった分だけ出す. 再開した実際の点群より
            # 新しい時刻にならないよう 1 周期分の余裕をとる
            t = self.cloud_out_t + self.cloud_period
            while t + self.gap['scan_len'] + self.cloud_period <= self.imu_out_t:
                self.pub.publish(self.gap_cloud(t))
                self.cloud_out_t = t
                t += self.cloud_period
            self.gap['att'].trim(self.cloud_out_t)

    def start_cloud_gap(self):
        """点群の途切れの補完を始める. 最後の点群を, 途切れ直前の姿勢を基準とした座標にする."""
        t_ref, pts, src = self.cloud_last
        hist = [h for h in self.imu_hist if h[0] >= t_ref - 1.0]
        if not hist:
            return
        att = Attitude(hist[0][0])
        for t, gyro, _ in hist[1:]:
            att.add(t, gyro)
        # 静止時に測る加速度 (重力の反力) を基準の座標で求める. 直前 1 s の加速度を
        # 姿勢の変化で回してそろえてから平均し, 手の揺れなどの並進の加速度を打ち消す
        before = [(t, acc) for t, _, acc in hist if t <= t_ref] or [(hist[0][0], hist[0][2])]
        r_ref_inv = att.at(t_ref).T
        g_ref = np.mean([r_ref_inv @ att.at(t) @ acc for t, acc in before], axis=0)
        att.rebase(t_ref)

        names = pts.dtype.names
        dt = pts['time'].astype(np.float64) if 'time' in names else np.zeros(len(pts))
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        xyz_ref = np.einsum('nij,nj->ni', att.at(t_ref + dt), xyz)  # 点ごとの姿勢で基準座標へ
        self.gap = {'t_ref': t_ref, 'att': att, 'g_ref': g_ref, 'pts': pts, 'src': src,
                    'dt': dt, 'xyz_ref': xyz_ref, 'scan_len': float(dt.max(initial=0.0))}
        self.get_logger().warn(
            f'cloud: {self.gap_timeout} s 以上届かないので, その場で回転しているとして補完を開始')

    def gap_cloud(self, t):
        """時刻 t からの 1 スキャン分の点群を, 途切れ直前の点群を回して作る."""
        g = self.gap
        xyz = np.einsum('nji,nj->ni', g['att'].at(t + g['dt']), g['xyz_ref'])  # R^T p
        pts = g['pts'].copy()
        pts['x'], pts['y'], pts['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        out = make_cloud(g['src'], pts, self.lidar_frame)
        out.header = Header(stamp=to_stamp(t), frame_id=self.lidar_frame)
        return out

    def filter_acc(self, msg):
        a = msg.linear_acceleration
        acc = np.array([a.x, a.y, a.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = None if self.imu_last_t is None else t - self.imu_last_t
        self.imu_last_t = t
        if self.acc_filt is None or dt is None or not 0.0 < dt < 0.1:
            # 初回・時刻の巻き戻り・長い途切れのあとはそのまま使ってやり直す
            self.acc_filt = acc
        else:
            tau = 1.0 / (2.0 * math.pi * self.acc_lpf_hz)
            self.acc_filt = self.acc_filt + dt / (tau + dt) * (acc - self.acc_filt)
        a.x, a.y, a.z = (float(v) for v in self.acc_filt)

    def on_cloud(self, msg):
        t = to_sec(msg.header.stamp)
        if self.cloud_rx is not None and 0.0 < t - self.cloud_rx[0] < 0.5:
            self.cloud_period += 0.1 * (t - self.cloud_rx[0] - self.cloud_period)
        self.cloud_rx = (t, time.monotonic())
        pts = cloud_to_array(msg)
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        finite = np.isfinite(xyz).all(axis=1)
        xyz[~finite] = 0.0
        r2 = np.einsum('ij,ij->i', xyz, xyz)
        valid = finite & (r2 >= self.range_min ** 2)
        near_ok = finite & (r2 >= self.obs_range_min ** 2)    # 障害物とクリア用

        base = xyz @ self.rot.T + self.trans        # LiDAR 座標 -> base_link 座標
        in_self = np.all((base > self.self_min) & (base < self.self_max), axis=1)
        keep = valid & ~in_self
        if self.cloud_out_t is None or t > self.cloud_out_t:   # 補完済みの時刻なら Point-LIO 用は捨てる
            if self.gap is not None:
                self.get_logger().warn(f'cloud: 再開. {t - self.gap["t_ref"]:.2f} s 分を補完した')
                self.gap = None
            out = make_cloud(msg, pts[keep], self.lidar_frame)
            self.cloud_last = (t, pts[keep], out)
            self.cloud_out_t = t
            self.pub.publish(out)

        h = base[:, 2] - self.ground_z              # 地面からの高さ
        near_keep = near_ok & ~in_self
        obs = near_keep & (h > self.obstacle_min) & (h < self.obstacle_max)
        floor = h <= self.obstacle_min              # 床 (クリア用に奥へずらす点)
        if self.slope_filter:
            slope = obs & self.traversable(base, h, near_keep & (h < self.obstacle_max))
            obs &= ~slope
            floor |= slope
            if self.pub_slope.get_subscription_count() > 0:
                self.pub_slope.publish(make_cloud(msg, pts[slope], self.lidar_frame))
        self.pub_obs.publish(make_cloud(msg, pts[obs], self.lidar_frame))
        if self.pub_clear.get_subscription_count() > 0:
            self.pub_clear.publish(self.clear_cloud(msg, pts, base, near_keep, floor))

    def traversable(self, base, h, cand):
        """各点が「床から坂を伝ってたどり着けるマス」にあるか (ファイル先頭の説明)."""
        c, r = self.slope_cell, self.slope_range
        w = int(math.ceil(2.0 * r / c))
        sel = cand & (np.abs(base[:, 0]) < r) & (np.abs(base[:, 1]) < r)
        out = np.zeros(len(h), dtype=bool)
        if not sel.any():
            return out
        ix = np.clip(((base[sel, 0] + r) / c).astype(int), 0, w - 1)
        iy = np.clip(((base[sel, 1] + r) / c).astype(int), 0, w - 1)
        cell = ix * w + iy
        hs = h[sel]
        cnt = np.bincount(cell, minlength=w * w)
        div = np.maximum(cnt, 1)
        z = np.bincount(cell, weights=hs, minlength=w * w) / div               # マスの高さ
        x = np.bincount(cell, weights=base[sel, 0], minlength=w * w) / div     # 点の重心
        y = np.bincount(cell, weights=base[sel, 1], minlength=w * w) / div
        zmin = np.full(w * w, np.inf)
        zmax = np.full(w * w, -np.inf)
        np.minimum.at(zmin, cell, hs)
        np.maximum.at(zmax, cell, hs)
        thin = (cnt > 0) & (zmax - zmin <= c * math.sqrt(2.0) * self.slope_tan + self.step_max)
        ground = thin & (z < self.obstacle_min)
        dense = thin & (cnt >= self.slope_min_pts)
        z, x, y, ground, dense = (a.reshape(w, w) for a in (z, x, y, ground, dense))

        def shift(a, dx, dy, fill):
            """b[i, j] = a[i + dx, j + dy] (範囲外は fill)."""
            b = np.full_like(a, fill)
            b[max(0, -dx):w - max(0, dx), max(0, -dy):w - max(0, dy)] = \
                a[max(0, dx):w - max(0, -dx), max(0, dy):w - max(0, -dy)]
            return b

        # 周り 5 x 5 マスの (x, y, z) に平面 z = a x + b y + d を最小二乗で当てはめる
        usable = (ground | dense).astype(float)
        names = ('n', 'x', 'y', 'z', 'xx', 'xy', 'yy', 'xz', 'yz')
        x0, y0 = x * usable, y * usable     # 使わないマスは 0 にする (重み 0)
        terms = dict(zip(names, (usable, x0, y0, z * usable, x0 * x, x0 * y, y0 * y, x0 * z, y0 * z)))
        sums = {k: np.zeros((w, w)) for k in names}
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                for k in names:
                    sums[k] += shift(terms[k], dx, dy, 0.0)
        n_ = np.maximum(sums['n'], 1.0)
        mx, my, mz = sums['x'] / n_, sums['y'] / n_, sums['z'] / n_
        sxx = sums['xx'] / n_ - mx * mx
        sxy = sums['xy'] / n_ - mx * my
        syy = sums['yy'] / n_ - my * my
        sxz = sums['xz'] / n_ - mx * mz
        syz = sums['yz'] / n_ - my * mz
        det = sxx * syy - sxy * sxy
        ok_fit = (sums['n'] >= 4) & (det > (0.25 * c * c) ** 2)   # 点が一直線に並ぶときは傾きを決めない
        safe = np.where(ok_fit, det, 1.0)
        ga = (sxz * syy - syz * sxy) / safe
        gb = (syz * sxx - sxz * sxy) / safe
        gentle = ok_fit & (np.hypot(ga, gb) <= self.slope_tan)
        passable = ground | (dense & gentle)

        # 床から, 隣との段差が step_max 以下で通れるマスを広げる
        links = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx or dy:
                    ok = passable & shift(passable, dx, dy, False) & \
                        (np.abs(z - shift(z, dx, dy, np.inf)) <= self.step_max)
                    links.append((dx, dy, ok))
        reach = dense & (z < 0.5 * self.obstacle_min)   # 出発点
        for _ in range(w * 2):
            grown = reach.copy()
            for dx, dy, ok in links:
                grown |= ok & shift(reach, dx, dy, False)
            if np.array_equal(grown, reach):
                break
            reach = grown
        out[np.flatnonzero(sel)] = reach.ravel()[cell]
        return out

    def clear_cloud(self, msg, pts, base, mask, floor):
        """Nav2 のクリア用の点群. 床 (坂を含む) の点だけ LiDAR から水平方向に ground_clear_extend 奥へずらす."""
        sel = pts[mask].copy()
        if self.ground_extend > 0.0 and len(sel):
            b = base[mask].copy()
            ground = floor[mask]
            d = b[ground, :2] - self.trans[:2]
            n = np.linalg.norm(d, axis=1, keepdims=True)
            b[ground, :2] += d / np.maximum(n, 1e-6) * self.ground_extend
            xyz = (b - self.trans) @ self.rot          # base_link 座標 -> LiDAR 座標
            sel['x'], sel['y'], sel['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        return make_cloud(msg, sel, self.lidar_frame)


def main():
    rclpy.init()
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
