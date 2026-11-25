"""Point-LIO の 3D 点群を積み上げて 2D 地図 (nav_msgs/OccupancyGrid) を配信する.

  ros2 run robot_map map                                   # 走りながら地図を作る
  ros2 run robot_map map --ros-args -p pcd_file:=scans.pcd # 保存した地図を読み込んでから続ける

購読: /cloud_registered (Point-LIO の 3D 点群. map 座標, 毎スキャン), TF (map -> base_link)
配信: /map (transient_local なので, 後から起動した RViz や Web も最新の地図を受け取れる)

3D の地図 (ボクセル地図)
  同じ点群を voxel_size (Point-LIO の地図の粗さ filter_size_map と同じ値を robot_config が渡す) の
  ボクセルに貯める. 同じ場所を何度見ても 1 つにまとめるのでメモリは増えにくい.
  終了時 (Ctrl-C) と save_period 秒ごとに, save_min_hits 回以上点が入ったボクセルの中心を
  PCD に書く (save_file. 空なら save_dir/<起動日時>_map.pcd). 地図の保存は Point-LIO ではなくここで行う
  (Point-LIO の保存は全スキャンをまとめずにメモリに貯めるため止めてある)

3D 地図の配信 (スマホの 3D 表示用)
  ボクセルの色 (床・坂・障害物) は obstacle_detector の /lidar_labeled (点ごとの分類の値) を
  TF で map 座標に直し, 同じマスのボクセルに付ける (voxels.LabelMap). 形は Point-LIO, 色は obstacle_detector.
  車両から見えている範囲 (直近 visible_window 秒の点が届いた水平距離の visible_percentile % 点.
  上限 voxel_max_range) の中のボクセルを, voxel_period 秒ごとに voxel_topic に配信する
  (x y z = ボクセルの中心, label = 分類. 購読者がいて, 中身が変わったときだけ)

点の高さは base_link 座標に直してから判定する (obstacle_detector と同じ考え方).
ground_z, obstacle_min_height, obstacle_max_height は obstacle_detector と同じ意味 (map.launch.py が robot_config から同じ値を渡す).
"""
import array
import math
import os
import time

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros

from robot_map.grid import classify, cloud_xyz, estimate_ground, Grid, read_pcd_xyz
from robot_map.voxels import LabelMap, unpack, VoxelMap
from sensor_msgs.msg import PointField


def _rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _apply(t, xyz):
    """TransformStamped を (N, 3) の点に適用する."""
    r = _rotation(t.transform.rotation)
    p = t.transform.translation
    return xyz @ r.T + np.array([p.x, p.y, p.z])


class MapNode(Node):

    def __init__(self):
        super().__init__('robot_map')
        p = self.declare_parameter
        self.cloud_topic = p('cloud_topic', '/cloud_registered').value
        self.map_topic = p('map_topic', '/map').value
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.ground_z = p('ground_z', -0.10).value
        self.floor_tol = p('floor_tolerance', 0.05).value
        self.obs_min = p('obstacle_min_height', 0.15).value
        self.obs_max = p('obstacle_max_height', 0.50).value
        self.max_range = p('max_range', 15.0).value
        period = p('publish_period', 1.0).value
        # ボクセル地図
        self.voxels = VoxelMap(p('voxel_size', 0.1).value)
        self.voxel_range = p('voxel_max_range', 15.0).value     # 車体からこれより遠い点は貯めない [m]
        save_file = p('save_file', '').value
        save_dir = os.path.expanduser(p('save_dir', '~/robot_maps').value)
        self.save_min_hits = p('save_min_hits', 2).value
        save_period = p('save_period', 60.0).value
        self.save_file = save_file or os.path.join(
            save_dir, time.strftime('%Y%m%d_%H%M%S') + '_map.pcd')
        self.saved_version = 0
        # 3D 地図の配信
        self.labels = LabelMap(self.voxels.size, p('obstacle_min_hits', 2).value)
        labeled_topic = p('labeled_topic', '/lidar_labeled').value
        self.voxel_pub = self.create_publisher(PointCloud2, p('voxel_topic', '/voxel_map').value, 1)
        self.visible_pct = p('visible_percentile', 95.0).value
        self.visible_window = p('visible_window', 1.0).value
        self.reach = []              # (時刻, 点が届いた水平距離) 直近 visible_window 秒分
        self.voxel_sent = None       # 前に配信した中身の目印
        pcd_file = p('pcd_file', '').value
        pcd_ground = p('pcd_ground_z', float('nan')).value
        live = p('live', True).value

        self.grid = Grid(p('resolution', 0.05).value, p('chunk', 10.0).value,
                         p('occupied_min_hits', 3).value, p('occupied_ratio', 0.5).value)
        self.published = -1
        self.tf_warned = False

        self.tf_buffer = tf2_ros.Buffer()
        # 点群の処理中も TF を受け取れるよう, TF は専用スレッドで受ける
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, self.map_topic, qos)

        if pcd_file:
            self.load_pcd(pcd_file, pcd_ground)
        if live:
            self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 5)
            self.create_subscription(PointCloud2, labeled_topic, self.on_labeled, 5)
            self.create_timer(p('voxel_period', 1.0).value, self.publish_voxels)
        self.create_timer(period, self.publish)
        self.create_timer(1.0, self.voxels.merge)
        if save_period > 0:
            self.create_timer(save_period, self.save)
        self.get_logger().info(
            f'{self.cloud_topic if live else "(購読なし)"} -> {self.map_topic} '
            f'(解像度 {self.grid.res} m, 床 {self.ground_z:+.2f} m, '
            f'障害物 {self.obs_min:.2f}..{self.obs_max:.2f} m. ボクセル {self.voxels.size} m, '
            f'保存先 {self.save_file})')

    def load_pcd(self, path, ground):
        try:
            xyz = read_pcd_xyz(path)
        except (OSError, ValueError, KeyError) as e:
            self.get_logger().error(f'PCD を読めない: {e}')
            return
        if not math.isfinite(ground):
            ground = estimate_ground(xyz[:, 2])
        floor, obs = classify(xyz[:, 2], ground, self.floor_tol, self.obs_min, self.obs_max)
        self.grid.add(xyz[floor, :2], xyz[obs, :2])
        # 保存済みの地図の点は, 何度も見た点として扱う (保存の基準 save_min_hits を満たす)
        self.voxels.add(xyz, hits=max(1, self.save_min_hits))
        self.voxels.merge()     # 保存先が別のファイルでも, 最初の保存で読み込んだ分も書く
        self.get_logger().info(
            f'{path}: {len(xyz)} 点を読み込み (床の高さ {ground:+.2f} m, 地図 {self.grid.shape[1]}'
            f' x {self.grid.shape[0]} セル)')

    def _lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(target, source, stamp,
                                                   timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return self.tf_buffer.lookup_transform(target, source, Time())

    def on_cloud(self, msg):
        frame = msg.header.frame_id
        stamp = Time.from_msg(msg.header.stamp)
        try:
            to_base = self._lookup(self.base_frame, frame, stamp)
            to_map = self._lookup(self.map_frame, frame, stamp)
        except tf2_ros.TransformException as e:
            if not self.tf_warned:
                self.get_logger().warn(f'TF が取れないので点群を捨てる: {e}')
                self.tf_warned = True
            return
        try:
            xyz = cloud_xyz([(f.name, f.offset, f.datatype) for f in msg.fields],
                            msg.point_step, msg.data, msg.is_bigendian)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        if not len(xyz):
            return
        base = _apply(to_base, xyz)
        dist2 = base[:, 0] ** 2 + base[:, 1] ** 2
        world_all = _apply(to_map, xyz)
        self.voxels.add(world_all[dist2 <= self.voxel_range ** 2])
        now = time.monotonic()
        self.reach.append((now, math.sqrt(float(np.percentile(dist2, self.visible_pct)))))
        self.reach = [r for r in self.reach if now - r[0] <= self.visible_window]
        near = dist2 <= self.max_range ** 2
        base = base[near]
        world = world_all[near]
        floor, obs = classify(base[:, 2], self.ground_z, self.floor_tol, self.obs_min,
                              self.obs_max)
        self.grid.add(world[floor, :2], world[obs, :2])

    def on_labeled(self, msg):
        """obstacle_detector の分類の値を, 同じマスのボクセルの色にする."""
        try:
            to_map = self._lookup(self.map_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except tf2_ros.TransformException:
            return
        names = {f.name: f for f in msg.fields}
        if not all(k in names for k in ('x', 'y', 'z', 'label')) or msg.width * msg.height == 0:
            return
        dtype = np.dtype({'names': ['x', 'y', 'z', 'label'],
                          'formats': ['<f4', '<f4', '<f4', 'u1'],
                          'offsets': [names[k].offset for k in ('x', 'y', 'z', 'label')],
                          'itemsize': msg.point_step})
        pts = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        self.labels.add(_apply(to_map, xyz), pts['label'])

    def visible_radius(self):
        """車両から見えている範囲の半径 [m]. 点群がまだ無ければ 0."""
        if not self.reach:
            return 0.0
        return min(max(r for _, r in self.reach), self.voxel_range)

    def publish_voxels(self):
        """見えている範囲のボクセル (中心と分類) を配信する."""
        if self.voxel_pub.get_subscription_count() == 0:
            return
        self.voxels.merge()
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return
        cx, cy = t.transform.translation.x, t.transform.translation.y
        radius = self.visible_radius()
        size = self.voxels.size
        mark = (self.voxels.version, self.labels.version, round(radius / size),
                round(cx / size), round(cy / size))
        if mark == self.voxel_sent:
            return
        keys = self.voxels.keys[self.voxels.hits >= self.save_min_hits]
        centers = (unpack(keys) + 0.5) * size
        near = (centers[:, 0] - cx) ** 2 + (centers[:, 1] - cy) ** 2 <= radius ** 2
        keys, centers = keys[near], centers[near]
        self.voxel_pub.publish(voxel_cloud(self.map_frame, self.get_clock().now().to_msg(),
                                           centers, self.labels.lookup(keys)))
        self.voxel_sent = mark

    def save(self):
        """ボクセル地図を PCD に書く (変わっていなければ書かない)."""
        self.voxels.merge()
        if self.voxels.version == self.saved_version or not len(self.voxels):
            return
        try:
            n = self.voxels.save_pcd(self.save_file, self.save_min_hits)
        except OSError as e:
            self.get_logger().error(f'地図を保存できない: {e}', throttle_duration_sec=60.0)
            return
        self.saved_version = self.voxels.version
        self.get_logger().info(f'地図を保存した: {self.save_file} ({n} 点)',
                               throttle_duration_sec=300.0)

    def publish(self):
        if self.grid.version == self.published or self.grid.ix0 is None:
            return
        occ = self.grid.occupancy()
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.grid.res
        msg.info.height, msg.info.width = occ.shape
        msg.info.origin.position.x, msg.info.origin.position.y = self.grid.origin()
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', occ.tobytes())
        self.pub.publish(msg)
        self.published = self.grid.version


def voxel_cloud(frame, stamp, centers, labels):
    """ボクセルの中心 x y z (float32) と分類 label (uint8) の点群."""
    pts = np.zeros(len(centers), np.dtype({'names': ['x', 'y', 'z', 'label'],
                                           'formats': ['<f4', '<f4', '<f4', 'u1'],
                                           'offsets': [0, 4, 8, 12], 'itemsize': 16}))
    pts['x'], pts['y'], pts['z'] = centers[:, 0], centers[:, 1], centers[:, 2]
    pts['label'] = labels
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.fields = [PointField(name=n, offset=o, datatype=d, count=1)
                  for n, o, d in (('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('label', 12, 2))]
    msg.point_step = 16
    msg.height = 1
    msg.width = len(pts)
    msg.row_step = 16 * len(pts)
    msg.data = array.array('B', pts.tobytes())
    msg.is_dense = True
    return msg


def main():
    # Ctrl-C は KeyboardInterrupt だけで受ける (rclpy の SIGINT ハンドラは片付けの途中で割り込む)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()     # 終了時に地図を保存する (launcher はこのファイルを地図として扱う)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
