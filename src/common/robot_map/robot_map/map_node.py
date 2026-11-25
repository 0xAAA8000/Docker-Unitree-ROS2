"""Point-LIO の 3D 点群を積み上げて 2D 地図 (nav_msgs/OccupancyGrid) を配信する.

  ros2 run robot_map map                                   # 走りながら地図を作る
  ros2 run robot_map map --ros-args -p pcd_file:=scans.pcd # 保存した地図を読み込んでから続ける

購読: /cloud_registered (Point-LIO の 3D 点群. map 座標, 毎スキャン), TF (map -> base_link)
配信: /map (transient_local なので, 後から起動した RViz や Web も最新の地図を受け取れる)

点の高さは base_link 座標に直してから判定する (obstacle_detector と同じ考え方).
ground_z, obstacle_min_height, obstacle_max_height は obstacle_detector と同じ意味 (map.launch.py が robot_config から同じ値を渡す).
"""
import array
import math

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
        self.create_timer(period, self.publish)
        self.get_logger().info(
            f'{self.cloud_topic if live else "(購読なし)"} -> {self.map_topic} '
            f'(解像度 {self.grid.res} m, 床 {self.ground_z:+.2f} m, '
            f'障害物 {self.obs_min:.2f}..{self.obs_max:.2f} m)')

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
        near = base[:, 0] ** 2 + base[:, 1] ** 2 <= self.max_range ** 2
        base = base[near]
        world = _apply(to_map, xyz[near])
        floor, obs = classify(base[:, 2], self.ground_z, self.floor_tol, self.obs_min,
                              self.obs_max)
        self.grid.add(world[floor, :2], world[obs, :2])

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


def main():
    # Ctrl-C は KeyboardInterrupt だけで受ける (rclpy の SIGINT ハンドラは片付けの途中で割り込む)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
