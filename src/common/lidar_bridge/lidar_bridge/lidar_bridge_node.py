"""Unitree L1/L2 ドライバのトピックを, sim と同じ名前・形で Point-LIO と Nav2 に配信する.

  /unilidar/cloud --(自車体・近すぎる点を除去)--> /lidar            (Point-LIO, Nav2 のクリア用)
                  --(地面からの高さで抽出)------> /lidar_obstacles  (Nav2 の障害物)
  /unilidar/imu   --(frame_id だけ付け替え)-----> /imu              (Point-LIO)

sim (sim_gazebo/filter_invalid_points.py) との違い
  * 実機の点群には time フィールドがあり, 無効点も来ないので time の付加はしない
  * LiDAR を前に傾けて付けているので, 高さと自車体の判定は LiDAR 座標ではなく
    base_link 座標 (取付位置・姿勢で変換) で行う. 配信する点の座標は LiDAR 座標のまま
  * frame_id を lidar_link / imu_link に付け替える. ドライバが出す
    unilidar_imu_initial -> unilidar_imu の TF (IMU の姿勢) とぶつからないようにするため
"""
import array
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2

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

        # IMU 加速度の 1 次ローパスのカットオフ [Hz] (0 以下で無効).
        # LiDAR の回転で車体が振動し, そのノイズを Point-LIO が積分して静止中も動いてしまうため
        self.acc_lpf_hz = p('imu_acc_lpf_hz', 0.0).value
        self.acc_filt = None        # フィルタ後の加速度
        self.imu_last_t = None      # 前回の IMU の時刻 [s]

        self.pub = self.create_publisher(PointCloud2, cloud_out, 10)
        self.pub_obs = self.create_publisher(PointCloud2, obstacles_out, 10)
        self.pub_imu = self.create_publisher(Imu, imu_out, 50)
        self.create_subscription(PointCloud2, cloud_in, self.on_cloud, 10)
        self.create_subscription(Imu, imu_in, self.on_imu, 50)

        self.get_logger().info(
            f'{cloud_in} -> {cloud_out}, {obstacles_out} / {imu_in} -> {imu_out}  '
            f'mount xyz={t} rpy[deg]={[round(math.degrees(a), 1) for a in rpy]}  '
            f'imu_acc_lpf_hz={self.acc_lpf_hz}')

    def on_imu(self, msg):
        msg.header.frame_id = self.imu_frame
        if self.acc_lpf_hz > 0.0:
            self.filter_acc(msg)
        self.pub_imu.publish(msg)

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
        pts = cloud_to_array(msg)
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        valid = np.isfinite(xyz).all(axis=1)
        valid &= np.einsum('ij,ij->i', xyz, xyz) >= self.range_min ** 2

        base = xyz @ self.rot.T + self.trans        # LiDAR 座標 -> base_link 座標
        in_self = np.all((base > self.self_min) & (base < self.self_max), axis=1)
        keep = valid & ~in_self
        self.pub.publish(make_cloud(msg, pts[keep], self.lidar_frame))

        h = base[:, 2] - self.ground_z              # 地面からの高さ
        obs = keep & (h > self.obstacle_min) & (h < self.obstacle_max)
        self.pub_obs.publish(make_cloud(msg, pts[obs], self.lidar_frame))


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
