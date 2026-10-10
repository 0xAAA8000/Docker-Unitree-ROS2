"""点群と IMU の欠落を 1 件ずつ表示し, どこでデータが落ちているかを切り分ける.

  ros2 run lidar_bridge watch_gaps
  ros2 run lidar_bridge watch_gaps --ros-args -p cloud:=/lidar -p imu:=/imu

check_inputs は 5 秒ごとの平均なので欠落した瞬間がわからない. こちらは起きるたびに表示する.
  * 欠落: stamp (LiDAR が付けた時刻) の間隔が普段の 1.5 倍 (IMU は 3 倍) 以上
  * 受信遅れ: 受信時刻 (この PC の時計) の間隔は開いたが stamp の間隔は普通
              -> データはあったが届くのが遅れた (CPU の詰まりなど)
  * 点数不足: 点数が普段の 7 割未満 (点群の一部が欠けた)
各行の t は最初の stamp からの秒数. 点群と IMU の欠落が同じ t で起きていれば,
ドライバより手前 (USB / シリアル) でデータが落ちている.
load は欠落時の CPU 負荷 (1 分平均の load average) で, 負荷と欠落の関係を見るのに使う.
"""
from collections import deque
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2


def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Stream:
    """1 つのトピックの stamp と受信時刻の間隔を見て欠落を数える."""

    def __init__(self, name, gap_ratio):
        self.name = name
        self.gap_ratio = gap_ratio
        self.dt = deque(maxlen=100)     # 欠落していないときの stamp の間隔
        self.prev = None                # (stamp, 受信時刻)
        self.received = 0
        self.missing = 0
        self.late = 0

    def add(self, t, r):
        """欠落なら ('gap', 間隔, 欠落数), 受信遅れなら ('late', 受信間隔, 0) を返す."""
        self.received += 1
        prev, self.prev = self.prev, (t, r)
        if prev is None:
            return None
        dt, rdt = t - prev[0], r - prev[1]
        if len(self.dt) < 20:           # 普段の間隔がわかるまで判定しない
            self.dt.append(dt)
            return None
        med = float(np.median(self.dt))
        if dt > self.gap_ratio * med:
            n = max(1, round(dt / med) - 1)
            self.missing += n
            return 'gap', dt, n
        self.dt.append(dt)
        if rdt > max(3.0 * med, 0.05):
            self.late += 1
            return 'late', rdt, 0
        return None


class WatchGaps(Node):

    def __init__(self):
        super().__init__('watch_gaps')
        cloud = self.declare_parameter('cloud', '/unilidar/cloud').value
        imu = self.declare_parameter('imu', '/unilidar/imu').value
        period = self.declare_parameter('period', 10.0).value
        self.cloud = Stream('点群', 1.5)
        self.imu = Stream('IMU', 3.0)
        self.points = deque(maxlen=50)
        self.few = 0
        self.t0 = None
        # lidar_bridge と同じ QoS (RELIABLE). 受信側の取りこぼしを起こしにくくして,
        # ドライバが出していないものだけを数える
        self.create_subscription(PointCloud2, cloud, self.on_cloud, 50)
        self.create_subscription(Imu, imu, self.on_imu, 500)
        self.create_timer(period, self.report)
        self.get_logger().info(f'cloud={cloud} imu={imu}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rel(self, t):
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def show(self, stream, t, res):
        load = os.getloadavg()[0]
        kind, dt, n = res
        if kind == 'gap':
            self.get_logger().warn(
                f't={self.rel(t):8.2f}  {stream.name} 欠落 {n} 個 (間隔 {dt:.3f} s)  load={load:.1f}')
        else:
            self.get_logger().warn(
                f't={self.rel(t):8.2f}  {stream.name} 受信遅れ (受信間隔 {dt:.3f} s)  load={load:.1f}')

    def on_imu(self, msg):
        t = sec(msg.header.stamp)
        self.rel(t)
        res = self.imu.add(t, self.now())
        if res:
            self.show(self.imu, t, res)

    def on_cloud(self, msg):
        t = sec(msg.header.stamp)
        self.rel(t)
        res = self.cloud.add(t, self.now())
        if res:
            self.show(self.cloud, t, res)
        n = msg.width * msg.height
        if len(self.points) >= 20 and n < 0.7 * float(np.median(self.points)):
            self.few += 1
            self.get_logger().warn(
                f't={self.rel(t):8.2f}  点群 点数不足 {n} 点 (普段 {np.median(self.points):.0f} 点)')
        self.points.append(n)

    def report(self):
        c, i = self.cloud, self.imu
        load = os.getloadavg()
        self.get_logger().info(
            f'[累計] 点群 受信 {c.received} 欠落 {c.missing} 遅れ {c.late} 点数不足 {self.few} | '
            f'IMU 受信 {i.received} 欠落 {i.missing} 遅れ {i.late} | '
            f'load {load[0]:.1f} {load[1]:.1f} (CPU {os.cpu_count()} 個)')


def main():
    rclpy.init()
    node = WatchGaps()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
