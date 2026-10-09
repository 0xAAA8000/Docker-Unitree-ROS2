"""実機の IMU・点群・Point-LIO の出力を 5 秒ごとに集計して表示する (静止中のドリフトの切り分け用).

  ros2 run lidar_bridge sensor_check      # real.launch.py と LiDAR ドライバを起動した状態で

見るところ
  * IMU    : 周期の乱れ (dt の最小/最大, 逆行・抜け), 加速度の大きさ (静止なら 9.8 前後), ジャイロの平均
  * 点群   : 点数, 点ごとの time の範囲, blind (0.5 m) 未満の点の割合,
             ヘッダ時刻と受信時刻の差 / 最新 IMU の時刻との差 (時計がずれていると大きくなる)
  * Point-LIO : 5 秒間の移動量と推定速度 (静止なら 0 付近のはず)
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from lidar_bridge.lidar_bridge_node import cloud_to_array

PERIOD = 5.0
BLIND = 0.5


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')
        p = self.declare_parameter
        self.imu_topic = p('imu_topic', '/unilidar/imu').value
        self.cloud_topics = [p('cloud_raw_topic', '/unilidar/cloud').value,
                             p('cloud_topic', '/lidar').value]
        self.odom_topic = p('odom_topic', '/aft_mapped_to_init').value

        self.imu_last_stamp = None
        self.reset()
        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
        for topic in self.cloud_topics:
            self.create_subscription(PointCloud2, topic,
                                     lambda m, t=topic: self.on_cloud(t, m), qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 100)
        self.create_timer(PERIOD, self.report)
        self.get_logger().info(f'{PERIOD:.0f} 秒ごとに集計します (静止させて見てください)')

    def reset(self):
        self.imu_dt, self.acc, self.gyr = [], [], []
        self.clouds = {t: {'n': [], 'tmin': [], 'tmax': [], 'blind': [], 'delay': [], 'vs_imu': []}
                       for t in self.cloud_topics}
        self.odom = []

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        t = stamp_sec(msg.header.stamp)
        if self.imu_last_stamp is not None:
            self.imu_dt.append(t - self.imu_last_stamp)
        self.imu_last_stamp = t
        a, w = msg.linear_acceleration, msg.angular_velocity
        self.acc.append((a.x, a.y, a.z))
        self.gyr.append((w.x, w.y, w.z))

    def on_cloud(self, topic, msg):
        c = self.clouds[topic]
        t = stamp_sec(msg.header.stamp)
        c['delay'].append(self.now_sec() - t)
        if self.imu_last_stamp is not None:
            c['vs_imu'].append(t - self.imu_last_stamp)
        pts = cloud_to_array(msg)
        c['n'].append(len(pts))
        if len(pts) == 0:
            return
        if 'time' in pts.dtype.names:
            c['tmin'].append(float(pts['time'].min()))
            c['tmax'].append(float(pts['time'].max()))
        r2 = pts['x'].astype(float) ** 2 + pts['y'].astype(float) ** 2 + pts['z'].astype(float) ** 2
        c['blind'].append(float(np.mean(r2 < BLIND ** 2)))

    def on_odom(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.odom.append((p.x, p.y, p.z, v.x, v.y, v.z))

    def report(self):
        lines = ['---------------- sensor_check ----------------']
        lines += self.report_imu()
        for topic in self.cloud_topics:
            lines += self.report_cloud(topic)
        lines += self.report_odom()
        self.get_logger().info('\n'.join(lines))
        self.reset()

    def report_imu(self):
        if not self.acc:
            return [f'IMU {self.imu_topic}: 受信なし']
        acc, gyr = np.array(self.acc), np.array(self.gyr)
        out = [f'IMU {self.imu_topic}: {len(acc) / PERIOD:.1f} Hz']
        if self.imu_dt:
            dt = np.array(self.imu_dt) * 1e3
            out.append(f'  dt[ms] 平均 {dt.mean():.2f} 最小 {dt.min():.2f} 最大 {dt.max():.2f} '
                       f'標準偏差 {dt.std():.2f}  逆行/同時刻 {int(np.sum(dt <= 0))}  '
                       f'20ms超の抜け {int(np.sum(dt > 20))}')
        norm = np.linalg.norm(acc, axis=1)
        out.append(f'  acc 平均 [{", ".join(f"{v:.3f}" for v in acc.mean(axis=0))}]  '
                   f'|acc| 平均 {norm.mean():.3f} 標準偏差 {norm.std():.3f}  '
                   f'軸ごとの標準偏差 [{", ".join(f"{v:.3f}" for v in acc.std(axis=0))}]')
        out.append(f'  gyro 平均[deg/s] [{", ".join(f"{math.degrees(v):.3f}" for v in gyr.mean(axis=0))}]  '
                   f'標準偏差 [{", ".join(f"{math.degrees(v):.3f}" for v in gyr.std(axis=0))}]')
        return out

    def report_cloud(self, topic):
        c = self.clouds[topic]
        if not c['n']:
            return [f'点群 {topic}: 受信なし']
        out = [f'点群 {topic}: {len(c["n"]) / PERIOD:.1f} Hz  点数 平均 {np.mean(c["n"]):.0f} '
               f'最小 {np.min(c["n"])}  blind未満 {100 * np.mean(c["blind"]) if c["blind"] else 0:.1f} %']
        if c['tmin']:
            out.append(f'  点の time[s] 最小 {np.min(c["tmin"]):.4f} 最大 {np.max(c["tmax"]):.4f}')
        out.append(f'  受信時刻 - ヘッダ時刻 [ms] 平均 {1e3 * np.mean(c["delay"]):.1f} '
                   f'最大 {1e3 * np.max(c["delay"]):.1f}')
        if c['vs_imu']:
            out.append(f'  ヘッダ時刻 - 最新IMU時刻 [ms] 平均 {1e3 * np.mean(c["vs_imu"]):.1f} '
                       f'最小 {1e3 * np.min(c["vs_imu"]):.1f} 最大 {1e3 * np.max(c["vs_imu"]):.1f}')
        return out

    def report_odom(self):
        if len(self.odom) < 2:
            return [f'Point-LIO {self.odom_topic}: 受信なし']
        o = np.array(self.odom)
        d = o[-1, :3] - o[0, :3]
        v = o[:, 3:].mean(axis=0)
        return [f'Point-LIO {self.odom_topic}: 位置 [{", ".join(f"{x:.3f}" for x in o[-1, :3])}]',
                f'  {PERIOD:.0f}秒の移動 [{", ".join(f"{x:.3f}" for x in d)}] '
                f'({np.linalg.norm(d) / PERIOD:.3f} m/s)  '
                f'推定速度 平均 [{", ".join(f"{x:.3f}" for x in v)}]']


def main():
    rclpy.init()
    node = SensorCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
