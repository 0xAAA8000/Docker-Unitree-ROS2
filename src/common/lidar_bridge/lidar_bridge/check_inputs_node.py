"""Point-LIO に入る点群と IMU を静止状態で調べ, ドリフトの原因を切り分ける.

  ros2 run lidar_bridge check_inputs --ros-args -p cloud:=/unilidar/cloud -p imu:=/unilidar/imu

車両を止めたまま 10 秒ほど流し, 一定周期で次を表示する.
  * IMU: 周期, stamp の逆行, 加速度の平均とノルム (静止なら 9.81 付近), ジャイロの平均 (バイアス)
  * 点群: 周期, 点数, time フィールドの範囲 [s] (Point-LIO は stamp + time をスキャン終了時刻にする)
  * stamp のずれ: 点群のスキャン終了時刻 - 受信時点で最新の IMU stamp.
    Point-LIO は IMU がスキャン終了時刻に追いつくまで待つので, ここが大きく外れていると
    IMU の積分区間と点群の時刻が合わず, 静止していても位置が流れる.
  * 受信遅れ: 受信時刻 (このPCの時計) - stamp
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from lidar_bridge.lidar_bridge_node import cloud_to_array


def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stats(values):
    a = np.asarray(values, dtype=float)
    if a.size == 0:
        return 'n/a'
    return f'mean={a.mean():+.4f} min={a.min():+.4f} max={a.max():+.4f}'


class CheckInputs(Node):

    def __init__(self):
        super().__init__('check_lio_inputs')
        cloud = self.declare_parameter('cloud', '/unilidar/cloud').value
        imu = self.declare_parameter('imu', '/unilidar/imu').value
        period = self.declare_parameter('period', 5.0).value
        self.reset()
        self.last_imu_stamp = None
        self.create_subscription(Imu, imu, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(period, self.report)
        self.get_logger().info(f'cloud={cloud} imu={imu}  車両を静止させたまま待つこと')

    def reset(self):
        self.acc = []
        self.gyr = []
        self.imu_dt = []
        self.imu_back = 0
        self.imu_delay = []
        self.cloud_dt = []
        self.cloud_n = []
        self.t_min = []
        self.t_max = []
        self.offset = []
        self.cloud_delay = []
        self.prev_imu = None
        self.prev_cloud = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        t = sec(msg.header.stamp)
        if self.prev_imu is not None:
            if t <= self.prev_imu:
                self.imu_back += 1
            self.imu_dt.append(t - self.prev_imu)
        self.prev_imu = t
        self.last_imu_stamp = t
        a = msg.linear_acceleration
        w = msg.angular_velocity
        self.acc.append((a.x, a.y, a.z))
        self.gyr.append((w.x, w.y, w.z))
        self.imu_delay.append(self.now() - t)

    def on_cloud(self, msg):
        t = sec(msg.header.stamp)
        if self.prev_cloud is not None:
            self.cloud_dt.append(t - self.prev_cloud)
        self.prev_cloud = t
        pts = cloud_to_array(msg)
        self.cloud_n.append(len(pts))
        if 'time' in pts.dtype.names and len(pts):
            tf = pts['time'].astype(float)
            self.t_min.append(tf.min())
            self.t_max.append(tf.max())
            end = t + tf.max()
        else:
            end = t
        if self.last_imu_stamp is not None:
            self.offset.append(end - self.last_imu_stamp)
        self.cloud_delay.append(self.now() - t)

    def report(self):
        log = self.get_logger().info
        if self.acc:
            acc = np.array(self.acc)
            gyr = np.array(self.gyr)
            m = acc.mean(axis=0)
            log('---- IMU ----')
            log(f'  rate={1.0 / np.mean(self.imu_dt) if self.imu_dt else 0:.1f} Hz '
                f'dt[{stats(self.imu_dt)}] stamp逆行={self.imu_back}')
            log(f'  acc mean=[{m[0]:+.3f} {m[1]:+.3f} {m[2]:+.3f}] |mean|={np.linalg.norm(m):.3f} '
                f'std=[{" ".join(f"{s:.3f}" for s in acc.std(axis=0))}]')
            g = gyr.mean(axis=0)
            log(f'  gyr mean=[{g[0]:+.4f} {g[1]:+.4f} {g[2]:+.4f}] rad/s '
                f'std=[{" ".join(f"{s:.4f}" for s in gyr.std(axis=0))}]')
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, m[2] / np.linalg.norm(m)))))
            log(f'  重力方向と IMU z 軸の角度={tilt:.1f} deg (取付 pitch と合うか)')
            log(f'  受信遅れ[{stats(self.imu_delay)}] s')
        else:
            log('IMU が来ていない')
        if self.cloud_n:
            log('---- Cloud ----')
            log(f'  rate={1.0 / np.mean(self.cloud_dt) if self.cloud_dt else 0:.1f} Hz '
                f'points mean={np.mean(self.cloud_n):.0f}')
            log(f'  time field min[{stats(self.t_min)}] max[{stats(self.t_max)}] s')
            log(f'  スキャン終了時刻 - 最新IMU stamp [{stats(self.offset)}] s  (0 付近のはず)')
            log(f'  受信遅れ[{stats(self.cloud_delay)}] s')
        else:
            log('点群が来ていない')
        last = self.last_imu_stamp
        self.reset()
        self.last_imu_stamp = last


def main():
    rclpy.init()
    node = CheckInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
