#!/usr/bin/env python3
"""Exit once L2 data is flowing with valid timestamps (used to start Point-LIO after it).

Right after the driver starts, a scan/IMU message can carry a zero timestamp. When Point-LIO
takes it as its first scan ("first lidar time0") its IMU initialization finishes at once and
the map comes out tilted (15 deg on 2026-10-09), or it never starts. Point-LIO subscribes only
after this node exits, so it never sees those first messages.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

VALID = 1e9           # stamps before 2001 are not real time
NEED_CLOUDS = 10      # ~1 s of scans
NEED_IMU = 200        # ~1 s of IMU


class WaitForData(Node):
    def __init__(self):
        super().__init__('l2_wait_for_data')
        self.n = {'cloud': 0, 'imu': 0}
        self.last = {'cloud': 0.0, 'imu': 0.0}
        self.create_subscription(PointCloud2, 'cloud', lambda m: self.seen('cloud', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', lambda m: self.seen('imu', m), qos_profile_sensor_data)
        self.start = time.monotonic()
        self.done = False
        self.get_logger().info('L2 のデータを待っています(届いたら Point-LIO を起動します)')

    def seen(self, name, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t < VALID or t < self.last[name]:          # zero / backwards stamp: start counting over
            self.get_logger().warn(f'{name} の時刻が不正です ({t:.3f})。待ち直します')
            self.n = {'cloud': 0, 'imu': 0}
        else:
            self.n[name] += 1
        self.last[name] = t
        if self.n['cloud'] >= NEED_CLOUDS and self.n['imu'] >= NEED_IMU and not self.done:
            self.done = True
            self.get_logger().info(f'L2 のデータを確認しました({time.monotonic() - self.start:.1f} 秒)。'
                                   'Point-LIO を起動します。IMU 初期化が終わるまで L2 を動かさないでください')


def main():
    rclpy.init()
    node = WaitForData()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
