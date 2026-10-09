#!/usr/bin/env python3
"""Warn when L2 data stops arriving (e.g. the USB-LAN adapter dropped out).

A few seconds without point clouds / IMU while moving is enough to break Point-LIO's map or
make localization lose track, so the gap is reported right away and again when data is back.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

NAMES = {'cloud': '点群', 'imu': 'IMU'}


class DataWatch(Node):
    def __init__(self):
        super().__init__('l2_data_watch')
        self.limit = self.declare_parameter('gap_warn_sec', 1.0).value
        self.last = {'cloud': None, 'imu': None}
        self.gap_start = {'cloud': None, 'imu': None}
        self.create_subscription(PointCloud2, 'cloud', lambda m: self.seen('cloud'),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', lambda m: self.seen('imu'), qos_profile_sensor_data)
        self.create_timer(0.25, self.check)
        self.last_warn = 0.0
        self.start = time.monotonic()

    def seen(self, name):
        now = time.monotonic()
        if self.gap_start[name] is not None:
            gap = now - self.gap_start[name]
            self.get_logger().warn(f'{NAMES[name]}が復帰しました(途切れ {gap:.1f} 秒)。'
                                   'その間に動いていたら、RViz で地図・位置を確認してください')
            self.gap_start[name] = None
        self.last[name] = now

    def check(self):
        now = time.monotonic()
        stopped = [n for n, t in self.last.items()
                   if t is not None and now - t > self.limit]
        for n in stopped:
            if self.gap_start[n] is None:
                self.gap_start[n] = self.last[n]
        never = [n for n, t in self.last.items() if t is None]
        if never and now - self.start > 5.0 and now - self.last_warn > 5.0:
            self.last_warn = now
            self.get_logger().error('L2 のデータがまだ届いていません('
                                    + '、'.join(NAMES[n] for n in never) + ')')
        if stopped and now - self.last_warn > 1.0:
            self.last_warn = now
            parts = '、'.join(f'{NAMES[n]} {now - self.last[n]:.0f} 秒' for n in stopped)
            self.get_logger().error(f'L2 のデータが止まっています({parts})。'
                                    '台車を止めて、LAN ケーブル / USB-LAN アダプタを確認してください')


def main():
    rclpy.init()
    node = DataWatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / launch shutdown
        pass


if __name__ == '__main__':
    main()
