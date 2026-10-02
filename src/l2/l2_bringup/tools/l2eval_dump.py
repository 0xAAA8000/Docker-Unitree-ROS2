"""Dump the /pcl_pose recorded in a bag to BAG_eval/live.csv and print the first pose (x y yaw_deg).

  python3 l2eval_dump.py BAG
"""
import math
import os
import sys

import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message

bag = sys.argv[1].rstrip('/')
out = bag + '_eval'
os.makedirs(out, exist_ok=True)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/pcl_pose']))
first = None
with open(f'{out}/live.csv', 'w') as f:
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, PoseWithCovarianceStamped)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        f.write(f'loc,{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.6f},'
                f'{p.x},{p.y},{p.z},{q.x},{q.y},{q.z},{q.w}\n')
        if first is None:
            first = (p.x, p.y, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                        1 - 2 * (q.y ** 2 + q.z ** 2))))
print(f'{first[0]:.3f} {first[1]:.3f} {first[2]:.2f}')
