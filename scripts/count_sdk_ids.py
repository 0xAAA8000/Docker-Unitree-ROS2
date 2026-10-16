#!/usr/bin/env python3
"""公式 SDK のサンプル example_lidar の出力から, SDK が取りこぼした IMU の数を数える.

  ./bin/example_lidar | python3 count_sdk_ids.py

example_lidar は IMU を解析するたびに "stamp = ..., id = N" を表示する. IMU の id は
LiDAR が 250 Hz で振る連番 (0-1023 で一周) なので, 飛んだ分が SDK の取りこぼしになる.
シリアルを直接解析すると欠落は 0.3% (scripts/mavlink_check.py) なので, ここで欠落が多ければ
SDK が落としている. WSL とネイティブ Linux (ラズパイ等) で比べる.
"""
import re
import sys
import time

pat = re.compile(r'stamp = [\d.]+, id = (\d+)')
imu = lost = cloud = 0
last = None
kind = None
t0 = time.monotonic()
next_report = t0 + 10.0
for line in sys.stdin:
    if line.startswith('An IMU msg'):
        kind = 'imu'
    elif line.startswith('A Cloud msg'):
        kind = 'cloud'
    else:
        m = pat.search(line)
        if m and kind == 'imu':
            i = int(m.group(1))
            imu += 1
            if last is not None:
                lost += (i - last - 1) % 1024
            last = i
        elif m and kind == 'cloud':
            cloud += 1
        if m:
            kind = None
    now = time.monotonic()
    if now >= next_report:
        total = imu + lost
        print(f'[累計] t={now - t0:6.1f}  IMU 受信 {imu} 欠落(id) {lost} '
              f'({100 * lost / max(total, 1):.1f}%) | 点群 {cloud}', flush=True)
        next_report += 10.0
print(f'[終了] IMU 受信 {imu} 欠落(id) {lost} ({100 * lost / max(imu + lost, 1):.1f}%) | 点群 {cloud}')
