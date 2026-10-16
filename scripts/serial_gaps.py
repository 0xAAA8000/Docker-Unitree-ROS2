#!/usr/bin/env python3
"""LiDAR のシリアルを ROS ドライバを通さず直接読み, 受信が止まる瞬間を表示する.

  python3 scripts/serial_gaps.py                 # /dev/ttyUSB0, 2000000 bps
  python3 scripts/serial_gaps.py /dev/ttyUSB0 0.2

ドライバ (unitree_lidar_ros2) は止めてから実行する (ポートを取り合うため).
watch_gaps で見えた約 3 秒の停止がここでも出れば, ドライバより手前
(L1 本体 / cp210x / usbipd / Windows) で止まっている. 出なければドライバ側.
"""
import os
import select
import sys
import termios
import time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
stall = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2   # これ以上受信が無ければ停止とみなす (s)

fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
attr = termios.tcgetattr(fd)
attr[0] = attr[1] = attr[3] = 0                           # iflag, oflag, lflag: raw
attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL    # cflag
attr[4] = attr[5] = termios.B2000000                      # ispeed, ospeed
termios.tcsetattr(fd, termios.TCSANOW, attr)
termios.tcflush(fd, termios.TCIFLUSH)

t0 = time.monotonic()
last = None          # 最後に受信した時刻
total = 0
window = 0
stalls = 0
reads = 0
big_reads = 0
next_report = t0 + 10.0
print(f'{port} を監視中 (停止判定 {stall} s). Ctrl+C で終了', flush=True)
try:
    while True:
        r, _, _ = select.select([fd], [], [], 0.05)
        now = time.monotonic()
        if r:
            n = len(os.read(fd, 65536))
            # 1 回の読み込みのバイト数. IMU 1 個は 50 B, 点群のパケットは 217-254 B なので,
            # 300 B を超えると 1 回で複数のメッセージが届いている
            reads += 1
            if n > 300:
                big_reads += 1
            if last is not None and now - last > stall:
                stalls += 1
                print(f't={last - t0:8.2f}  受信停止 {now - last:.3f} s', flush=True)
            last = now
            total += n
            window += n
        if now >= next_report:
            print(f'[累計] t={now - t0:8.2f}  直近 10 s {window / 10 / 1000:.0f} kB/s  '
                  f'合計 {total / 1e6:.1f} MB  停止 {stalls} 回  '
                  f'読み込み {reads} 回 (平均 {window / max(reads, 1):.0f} B, 300 B 超 {100 * big_reads / max(reads, 1):.0f}%)',
                  flush=True)
            window = 0
            reads = big_reads = 0
            next_report += 10.0
except KeyboardInterrupt:
    pass
finally:
    os.close(fd)
