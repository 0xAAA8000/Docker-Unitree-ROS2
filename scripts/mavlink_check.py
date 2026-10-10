#!/usr/bin/env python3
"""L1 のシリアルを SDK を通さず MAVLink として解析し, どこでデータが欠けているかを数える.

  python3 scripts/mavlink_check.py                 # /dev/ttyUSB0
  python3 scripts/mavlink_check.py /dev/ttyUSB0

ドライバ (unitree_lidar_ros2) は止めてから実行する (ポートを取り合うため).
10 秒ごとに次を表示する.
  * IMU 受信 / 欠落(id)  : IMU の packet_id の飛び. ここで欠けていれば L1 本体か USB 経路で欠けている
  * seq 欠落             : MAVLink ヘッダの連番 (全メッセージ共通) の飛び. フレームごと消えた数
  * CRC 不一致 / 読み捨て : 途中のバイトが欠けて壊れたフレームと, 同期が外れて捨てたバイト数
ドライバの [計測] で IMU の約 2 割が欠けていたが, ここで欠けていなければ SDK 側で落としている.
"""
import os
import select
import struct
import sys
import termios
import time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

# msgid: (payload 長, CRC_EXTRA)  (include/mavlink/SysMavlink の定義)
MSGS = {
    11: (1, 106), 12: (1, 204), 13: (38, 62), 14: (1, 84), 15: (56, 236),
    16: (246, 74), 17: (209, 99), 18: (5, 12), 19: (42, 110),
}
IMU = 19


def crc_x25(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
attr = termios.tcgetattr(fd)
attr[0] = attr[1] = attr[3] = 0
attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
attr[4] = attr[5] = termios.B2000000
termios.tcsetattr(fd, termios.TCSANOW, attr)
termios.tcflush(fd, termios.TCIFLUSH)

buf = bytearray()
stat = dict(frames=0, imu=0, imu_lost=0, seq_lost=0, crc_bad=0, skipped=0, unknown=0)
last_seq = None
last_imu = None
imu_max = 0
t0 = time.monotonic()
next_report = t0 + 10.0
print(f'{port} を解析中. Ctrl+C で終了', flush=True)


def parse():
    """buf から取り出せるだけフレームを取り出す."""
    global buf, last_seq, last_imu, imu_max
    i = 0
    n = len(buf)
    while True:
        j1 = buf.find(b'\xfe', i)
        j2 = buf.find(b'\xfd', i)
        cands = [j for j in (j1, j2) if j >= 0]
        if not cands:
            stat['skipped'] += n - i
            i = n
            break
        j = min(cands)
        stat['skipped'] += j - i
        i = j
        if buf[i] == 0xFE:                       # MAVLink v1
            if n - i < 6:
                break
            plen, seq, msgid = buf[i + 1], buf[i + 2], buf[i + 5]
            hdr, total = buf[i + 1:i + 6], 6 + plen + 2
        else:                                    # MAVLink v2
            if n - i < 10:
                break
            plen, incompat, seq = buf[i + 1], buf[i + 2], buf[i + 4]
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            hdr, total = buf[i + 1:i + 10], 10 + plen + 2 + (13 if incompat & 1 else 0)
        if msgid not in MSGS:
            stat['unknown'] += 1
            stat['skipped'] += 1
            i += 1
            continue
        if n - i < total:
            break
        hlen = 6 if buf[i] == 0xFE else 10
        payload = bytes(buf[i + hlen:i + hlen + plen])
        crc = crc_x25(bytes([MSGS[msgid][1]]), crc_x25(hdr + payload))
        got = buf[i + hlen + plen] | (buf[i + hlen + plen + 1] << 8)
        if crc != got:
            stat['crc_bad'] += 1
            stat['skipped'] += 1
            i += 1
            continue
        stat['frames'] += 1
        if last_seq is not None:
            stat['seq_lost'] += (seq - last_seq - 1) % 256
        last_seq = seq
        if msgid == IMU:
            payload = payload.ljust(MSGS[IMU][0], b'\0')   # v2 は末尾の 0 が省略される
            pid = struct.unpack_from('<H', payload, 40)[0]
            stat['imu'] += 1
            imu_max = max(imu_max, pid)
            mod = 1024 if imu_max < 1024 else 65536
            if last_imu is not None:
                stat['imu_lost'] += (pid - last_imu - 1) % mod
            last_imu = pid
        i += total
    del buf[:i]


try:
    while True:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            buf += os.read(fd, 65536)
            parse()
        now = time.monotonic()
        if now >= next_report:
            s = stat
            print(f'[累計] t={now - t0:6.1f}  IMU 受信 {s["imu"]} 欠落(id) {s["imu_lost"]} | '
                  f'フレーム {s["frames"]} seq 欠落 {s["seq_lost"]} | '
                  f'CRC 不一致 {s["crc_bad"]} 読み捨て {s["skipped"]} B', flush=True)
            next_report += 10.0
except KeyboardInterrupt:
    pass
finally:
    os.close(fd)
