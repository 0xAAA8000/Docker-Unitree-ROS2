#!/usr/bin/env python3
"""FS5103B サーボ (Arduino Uno D13) にシリアルでコマンドを送る。

使い方:
  ./servo_cmd.py 90            # 角度 90deg
  ./servo_cmd.py p1500         # パルス幅 1500us
  ./servo_cmd.py 0 90 180 90   # 順に送る (各 1 秒待機)
  ./servo_cmd.py               # 対話モード
"""
import sys
import time

import serial

PORT = '/dev/ttyACM0'
BAUD = 115200


def send(ser, cmd):
    ser.write((cmd + '\n').encode())
    print(f'> {cmd}  < {ser.readline().decode().strip()}')


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        # ポートを開くと Uno がリセットされるので READY を待つ
        deadline = time.time() + 5
        while time.time() < deadline:
            if ser.readline().startswith(b'READY'):
                break
        args = sys.argv[1:]
        if args:
            for i, cmd in enumerate(args):
                if i:
                    time.sleep(1.0)
                send(ser, cmd)
        else:
            try:
                while True:
                    send(ser, input('angle / pXXXX / ? : ').strip())
            except (EOFError, KeyboardInterrupt):
                print()


if __name__ == '__main__':
    main()
