"""
ステアリング: FS5103Bサーボ
スロットル  : IBT-4モータードライバ

Usage:
  operator_cmd.py

Args:
  stg: ステアリングサーボの制御角(center=85)
  thr: PWMのスロットル制御
"""
import sys
import time

import serial

PORT = '/dev/ttyACM0'
BAUD = 115200


def send(ser, cmd):
    ser.write((cmd + '\n').encode())
    print(f'COMMAND: {cmd}')
    time.sleep(0.01)
    while (ser.in_waiting > 0):
        line = ser.readline().decode('utf-8').strip()
        print(line)

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        deadline = time.time() + 5
        while time.time() < deadline:
            if ser.readline().startswith(b"[ READY ]"):
                break

        while True:
            send(ser, input("stg thr: ").strip())

if __name__ == '__main__':
    main()
