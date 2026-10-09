"""firmware/operator/operator.ino とのシリアル通信.

プロトコル: "<サーボ角> <スロットル>\\n"
  サーボ角   : 40-130 [deg] (中央 85)
  スロットル : -100..100 (負は後退)
"""
import threading
import time

SERVO_MIN = 40      # operator.ino の ANGLE_MIN / ANGLE_MAX
SERVO_MAX = 130
THROTTLE_LIMIT = 100


class Arduino:

    def __init__(self, port, baud, center, log=print, ready_timeout=5.0):
        import serial  # python3-serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.center = center
        self.last = None
        self.log = log
        self._lock = threading.Lock()
        # ポートを開くと Uno がリセットされるので READY を待つ
        deadline = time.time() + ready_timeout
        while time.time() < deadline:
            if self.ser.readline().startswith(b'[ READY ]'):
                break
        else:
            self.log('[WARN] Arduino の READY を受信できませんでした (続行します)')

    def send(self, servo, throttle):
        servo = int(max(SERVO_MIN, min(SERVO_MAX, round(servo))))
        throttle = int(max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, round(throttle))))
        with self._lock:
            self.ser.write(f'{servo} {throttle}\n'.encode())
            self.last = (servo, throttle)
            # 応答を読み捨てる (ERR だけ表示)
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors='replace').strip()
                if 'ERR' in line:
                    self.log(f'[ARDUINO] {line}')

    def stop(self):
        self.send(self.center, 0)

    def close(self):
        try:
            self.stop()
            time.sleep(0.05)
            self.stop()
        finally:
            self.ser.close()
