"""実機を自動で走らせて、スロットル(PWM)と速度、サーボ角と舵角の対応を計測する.

前提
  * Arduino に firmware/operator/operator.ino が書き込まれている
    (シリアル "<サーボ角> <スロットル>\\n", サーボ 40-130 / スロットル -100..100)
  * Point-LIO (mapping_unilidar_l2.launch.py) が起動していて、
    /aft_mapped_to_init (nav_msgs/Odometry) が出ている
  * 周囲に半径 3 m ほどの空きがある

使い方
  ros2 run robot_operation calibrate_drive --wheelbase 0.26 --lidar-x 0.20
  ros2 run robot_operation calibrate_drive --analyze ~/drive_calib/20261007_120000 \\
      --wheelbase 0.26 --lidar-x 0.20          # 記録済みデータを解析し直すだけ

Ctrl-C で止めると、すぐにスロットル 0・サーボ中央を送る.
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from robot_operation.calibration_analysis import (
    analyze_run, RAW_FIELDS, SEGMENT_FIELDS, yaw_of_body_x)

SERVO_MIN = 40      # operator.ino の ANGLE_MIN / ANGLE_MAX
SERVO_MAX = 130
THROTTLE_LIMIT = 100


class Arduino:
    """operator.ino とのシリアル通信."""

    def __init__(self, port, baud, center):
        import serial  # python3-serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.center = center
        self.last = None
        self._lock = threading.Lock()
        # ポートを開くと Uno がリセットされるので READY を待つ
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self.ser.readline().startswith(b'[ READY ]'):
                break
        else:
            print('[WARN] Arduino の READY を受信できませんでした (続行します)')

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
                    print(f'[ARDUINO] {line}')

    def stop(self):
        self.send(self.center, 0)

    def close(self):
        try:
            self.stop()
            time.sleep(0.05)
            self.stop()
        finally:
            self.ser.close()


class OdomRecorder(Node):
    """オドメトリをすべて記録し, 最新の位置と向きを提供する."""

    def __init__(self, topic):
        super().__init__('drive_calibrator')
        self.samples = []
        self.lock = threading.Lock()
        self.create_subscription(Odometry, topic, self._cb, 10)

    def _cb(self, msg):
        # 区間の開始/終了と揃えるため, 受信時刻 (wall time) で記録する
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.lock:
            self.samples.append((time.time(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None


class Calibrator:

    def __init__(self, args, node, arduino, out_dir):
        self.a = args
        self.node = node
        self.ard = arduino
        self.out_dir = out_dir
        self.segments = []

    # ---------------- 基本動作 ----------------
    def wait_odom(self, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            s = self.node.latest()
            if s and time.time() - s[0] < 0.5:
                return s
            time.sleep(0.05)
        raise RuntimeError('オドメトリが来ていません (Point-LIO は起動していますか?)')

    def hold(self, servo, throttle, seconds):
        """一定時間, 同じ指令を送り続ける (オドメトリ途絶で停止)."""
        end = time.time() + seconds
        while time.time() < end:
            self.ard.send(servo, throttle)
            self._check_odom_alive()
            time.sleep(1.0 / self.a.rate)

    def _check_odom_alive(self):
        s = self.node.latest()
        if s is None or time.time() - s[0] > self.a.odom_timeout:
            self.ard.stop()
            raise RuntimeError('オドメトリが途絶えたので停止しました')

    def run_segment(self, kind, servo, throttle, max_time, max_yaw_deg=None):
        """1区間走る. 時間 / 移動距離 / 旋回角のどれかに達したら止める."""
        if self.a.pause:
            input(f'  [Enter] で開始: {kind} servo={servo} throttle={throttle} ')
        self.ard.send(self.a.center if kind == 'throttle' else servo, 0)
        time.sleep(self.a.steer_wait)           # サーボが切れ終わるのを待つ
        start = self.wait_odom()
        yaw0 = yaw_of_body_x(*start[4:8])
        prev_yaw, turned = yaw0, 0.0
        t0 = time.time()
        reason = 'time'
        while True:
            self.ard.send(servo, throttle)
            self._check_odom_alive()
            s = self.node.latest()
            yaw = yaw_of_body_x(*s[4:8])
            turned += math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
            prev_yaw = yaw
            dist = math.hypot(s[1] - start[1], s[2] - start[2])
            if time.time() - t0 >= max_time:
                break
            if dist >= self.a.max_distance:
                reason = 'distance'
                break
            if max_yaw_deg is not None and abs(math.degrees(turned)) >= max_yaw_deg:
                reason = 'yaw'
                break
            time.sleep(1.0 / self.a.rate)
        t1 = time.time()
        self.ard.stop()
        seg = {'id': len(self.segments), 'kind': kind, 'servo': servo,
               'throttle': throttle, 't_start': f'{t0:.4f}', 't_end': f'{t1:.4f}',
               'end_reason': reason}
        self.segments.append(seg)
        print(f'  #{seg["id"]:02d} {kind:8s} servo={servo:3d} throttle={throttle:4d} '
              f'{t1 - t0:5.1f}s dist={dist:4.2f}m yaw={math.degrees(turned):6.1f}deg '
              f'({reason})')
        self.hold(self.a.center, 0, self.a.rest)   # 完全に止まるまで待つ
        return seg

    # ---------------- 計測手順 ----------------
    def throttle_sweep(self):
        print('\n[1/2] スロットル -> 速度 (ステアリング中央で直進)')
        for thr in self.a.throttles:
            self.run_segment('throttle', self.a.center, thr, self.a.run_time)
            if self.a.reverse:
                # 同じだけ後退して元の位置に戻る (後退側の計測も兼ねる)
                self.run_segment('throttle', self.a.center, -thr, self.a.run_time)

    def steering_sweep(self):
        print('\n[2/2] サーボ角 -> 舵角 (一定スロットルで旋回)')
        for servo in self.a.servos:
            self.run_segment('steering', servo, self.a.steer_throttle,
                             self.a.steer_timeout, max_yaw_deg=self.a.turn_angle)

    def save(self):
        with open(os.path.join(self.out_dir, 'segments.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=SEGMENT_FIELDS)
            w.writeheader()
            w.writerows(self.segments)
        with self.node.lock:
            samples = list(self.node.samples)
        with open(os.path.join(self.out_dir, 'raw.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(RAW_FIELDS)
            w.writerows(samples)
        print(f'\n記録: {self.out_dir} (raw {len(samples)} 点, 区間 {len(self.segments)})')


def int_list(text):
    return [int(v) for v in text.split(',') if v.strip()]


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--analyze', metavar='DIR',
                   help='走行せず, 記録済みディレクトリを解析し直す')
    g = p.add_argument_group('車体')
    g.add_argument('--wheelbase', type=float, default=0.0,
                   help='ホイールベース [m] (舵角の計算に必須. 0 なら旋回半径だけ出す)')
    g.add_argument('--lidar-x', type=float, default=0.0,
                   help='後輪軸から LiDAR までの前方向の水平距離 [m]')
    g = p.add_argument_group('接続')
    g.add_argument('--port', default='/dev/ttyACM0')
    g.add_argument('--baud', type=int, default=115200)
    g.add_argument('--odom-topic', default='/aft_mapped_to_init')
    g.add_argument('--out-dir', default=os.path.expanduser('~/drive_calib'))
    g = p.add_argument_group('スロットル計測')
    g.add_argument('--throttles', type=int_list, default=int_list('15,20,25,30,40,50'),
                   help='計測するスロットル値 (0-100, カンマ区切り)')
    g.add_argument('--run-time', type=float, default=2.5, help='1区間の走行時間 [s]')
    g.add_argument('--no-reverse', dest='reverse', action='store_false',
                   help='前進のあと後退で戻る動作をしない')
    g.add_argument('--skip-throttle', action='store_true')
    g = p.add_argument_group('ステアリング計測')
    g.add_argument('--servos', type=int_list, default=int_list('40,55,70,85,100,115,130'),
                   help='計測するサーボ角 (40-130, カンマ区切り)')
    g.add_argument('--steer-throttle', type=int, default=25, help='旋回中のスロットル')
    g.add_argument('--turn-angle', type=float, default=360.0,
                   help='この角度だけ旋回したら区間終了 [deg]')
    g.add_argument('--steer-timeout', type=float, default=15.0, help='旋回区間の最大時間 [s]')
    g.add_argument('--skip-steering', action='store_true')
    g = p.add_argument_group('共通')
    g.add_argument('--center', type=int, default=85, help='サーボの中央 [deg]')
    g.add_argument('--settle', type=float, default=1.0,
                   help='区間の最初のこの時間は加速中として解析から除く [s]')
    g.add_argument('--max-distance', type=float, default=3.0,
                   help='1区間でスタート地点からこれ以上離れたら止める [m]')
    g.add_argument('--rest', type=float, default=1.5, help='区間の間の停止時間 [s]')
    g.add_argument('--steer-wait', type=float, default=0.5,
                   help='走り出す前にサーボを切って待つ時間 [s]')
    g.add_argument('--rate', type=float, default=20.0, help='指令の送信周期 [Hz]')
    g.add_argument('--odom-timeout', type=float, default=0.5,
                   help='オドメトリがこの時間来なければ非常停止 [s]')
    g.add_argument('--pause', action='store_true', help='区間ごとに Enter を待つ')
    args = p.parse_args(argv)

    for v in args.servos:
        if not SERVO_MIN <= v <= SERVO_MAX:
            p.error(f'--servos は {SERVO_MIN}-{SERVO_MAX} の範囲: {v}')
    for v in args.throttles + [args.steer_throttle]:
        if not 0 < v <= THROTTLE_LIMIT:
            p.error(f'スロットルは 1-{THROTTLE_LIMIT}: {v}')
    return args


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    args = parse_args(argv)
    if args.wheelbase <= 0:
        print('[WARN] --wheelbase が未指定なので, 舵角は計算せず旋回半径だけ出します')

    if args.analyze:
        analyze_run(os.path.expanduser(args.analyze), args.wheelbase, args.lidar_x,
                    settle=args.settle)
        return

    out_dir = os.path.join(args.out_dir, time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = OdomRecorder(args.odom_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    arduino = Arduino(args.port, args.baud, args.center)
    cal = Calibrator(args, node, arduino, out_dir)
    completed = False
    try:
        arduino.stop()
        print('オドメトリ待ち...')
        cal.wait_odom()
        print('Point-LIO の初期化のため, 2 秒静止します')
        cal.hold(args.center, 0, 2.0)
        if not args.skip_throttle:
            cal.throttle_sweep()
        if not args.skip_steering:
            cal.steering_sweep()
        completed = True
    except KeyboardInterrupt:
        print('\n中断しました')
    except RuntimeError as e:
        print(f'\n[ERROR] {e}')
    finally:
        arduino.close()
        cal.save()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    if cal.segments:
        analyze_run(out_dir, args.wheelbase, args.lidar_x, settle=args.settle)
    if not completed:
        print('(途中までのデータで解析しました)')


if __name__ == '__main__':
    main()
