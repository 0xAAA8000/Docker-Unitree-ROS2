"""一定速度で決めた秒数だけ前進して止まる (走行中の点群の記録用).

  ros2 run robot_operation drive_forward --speed 0.3 --duration 5
  ros2 run robot_operation drive_forward --speed 0.3 --duration 5 \\
      --bag /opt/common_ws/src/bags/forward1

/cmd_vel に Twist を送り, cmd_vel_converter (operation.launch.py) が Arduino に送る.
速度からスロットルへの変換は cmd_vel_converter のキャリブレーション値による.

--bag を指定すると, 走り出す前 (--pre 秒) から止まった後 (--post 秒) まで
ros2 bag record で記録する. 出力先はコンテナを終了しても消えない場所
(マウントしている /opt/common_ws/src や /ros2_ws/src の下) にすること.

Ctrl-C で止めると速度 0 を送ってから終了する.
"""
import argparse
import os
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

DEFAULT_TOPICS = [
    '/unilidar/cloud', '/unilidar/imu',     # ドライバの生データ (後で別の SLAM で処理し直せる)
    '/lidar', '/imu',                       # lidar_bridge の出力 (Point-LIO の入力)
    '/aft_mapped_to_init', '/cloud_registered', '/path',   # Point-LIO の出力
    '/tf', '/tf_static',
]


class DriveForward(Node):

    def __init__(self, topic):
        super().__init__('drive_forward')
        self.pub = self.create_publisher(Twist, topic, 10)
        self.topic = topic

    def wait_subscriber(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def send(self, speed):
        msg = Twist()
        msg.linear.x = float(speed)
        self.pub.publish(msg)

    def run(self, speed, duration, rate, log):
        """duration 秒間 speed で走り, 速度 0 を送る."""
        period = 1.0 / rate
        start = time.time()
        next_log = start
        try:
            while time.time() - start < duration:
                self.send(speed)
                if time.time() >= next_log:
                    log(f'走行中 {time.time() - start:4.1f} / {duration:.1f} s  speed={speed} m/s')
                    next_log += 1.0
                time.sleep(period)
        finally:
            self.stop(rate)

    def stop(self, rate):
        for _ in range(int(rate)):      # 1 秒間速度 0 を送り続ける
            self.send(0.0)
            time.sleep(1.0 / rate)


def start_bag(path, topics):
    if os.path.exists(path):
        raise SystemExit(f'{path} は既にあります. 別の名前を指定してください')
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    # 自分と同じプロセスグループにすると Ctrl-C が先に届いてしまうので分ける
    return subprocess.Popen(['ros2', 'bag', 'record', '-o', path] + topics,
                            stdout=subprocess.DEVNULL, start_new_session=True)


def stop_bag(proc):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)     # SIGINT で止めるとメタデータが正しく書かれる
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=float, required=True, help='前進速度 [m/s] (負なら後退)')
    ap.add_argument('--duration', type=float, required=True, help='走る時間 [s]')
    ap.add_argument('--topic', default='/cmd_vel', help='Twist の送り先')
    ap.add_argument('--rate', type=float, default=20.0, help='送信周期 [Hz]')
    ap.add_argument('--countdown', type=float, default=3.0, help='走り出すまでの待ち時間 [s]')
    ap.add_argument('--bag', default='', help='ros2 bag の出力先 (空なら記録しない)')
    ap.add_argument('--topics', nargs='+', default=DEFAULT_TOPICS, help='記録するトピック')
    ap.add_argument('--pre', type=float, default=2.0, help='走り出す前に記録する時間 [s]')
    ap.add_argument('--post', type=float, default=2.0, help='止まった後に記録する時間 [s]')
    ap.add_argument('--max-speed', type=float, default=1.0, help='安全のための速度の上限 [m/s]')
    a = ap.parse_args()

    if abs(a.speed) > a.max_speed:
        raise SystemExit(f'--speed {a.speed} が --max-speed {a.max_speed} を超えています')
    if a.duration <= 0.0:
        raise SystemExit('--duration は正の値にしてください')

    # rclpy に SIGINT を処理させない (処理させると Ctrl-C で通信が閉じ, 速度 0 を送れなくなる)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DriveForward(a.topic)
    log = node.get_logger().info
    bag = None
    try:
        if not node.wait_subscriber(3.0):
            raise SystemExit(f'{a.topic} の購読者がいません. operation.launch.py を起動してください')
        if a.bag:
            bag = start_bag(a.bag, a.topics)
            log(f'記録開始: {a.bag}')
            time.sleep(a.pre)
        for i in range(int(a.countdown), 0, -1):
            log(f'{i} 秒後に走り出します (Ctrl-C で中止)')
            time.sleep(1.0)
        node.run(a.speed, a.duration, a.rate, log)
        log('停止しました')
        if bag:
            time.sleep(a.post)
    except KeyboardInterrupt:
        node.stop(a.rate)
        log('中止しました (速度 0 を送信済み)')
    finally:
        stop_bag(bag)
        if bag:
            log(f'記録終了: {a.bag}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
