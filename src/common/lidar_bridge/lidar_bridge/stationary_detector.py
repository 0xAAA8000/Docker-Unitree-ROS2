"""車両が止まっているかを判定して /stationary (std_msgs/Bool) に出す. Point-LIO の ZUPT が使う.

  ros2 run lidar_bridge stationary_detector

次の 2 つがそろったときだけ「停止」とする.
  1. 速度指令 (/cmd_vel) が settle_time 秒以上, 走る指令になっていない
     (指令が来ない間も「走る指令ではない」とみなす. 惰性で転がる分を settle_time で待つ)
  2. IMU が静か: 直近 window 秒の角速度の大きさの最大が gyro_max 以下,
     加速度の大きさの標準偏差が acc_std_max 以下
     (手で持ち上げた, 坂で転がった, 押されたなど, 指令なしで動いたときに誤って停止としないため)

LiDAR の回転による振動で IMU は静止中も揺れるので, しきい値は sensor_check / check_inputs で
静止中の値を見て決める (既定は静止中の実測 acc std 0.12-0.2, gyr std 0.03 程度より余裕をとった値).
"""
from collections import deque
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class StationaryDetector(Node):

    def __init__(self):
        super().__init__('stationary_detector')
        p = self.declare_parameter
        self.out_topic = p('stationary_topic', '/stationary').value
        cmd_topic = p('cmd_vel_topic', '/cmd_vel').value
        imu_topic = p('imu_topic', '/imu').value
        self.settle_time = p('settle_time', 1.0).value      # 走る指令が止んでから待つ時間 [s]
        self.min_speed = p('min_speed', 0.02).value         # これ未満の速度指令は「止まれ」 [m/s]
        self.min_turn = p('min_turn', 0.05).value           # これ未満の角速度指令は「止まれ」 [rad/s]
        self.window = p('window', 0.5).value                # IMU を見る時間 [s]
        self.gyro_max = p('gyro_max', 0.15).value           # 角速度の大きさの上限 [rad/s]
        self.acc_std_max = p('acc_std_max', 0.5).value      # 加速度の大きさの標準偏差の上限 [m/s^2]
        rate = p('rate', 20.0).value

        self.last_move_cmd = None       # 最後に「走る指令」を受けた時刻 (time.monotonic)
        self.started = time.monotonic()
        self.imu = deque()              # (時刻, 角速度の大きさ, 加速度の大きさ)
        self.state = None

        self.pub = self.create_publisher(Bool, self.out_topic, 10)
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'{cmd_topic} と {imu_topic} から停止を判定して {self.out_topic} に出す '
            f'(指令が止んで {self.settle_time} s, 角速度 <= {self.gyro_max} rad/s, '
            f'加速度の標準偏差 <= {self.acc_std_max} m/s^2)')

    def on_cmd(self, msg):
        if abs(msg.linear.x) >= self.min_speed or abs(msg.angular.z) >= self.min_turn:
            self.last_move_cmd = time.monotonic()

    def on_imu(self, msg):
        now = time.monotonic()
        w, a = msg.angular_velocity, msg.linear_acceleration
        self.imu.append((now, math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z),
                         math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)))
        while self.imu and now - self.imu[0][0] > self.window:
            self.imu.popleft()

    def imu_quiet(self, now):
        """IMU が静かか. IMU が届いていない, または window 分たまっていなければ False."""
        if not self.imu or now - self.imu[-1][0] > 0.2 or now - self.imu[0][0] < 0.8 * self.window:
            return False, 'IMU なし'
        gyro = max(g for _, g, _ in self.imu)
        accs = [a for _, _, a in self.imu]
        mean = sum(accs) / len(accs)
        std = math.sqrt(sum((a - mean) ** 2 for a in accs) / len(accs))
        if gyro > self.gyro_max:
            return False, f'角速度 {gyro:.2f} rad/s'
        if std > self.acc_std_max:
            return False, f'加速度の標準偏差 {std:.2f} m/s^2'
        return True, ''

    def on_timer(self):
        now = time.monotonic()
        since_move = now - (self.last_move_cmd if self.last_move_cmd is not None else self.started)
        quiet, why = self.imu_quiet(now)
        stopped = since_move >= self.settle_time and quiet
        self.pub.publish(Bool(data=stopped))
        if stopped != self.state:
            if stopped:
                self.get_logger().info('停止')
            elif since_move < self.settle_time:
                self.get_logger().info('走行中 (速度指令あり)')
            else:
                self.get_logger().info(f'走行中 ({why})')
            self.state = stopped


def main():
    rclpy.init()
    node = StationaryDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
