"""Nav2 の Twist をステアリング角とスロットルに変換し, シリアル経由で Arduino に送るノード.

Twist のフィールド
  linear.x : 前後の速度 [m/s]
  angular.z: 旋回の角速度 [rad/s] (左旋回が正)

自転車モデル (後輪軸中心の速度 v, ホイールベース L, 前輪の舵角 delta)
  R = v / omega,  tan(delta) = L / R = L * omega / v

サーボ角 phi への変換 (calibrate_drive の summary.yaml の値を使う)
  sin(delta) = K * sin(phi - phi0)
スロットルへの変換
  throttle = deadband + v / gain   (前進 / 後退で別の値)

安全策
  * cmd_timeout 秒 Twist が来なければスロットル 0 にする
  * 一定周期 (rate) で指令を送り続ける. 終了時はスロットル 0・サーボ中央を送る
"""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import yaml

from robot_operation.arduino import Arduino, SERVO_MAX, SERVO_MIN


def load_calibration(path):
    """calibrate_drive の summary.yaml から変換パラメータを取り出す."""
    with open(path) as f:
        data = yaml.safe_load(f)
    out = {}
    if data.get('wheelbase'):
        out['wheelbase'] = float(data['wheelbase'])
    steer = data.get('steering') or {}
    if steer:
        out['servo_center'] = float(steer['servo_center_deg'])
        out['steer_ratio_k'] = float(steer['sine_ratio_K'])
        out['max_steer_deg'] = min(abs(steer['left_max_deg']), abs(steer['right_max_deg']))
    thr = data.get('throttle') or {}
    for name in ('forward', 'backward'):
        if thr.get(name):
            out[f'{name}_gain'] = float(thr[name]['gain_mps_per_throttle'])
            out[f'{name}_deadband'] = float(thr[name]['deadband_throttle'])
    return out


class CmdVelConverter(Node):

    def __init__(self):
        super().__init__('cmd_vel_converter')
        p = self.declare_parameter
        self.topic = p('cmd_vel_topic', '/cmd_vel').value
        self.port = p('port', '/dev/ttyACM0').value
        self.baud = p('baud', 115200).value
        self.dry_run = p('dry_run', False).value            # True: シリアルに送らずログだけ
        self.rate = p('rate', 20.0).value                   # 指令の送信周期 [Hz]
        self.cmd_timeout = p('cmd_timeout', 0.5).value      # [s]
        calibration_file = p('calibration_file', '').value  # summary.yaml (空なら下の値)

        # 車体とステアリング
        self.wheelbase = p('wheelbase', 0.26).value                # [m]
        self.servo_center = p('servo_center', 85.0).value          # 舵角 0 のサーボ角 [deg]
        self.steer_ratio_k = p('steer_ratio_k', 0.48).value        # sin(d) = K sin(phi - phi0)
        self.max_steer_deg = p('max_steer_deg', 20.0).value        # 舵角の上限 [deg]
        # スロットル (throttle = deadband + v / gain)
        self.forward_gain = p('forward_gain', 0.03).value          # [m/s per throttle]
        self.forward_deadband = p('forward_deadband', 12.0).value
        self.backward_gain = p('backward_gain', 0.03).value
        self.backward_deadband = p('backward_deadband', -12.0).value
        self.max_throttle = p('max_throttle', 50.0).value          # 安全のための上限
        self.min_speed = p('min_speed', 0.02).value                # これ未満の速度指令は停止

        if calibration_file:
            for key, value in load_calibration(calibration_file).items():
                setattr(self, key, value)
            self.get_logger().info(f'キャリブレーションを読み込みました: {calibration_file}')
        if self.steer_ratio_k == 0.0:
            raise ValueError('steer_ratio_k が 0 です')
        self.get_logger().info(
            f'L={self.wheelbase:.3f} center={self.servo_center:.1f} K={self.steer_ratio_k:.3f} '
            f'max_steer={self.max_steer_deg:.1f}deg '
            f'fwd={self.forward_gain:.4f}/{self.forward_deadband:.1f} '
            f'bwd={self.backward_gain:.4f}/{self.backward_deadband:.1f}')

        self.arduino = None
        if not self.dry_run:
            self.arduino = Arduino(self.port, self.baud, round(self.servo_center),
                                   log=self.get_logger().warn)
        self.servo = self.servo_center
        self.throttle = 0
        self.last_cmd_time = None

        self.create_subscription(Twist, self.topic, self.on_twist, 10)
        self.create_timer(1.0 / self.rate, self.on_timer)

    # ------------------------------------------------------------------
    def steer_to_servo(self, delta):
        """舵角 [rad] -> サーボ角 [deg]."""
        s = max(-1.0, min(1.0, math.sin(delta) / self.steer_ratio_k))
        return self.servo_center + math.degrees(math.asin(s))

    def speed_to_throttle(self, v):
        if abs(v) < self.min_speed:
            return 0
        if v > 0:
            thr = self.forward_deadband + v / self.forward_gain
            return int(round(min(max(thr, 0.0), self.max_throttle)))
        thr = self.backward_deadband + v / self.backward_gain
        return int(round(max(min(thr, 0.0), -self.max_throttle)))

    def convert(self, v, omega):
        """(v, omega) -> (サーボ角, スロットル)."""
        throttle = self.speed_to_throttle(v)
        if throttle == 0:
            # 停止中は舵角を求められない (アッカーマン車はその場旋回できない) ので前の舵角を保つ
            return self.servo, 0
        delta = math.atan(self.wheelbase * omega / v)    # 後退時は符号が自然に反転する
        limit = math.radians(self.max_steer_deg)
        delta = max(-limit, min(limit, delta))
        return self.steer_to_servo(delta), throttle

    # ------------------------------------------------------------------
    def on_twist(self, msg):
        self.servo, self.throttle = self.convert(msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        if self.last_cmd_time is None:
            servo, throttle = self.servo_center, 0
        else:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                if self.throttle != 0:
                    self.get_logger().warn(f'{self.topic} が {age:.1f}s 途絶えたので停止します')
                self.throttle = 0
            servo, throttle = self.servo, self.throttle
        servo = max(SERVO_MIN, min(SERVO_MAX, servo))
        if self.arduino:
            self.arduino.send(servo, throttle)
        else:
            self.get_logger().info(f'servo={servo:.0f} throttle={throttle}',
                                   throttle_duration_sec=0.5)

    def shutdown(self):
        if self.arduino:
            self.arduino.close()


def main():
    rclpy.init()
    node = CmdVelConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
