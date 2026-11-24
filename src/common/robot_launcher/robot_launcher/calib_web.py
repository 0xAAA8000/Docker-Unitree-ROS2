"""キャリブレーションのモードの画面 (ポート 8081).

  ros2 launch robot_launcher calibration.launch.py      # LiDAR と Point-LIO も一緒に起動する

  * calib_dir (~/drive_calib) の計測結果を一覧し, 使うものを選ぶ (calib_dir/selected に書く).
    モード選択の launcher は, 目的地設定の自走を起動するときにそれを calibration_file として渡す
  * calibrate_drive を起動して計測する. 出力を画面に流し, 中止できる
  * 計測中に画面からの問い合わせが deadman 秒途切れたら, 計測を中止する (車が自分で走り回るため)
"""
from collections import deque
import os
import re
import shlex
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import yaml

from robot_launcher.http_util import make_server

RUN_NAME = re.compile(r'^[0-9A-Za-z_.-]+$')


def _get(d, *keys):
    for k in keys:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def read_run(calib_dir, name):
    """計測結果 1 つ分の要約. summary.yaml が無ければ values は None."""
    path = os.path.join(calib_dir, name, 'summary.yaml')
    run = {'name': name, 'has_summary': os.path.isfile(path), 'values': None}
    if not run['has_summary']:
        return run
    try:
        with open(path) as f:
            s = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return run
    run['values'] = {
        'wheelbase': _get(s, 'wheelbase'),
        'forward_gain': _get(s, 'throttle', 'forward', 'gain_mps_per_throttle'),
        'forward_deadband': _get(s, 'throttle', 'forward', 'deadband_throttle'),
        'backward_gain': _get(s, 'throttle', 'backward', 'gain_mps_per_throttle'),
        'backward_deadband': _get(s, 'throttle', 'backward', 'deadband_throttle'),
        'servo_center': _get(s, 'steering', 'servo_center_deg'),
        'sine_ratio_K': _get(s, 'steering', 'sine_ratio_K'),
        'left_max': _get(s, 'steering', 'left_max_deg'),
        'right_max': _get(s, 'steering', 'right_max_deg'),
    }
    return run


def int_list(text):
    """'15, 20,30' -> '15,20,30'. 整数のカンマ区切りでなければ ValueError."""
    items = [t.strip() for t in str(text).split(',') if t.strip()]
    return ','.join(str(int(t)) for t in items)


class CalibWeb(Node):

    def __init__(self):
        super().__init__('calib_web')
        p = self.declare_parameter
        self.host = p('host', '0.0.0.0').value
        self.port = p('port', 8081).value
        self.token = p('token', '').value
        self.launcher_port = p('launcher_port', 0).value
        self.arduino_port = p('arduino_port', '/dev/ttyACM0').value
        self.calib_dir = os.path.expanduser(p('calib_dir', '~/drive_calib').value)
        self.wheelbase = p('wheelbase', 0.26).value       # 画面の初期値 [m]
        self.lidar_x = p('lidar_x', 0.25).value           # 後輪軸から LiDAR まで [m] (画面の初期値)
        self.deadman = p('deadman', 5.0).value            # 画面からの問い合わせが途切れたら中止 [s]
        self.command = p('command', 'ros2 run robot_operation calibrate_drive').value

        self.lock = threading.Lock()
        self.proc = None
        self.job = {'status': 'idle', 'message': '', 'out_dir': '', 'started': 0.0}
        self.lines = deque(maxlen=400)
        self.last_seen = time.monotonic()
        threading.Thread(target=self.watchdog, daemon=True).start()

    # ---------- 計測結果 ----------
    def runs(self):
        try:
            names = sorted(
                (n for n in os.listdir(self.calib_dir)
                 if RUN_NAME.match(n) and os.path.isdir(os.path.join(self.calib_dir, n))),
                reverse=True)
        except OSError:
            names = []
        return [read_run(self.calib_dir, n) for n in names]

    def selected(self):
        try:
            with open(os.path.join(self.calib_dir, 'selected')) as f:
                return f.read().strip()
        except OSError:
            return ''

    def select(self, name):
        name = str(name)
        if not RUN_NAME.match(name) or not os.path.isfile(
                os.path.join(self.calib_dir, name, 'summary.yaml')):
            return False, 'その計測結果は使えない (summary.yaml が無い)'
        os.makedirs(self.calib_dir, exist_ok=True)
        with open(os.path.join(self.calib_dir, 'selected'), 'w') as f:
            f.write(name + '\n')
        self.get_logger().info(f'計測結果 {name} を選んだ')
        return True, f'{name} を使う. 目的地設定の自走を起動し直すと反映される'

    # ---------- 計測 ----------
    def start(self, body):
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return False, '計測中'
        try:
            wheelbase = float(body.get('wheelbase', self.wheelbase))
            lidar_x = float(body.get('lidar_x', self.lidar_x))
            throttles = int_list(body.get('throttles', '')) if body.get('throttles') else ''
            servos = int_list(body.get('servos', '')) if body.get('servos') else ''
        except (TypeError, ValueError):
            return False, '数値が読めない'
        if not 0.05 <= wheelbase <= 2.0 or not -1.0 <= lidar_x <= 2.0:
            return False, 'ホイールベースか LiDAR の位置が範囲外'
        if not body.get('confirmed'):
            return False, '周りに 3 m の空きがあることを確認する'
        cmd = shlex.split(self.command) + [
            '--wheelbase', f'{wheelbase}', '--lidar-x', f'{lidar_x}',
            '--port', self.arduino_port, '--out-dir', self.calib_dir]
        if throttles:
            cmd += ['--throttles', throttles]
        if servos:
            cmd += ['--servos', servos]
        os.makedirs(self.calib_dir, exist_ok=True)
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, env=env, start_new_session=True)
        except OSError as e:
            return False, f'起動できない: {e}'
        with self.lock:
            self.proc = proc
            self.lines.clear()
            self.lines.append('$ ' + ' '.join(shlex.quote(c) for c in cmd))
            self.job = {'status': 'running', 'message': '', 'out_dir': '',
                        'started': time.time()}
            self.last_seen = time.monotonic()
        threading.Thread(target=self.read_output, args=(proc,), daemon=True).start()
        self.get_logger().info(f'計測を開始: {" ".join(cmd)}')
        return True, '計測を開始した'

    def read_output(self, proc):
        for raw in proc.stdout:
            line = raw.decode(errors='replace').rstrip()
            with self.lock:
                self.lines.append(line)
                m = re.search(r'記録: (\S+)', line)
                if m:
                    self.job['out_dir'] = os.path.basename(m.group(1).rstrip('/'))
        code = proc.wait()
        with self.lock:
            if self.job['status'] == 'canceling':
                status = 'canceled'
            else:
                status = 'done' if code == 0 else 'failed'
            self.job['status'] = status
            if not self.job['message']:
                self.job['message'] = {'done': '計測が終わった', 'canceled': '中止した',
                                       'failed': f'計測が失敗した (終了コード {code})'}[status]
        self.get_logger().info(f'計測が終わった: {status} (終了コード {code})')

    def cancel(self, reason='中止した'):
        with self.lock:
            proc = self.proc
            if proc is None or proc.poll() is not None:
                return False, '計測していない'
            self.job['status'] = 'canceling'
            self.job['message'] = reason
        try:
            os.killpg(proc.pid, signal.SIGINT)   # calibrate_drive は Ctrl-C でスロットル 0 を送って止まる
        except ProcessLookupError:
            pass
        self.get_logger().warn(f'計測を中止: {reason}')
        return True, reason

    def watchdog(self):
        while True:
            time.sleep(0.5)
            with self.lock:
                running = self.proc is not None and self.proc.poll() is None \
                    and self.job['status'] == 'running'
                idle = time.monotonic() - self.last_seen
            if running and idle > self.deadman:
                self.cancel(f'画面との通信が {idle:.0f} 秒途切れたので中止した')

    def shutdown(self):
        with self.lock:
            proc = self.proc
        if proc is not None and proc.poll() is None:
            self.cancel('モードを止めるので中止した')
            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)

    def get_state(self):
        runs = self.runs()
        with self.lock:
            self.last_seen = time.monotonic()
            return {
                'runs': runs,
                'selected': self.selected(),
                'job': dict(self.job),
                'lines': list(self.lines)[-200:],
                'defaults': {'wheelbase': self.wheelbase, 'lidar_x': self.lidar_x},
                'launcher_port': self.launcher_port,
                'calib_dir': self.calib_dir,
                'deadman': self.deadman,
            }


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CalibWeb()

    def on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    share = get_package_share_directory('robot_launcher')
    with open(os.path.join(share, 'web', 'calib.html'), 'rb') as f:
        page = f.read()

    def reply(result):
        ok, msg = result
        return (200 if ok else 409), {'ok': ok, 'message': msg}
    routes = {
        ('GET', '/api/state'): lambda q, b: (200, node.get_state()),
        ('POST', '/api/select'): lambda q, b: reply(node.select(b.get('run', ''))),
        ('POST', '/api/start'): lambda q, b: reply(node.start(b)),
        ('POST', '/api/cancel'): lambda q, b: reply(node.cancel()),
    }
    try:
        server = make_server(node.host, node.port, routes, {'/': page, '/index.html': page},
                             node.token)
    except OSError as e:
        node.get_logger().error(f'ポート {node.port} を開けない: {e}')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'キャリブレーションの画面: http://<このマシンの IP>:{node.port} '
                           f'(計測結果 {node.calib_dir})')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
