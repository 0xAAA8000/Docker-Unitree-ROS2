"""モード選択の Web (ポート 8080) と, モードの起動・停止を行うノード.

  ros2 run robot_launcher launcher --ros-args -p token:=abcd -p launch_args:="lidar:=l1"

スマホで http://<IP>:8080 を開いてモードを選ぶ. 選んだモードを ros2 launch で起動し,
準備ができたか (LiDAR の点群, Point-LIO の自己位置, Nav2, モードの画面) を順に確かめてから,
モードの画面 (http://<IP>:8081) へ移る. モードは同時に 1 つだけ動かす (Arduino のポートの取り合いを
防ぐ). 問題の切り分けのため, LiDAR ドライバや Point-LIO も含めてモードごとに起動し直す.

  * モードごとの出力は log_dir (~/robot_logs) に <日時>_<モード>.log として保存する
  * 目的地設定のモードでは, robot_map が貯めた地図 (ボクセル地図) を map_dir (~/robot_maps) の
    <日時>_<モード>.pcd に保存させる (map_save:= で渡す. robot_map は 60 秒ごとと終了時に書く)
  * キャリブレーションのモードで選んだ計測結果 (calib_dir/selected) を, 目的地設定のモードの
    calibration_file に渡す
  * パラメータの既定値は robot_config (nodes.launcher) から読む (systemd から ros2 launch を挟まずに
    起動するため, launch ではなくこのノードが自分で読む). --ros-args -p で上書きできる.
    ロボットは -p robot:=<名前> か環境変数 ROBOT_NAME (無ければ rc_car). 各モードにも robot:= で渡す
"""
import os
import shlex
import signal
import subprocess
import threading
import time
import urllib.request

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_config import default_robot, load as load_config
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2

from robot_launcher.http_util import make_server

STEP_NAMES = {
    'stop': '前のモードを停止',
    'lidar': 'LiDAR の点群',
    'pose': '自己位置 (Point-LIO)',
    'nav2': 'Nav2',
    'web': 'モードの画面',
}
MODES = {
    'goal': {'name': '目的地設定の自走', 'steps': ['stop', 'lidar', 'pose', 'nav2', 'web'],
             'available': True, 'note': '地図をタップして目的地へ自走する'},
    'calibration': {'name': 'キャリブレーション', 'steps': ['stop', 'lidar', 'pose', 'web'],
                    'available': True, 'note': 'スロットルと舵角を計測する. 使う計測結果を選ぶ'},
    'explore': {'name': 'マップ自動生成の自走', 'steps': [], 'available': False,
                'note': '準備中 (探索ノードが未実装)'},
}
NAV2_ACTIVE = 3   # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE


def selected_calibration(calib_dir):
    """calib_dir/selected に書かれた計測結果の summary.yaml. 無ければ ''."""
    try:
        with open(os.path.join(calib_dir, 'selected')) as f:
            run = f.read().strip()
    except OSError:
        return ''
    path = os.path.join(calib_dir, run, 'summary.yaml')
    return path if run and os.path.isfile(path) else ''


def tail(path, n=40):
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 16000))
            lines = f.read().decode(errors='replace').splitlines()
        return lines[-n:]
    except OSError:
        return []


class Launcher(Node):

    def __init__(self):
        super().__init__('robot_launcher')
        self.robot = self.declare_parameter('robot', default_robot()).value or default_robot()
        cfg = load_config(self.robot).node('launcher')     # 既定値 (robot_config)

        def p(name, default):
            return self.declare_parameter(name, cfg.get(name, default))
        self.host = p('host', '0.0.0.0').value
        self.port = p('port', 8080).value
        self.mode_port = p('mode_port', 8081).value
        self.token = p('token', '').value
        self.launch_args = p('launch_args', '').value       # 全モードの ros2 launch に足す引数
        self.arduino_port = p('arduino_port', '').value     # 空ならロボットの設定の値 (各モードが読む)
        self.calib_dir = os.path.expanduser(p('calib_dir', '~/drive_calib').value)
        self.log_dir = os.path.expanduser(p('log_dir', '~/robot_logs').value)
        self.map_dir = os.path.expanduser(p('map_dir', '~/robot_maps').value)
        self.start_timeout = p('start_timeout', 90.0).value
        self.stop_timeout = p('stop_timeout', 30.0).value
        self.start_mode = p('start_mode', '').value         # 起動時に自動で始めるモード (空なら待つ)
        cloud_topic = p('cloud_topic', '/unilidar/cloud').value
        odom_topic = p('odom_topic', '/aft_mapped_to_init').value
        nav_state = p('nav2_state_service', '/bt_navigator/get_state').value
        # 試験用: モードの起動コマンドを差し替える (空なら既定のコマンド)
        self.command_override = {m: p(f'command_{m}', '').value for m in MODES}

        self.lock = threading.Lock()
        self.phase = 'idle'          # idle / starting / running / stopping / failed
        self.mode = None
        self.step = 0
        self.message = ''
        self.log_file = ''
        self.fail_tail = []
        self.last_map = ''
        self.proc = None
        self.proc_mode = None        # 今のプロセスのモード
        self.map_file = ''           # 今のモードが地図を保存するファイル (無ければ '')
        self.mode_started_wall = 0.0
        self.abort = threading.Event()
        self.worker = None
        self.gen = 0                 # 切り替えの世代. 古いスレッドが新しい状態を上書きしないように

        self.last_cloud = 0.0
        self.last_odom = 0.0
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.nav_state = self.create_client(GetState, nav_state)

    # ---------- ROS ----------
    def on_cloud(self, msg):
        self.last_cloud = time.monotonic()

    def on_odom(self, msg):
        self.last_odom = time.monotonic()

    def nav2_active(self):
        if not self.nav_state.service_is_ready():
            return False
        future = self.nav_state.call_async(GetState.Request())
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            future.cancel()
            return False
        res = future.result()
        return res is not None and res.current_state.id == NAV2_ACTIVE

    def web_ready(self):
        url = f'http://127.0.0.1:{self.mode_port}/api/state'
        if self.token:
            url += '?key=' + urllib.request.quote(self.token)
        try:
            with urllib.request.urlopen(url, timeout=1.0) as r:
                return r.status == 200
        except OSError:
            return False

    def check(self, step, t0):
        if step == 'lidar':
            return self.last_cloud > t0
        if step == 'pose':
            return self.last_odom > t0
        if step == 'nav2':
            return self.nav2_active()
        if step == 'web':
            return self.web_ready()
        return True

    # ---------- プロセス ----------
    def command(self, mode):
        if self.command_override.get(mode):
            return shlex.split(self.command_override[mode])
        common = [f'robot:={self.robot}', f'launcher_port:={self.port}']
        if self.arduino_port:
            common.append(f'arduino_port:={self.arduino_port}')
        if mode == 'goal':
            cmd = ['ros2', 'launch', 'robot_bringup', 'robot.launch.py',
                   f'web_port:={self.mode_port}'] + common
            if self.token:
                cmd.append(f'web_token:={self.token}')
            stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(self.mode_started_wall))
            self.map_file = os.path.join(self.map_dir, f'{stamp}_{mode}.pcd')
            cmd.append(f'map_save:={self.map_file}')
            calib = selected_calibration(self.calib_dir)
            if calib:
                cmd.append(f'calibration_file:={calib}')
        else:
            self.map_file = ''      # キャリブレーションのモードは地図を作らない
            cmd = ['ros2', 'launch', 'robot_launcher', 'calibration.launch.py',
                   f'port:={self.mode_port}', f'calib_dir:={self.calib_dir}'] + common
            if self.token:
                cmd.append(f'token:={self.token}')
        return cmd + shlex.split(self.launch_args)

    def stop_proc(self):
        """モードのプロセス (とその子) を Ctrl-C と同じ SIGINT で止め, 終わるまで待つ."""
        proc = self.proc
        if proc is None:
            return
        pgid = proc.pid   # start_new_session=True なのでプロセスグループ ID = pid
        for sig, wait in ((signal.SIGINT, self.stop_timeout), (signal.SIGTERM, 5.0),
                          (signal.SIGKILL, 5.0)):
            if sig != signal.SIGINT:
                self.get_logger().warn(f'モードが終わらないので {sig.name} を送る')
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + wait
            gone = False
            while time.monotonic() < deadline and not gone:
                proc.poll()
                try:
                    os.killpg(pgid, 0)   # グループに 1 つでも残っているか
                except ProcessLookupError:
                    gone = True
                    break
                time.sleep(0.2)
            if gone:
                break
        proc.poll()
        self.proc = None
        self.save_map()

    def save_map(self):
        """モードが保存した地図 (robot_map のボクセル地図) を最後の地図として覚える."""
        path = self.map_file
        # ファイルの時刻は粗い時計で記録され, 起動時刻よりわずかに前になることがあるので 2 秒の余裕を見る
        if path and os.path.isfile(path) and os.path.getmtime(path) >= self.mode_started_wall - 2.0:
            self.last_map = path
            self.get_logger().info(f'地図: {path}')

    def set(self, gen, **kw):
        """gen が今の世代のときだけ状態を書き換える."""
        with self.lock:
            if gen != self.gen:
                return
            for k, v in kw.items():
                setattr(self, k, v)

    def run_mode(self, mode, gen):
        """別スレッド: 前のモードを止め, mode を起動し, 準備ができるまで確かめる."""
        steps = MODES[mode]['steps']
        self.stop_proc()
        if self.abort.is_set():
            self.set(gen, phase='idle', mode=None)
            return

        os.makedirs(self.log_dir, exist_ok=True)
        self.mode_started_wall = time.time()
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(self.mode_started_wall))
        log_path = os.path.join(self.log_dir, f'{stamp}_{mode}.log')
        cmd = self.command(mode)
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
        with open(log_path, 'ab') as log:
            log.write(('$ ' + ' '.join(shlex.quote(c) for c in cmd) + '\n').encode())
            log.flush()
            try:
                self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                             stdin=subprocess.DEVNULL, env=env,
                                             start_new_session=True)
            except OSError as e:
                self.set(gen, phase='failed', message=f'起動できない: {e}', log_file=log_path,
                         fail_tail=tail(log_path))
                return
        self.proc_mode = mode
        self.set(gen, log_file=log_path, step=1)
        self.get_logger().info(f'モード {mode} を起動: {" ".join(cmd)} (ログ {log_path})')

        t0 = time.monotonic()
        for i in range(1, len(steps)):
            self.set(gen, step=i)
            while not self.check(steps[i], t0):
                if self.abort.is_set():
                    self.stop_proc()
                    self.set(gen, phase='idle', mode=None)
                    return
                if self.proc.poll() is not None:
                    self.fail(gen, f'{STEP_NAMES[steps[i]]} の前にモードが終了した '
                              f'(終了コード {self.proc.returncode})')
                    return
                if time.monotonic() - t0 > self.start_timeout:
                    self.fail(gen, f'{STEP_NAMES[steps[i]]} が {self.start_timeout:.0f} 秒以内に'
                              'そろわなかった')
                    return
                time.sleep(0.3)
        self.set(gen, phase='running', step=len(steps), message='')
        self.get_logger().info(f'モード {mode} の準備ができた ({time.monotonic() - t0:.0f} 秒)')

        # 動いている間, モードが勝手に終わらないか見張る
        while not self.abort.is_set():
            if self.proc is None or self.proc.poll() is not None:
                code = self.proc.returncode if self.proc else None
                self.fail(gen, f'モードが終了した (終了コード {code})')
                return
            time.sleep(0.5)
        self.stop_proc()
        self.set(gen, phase='idle', mode=None)

    def fail(self, gen, message):
        self.get_logger().error(message)
        log = self.log_file
        self.stop_proc()
        self.set(gen, phase='failed', message=message, fail_tail=tail(log))

    # ---------- HTTP から ----------
    def start(self, mode):
        if mode not in MODES or not MODES[mode]['available']:
            return False, 'このモードは選べない'
        with self.lock:
            if self.phase in ('starting', 'stopping'):
                return False, '切り替え中. 少し待ってからもう一度'
            self.gen += 1
            gen = self.gen
            self.phase, self.mode, self.step = 'starting', mode, 0
            self.message, self.fail_tail = '', []
            old = self.worker

        def run():
            # 前のモードの見張りを止めてから (止めるのは run_mode の stop_proc), 新しいモードを起動する
            if old is not None and old.is_alive():
                self.abort.set()
                old.join()
            self.abort.clear()
            self.run_mode(mode, gen)
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()
        return True, f'{MODES[mode]["name"]} を起動する'

    def stop(self):
        with self.lock:
            if self.phase == 'stopping':
                return False, '停止中'
            self.gen += 1
            gen = self.gen
            self.phase = 'stopping'
            worker = self.worker

        def run():
            self.abort.set()
            if worker is not None and worker.is_alive():
                worker.join()
            self.stop_proc()          # 失敗したあとなど, 見張りのスレッドが無い場合
            self.abort.clear()
            self.set(gen, phase='idle', mode=None, message='', fail_tail=[])
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()
        return True, '停止する'

    def shutdown(self):
        """サービス終了時: 動いているモードを止めてから終わる."""
        with self.lock:
            self.gen += 1
            worker = self.worker
        self.abort.set()
        if worker is not None and worker.is_alive():
            worker.join(timeout=self.stop_timeout + 15.0)
        self.stop_proc()

    def get_state(self):
        with self.lock:
            mode = self.mode
            steps = MODES[mode]['steps'] if mode else []
            status = []
            for i, s in enumerate(steps):
                if self.phase == 'running' or i < self.step:
                    st = 'done'
                elif i == self.step:
                    st = 'failed' if self.phase == 'failed' else 'active'
                else:
                    st = 'pending'
                status.append({'id': s, 'name': STEP_NAMES[s], 'status': st})
            calib = selected_calibration(self.calib_dir)
            return {
                'phase': self.phase,
                'mode': mode,
                'mode_name': MODES[mode]['name'] if mode else '',
                'steps': status,
                'message': self.message,
                'log_file': self.log_file,
                'log_tail': self.fail_tail if self.phase == 'failed' else [],
                'mode_port': self.mode_port,
                'last_map': self.last_map,
                'calibration': os.path.basename(os.path.dirname(calib)) if calib else '',
                'modes': [{'id': k, 'name': v['name'], 'available': v['available'],
                           'note': v['note']} for k, v in MODES.items()],
            }


def main():
    # rclpy の SIGINT ハンドラは使わない (止めるときにモードを順に止めるため)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Launcher()

    def on_term(signum, frame):   # systemd の停止 (SIGTERM) も Ctrl-C と同じに扱う
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    # 親から SIGINT を無視する設定を引き継いでいても (nohup やシェルのバックグラウンドで起動したとき),
    # Ctrl-C で止まり, 起動するモードのプロセスにも Ctrl-C が効くようにする
    signal.signal(signal.SIGINT, signal.default_int_handler)

    share = get_package_share_directory('robot_launcher')
    with open(os.path.join(share, 'web', 'launcher.html'), 'rb') as f:
        page = f.read()
    routes = {
        ('GET', '/api/state'): lambda q, b: (200, node.get_state()),
        ('POST', '/api/start'): lambda q, b: (lambda r: (200 if r[0] else 409,
                                                         {'ok': r[0], 'message': r[1]}))(
                                                             node.start(str(b.get('mode', '')))),
        ('POST', '/api/stop'): lambda q, b: (lambda r: (200, {'ok': r[0], 'message': r[1]}))(
            node.stop()),
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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    key = f'/?key={node.token}' if node.token else '/'
    node.get_logger().info(f'モード選択: http://<このマシンの IP>:{node.port}{key} '
                           f'(モードの画面は {node.mode_port}, ログ {node.log_dir}, '
                           f'地図 {node.map_dir})')
    if node.start_mode:
        ok, msg = node.start(node.start_mode)
        node.get_logger().info(f'起動時のモード {node.start_mode}: {msg}')
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        node.get_logger().info('終了: 動いているモードを止める')
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 止めている途中の Ctrl-C を無視
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.shutdown()
    finally:
        server.shutdown()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
