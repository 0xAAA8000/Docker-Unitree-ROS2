"""スマホのブラウザから地図をタップして Nav2 に目的地を送る Web サーバー.

  ros2 run robot_goal web
  ros2 run robot_goal web --ros-args -p port:=8080 -p token:=abcd   # ?key=abcd を付けた人だけ操作できる

同じ Wi-Fi のスマホで http://<このマシンの IP>:8080 を開く.
地図は自分では作らず, robot_map が配信する /map を表示する.

購読: /map (2D 地図), /plan (Nav2 の経路), TF (map -> base_link),
      /global_costmap/costmap, /local_costmap/costmap (Nav2 が見ている costmap. 画面で切り替える),
      /voxel_map (robot_map の 3D 地図のうち見えている範囲. 画面の 3D 表示)
送信: navigate_to_pose アクション (robot_goal goal と同じ)
呼ぶ: costmap の障害物の消去 (画面の「障害物を消す」. 誤検知が残ったとき用)
"""
import math
import os

import numpy as np
import socket
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
import tf2_ros

from robot_goal.goal import quat_to_yaw, yaw_to_quat
from robot_goal.voxel_store import VoxelStore
from robot_goal.web_server import encode_png, known_bounds, make_server, occupancy_to_image
from sensor_msgs.msg import PointCloud2

RESULT = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}
MAX_PLAN_POINTS = 300


class WebGoal(Node):

    def __init__(self):
        super().__init__('robot_goal_web')
        p = self.declare_parameter
        self.host = p('host', '0.0.0.0').value
        self.port = p('port', 8080).value
        self.token = p('token', '').value
        self.launcher_port = p('launcher_port', 0).value   # モード選択の画面 (0 なら無し)
        self.frame = p('frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        map_topic = p('map_topic', '/map').value
        plan_topic = p('plan_topic', '/plan').value
        costmap_topics = {'global': p('global_costmap_topic', '/global_costmap/costmap').value,
                          'local': p('local_costmap_topic', '/local_costmap/costmap').value}
        voxel_topic = p('voxel_topic', '/voxel_map').value
        voxel_size = p('voxel_size', 0.1).value

        self.lock = threading.Lock()
        self.map = None            # (meta, png)
        self.map_version = 0
        self.map_received = None   # time.time()
        self.pose = None           # (x, y, yaw, time.time())
        self.plan = []
        self.goal = {'status': 'idle'}
        self.goal_handle = None
        self.goal_seq = 0
        self.costmaps = {name: None for name in costmap_topics}   # name -> {meta, data, png}
        self.costmap_version = 0
        self.voxels = VoxelStore(voxel_size)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, name) for name in p(
                'clear_costmap_services',
                ['/global_costmap/clear_entirely_global_costmap',
                 '/local_costmap/clear_entirely_local_costmap']).value]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.create_subscription(Path, plan_topic, self.on_plan, 1)
        # costmap は volatile で受ける (Nav2 の publisher が transient_local でも受け取れる).
        # 全体を毎回送らせるため Nav2 側は always_send_full_costmap: true (robot_config)
        costmap_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for name, topic in costmap_topics.items():
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, n=name: self.on_costmap(n, m), costmap_qos)
        self.create_subscription(PointCloud2, voxel_topic, self.on_voxels, 1)
        self.create_timer(0.2, self.update_pose)

    # ---------- ROS 側 ----------
    def on_map(self, msg):
        info = msg.info
        if info.width == 0 or info.height == 0:
            return
        png = encode_png(occupancy_to_image(msg.data, info.width, info.height))
        known = known_bounds(msg.data, info.width, info.height, info.resolution,
                             info.origin.position.x, info.origin.position.y)
        with self.lock:
            self.map_version += 1
            meta = {
                'version': self.map_version,
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'known': known,
            }
            self.map = (meta, png)
            self.map_received = time.time()

    def on_costmap(self, name, msg):
        info = msg.info
        if info.width == 0 or info.height == 0:
            return
        with self.lock:
            self.costmap_version += 1
            self.costmaps[name] = {
                'meta': {'version': self.costmap_version, 'width': info.width,
                         'height': info.height, 'resolution': info.resolution,
                         'origin_x': info.origin.position.x, 'origin_y': info.origin.position.y,
                         'known': None},
                'data': bytes(msg.data),
                'png': None,                   # 求められたときに作る (見ていない方は作らない)
                'received': time.time(),
            }

    def on_voxels(self, msg):
        names = {f.name: f for f in msg.fields}
        n = msg.width * msg.height
        if not all(k in names for k in ('x', 'y', 'z', 'label')):
            return
        dtype = np.dtype({'names': ['x', 'y', 'z', 'label'], 'formats': ['<f4', '<f4', '<f4', 'u1'],
                          'offsets': [names[k].offset for k in ('x', 'y', 'z', 'label')],
                          'itemsize': msg.point_step})
        pts = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
        centers = np.column_stack([pts['x'], pts['y'], pts['z']])
        with self.lock:
            self.voxels.update(centers, pts['label'])

    def on_plan(self, msg):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) > MAX_PLAN_POINTS:
            step = math.ceil(len(pts) / MAX_PLAN_POINTS)
            pts = pts[::step] + [pts[-1]]
        with self.lock:
            self.plan = [[round(x, 3), round(y, 3)] for x, y in pts]

    def update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return
        p = t.transform.translation
        with self.lock:
            self.pose = (p.x, p.y, quat_to_yaw(t.transform.rotation), time.time(), p.z)

    def _set_goal(self, seq, **kw):
        """古いゴールのコールバックが新しいゴールの表示を上書きしないよう seq で確かめる."""
        with self.lock:
            if seq == self.goal_seq:
                self.goal.update(kw)

    def on_accepted(self, seq, future):
        handle = future.result()
        if not handle.accepted:
            self._set_goal(seq, status='rejected', message='Nav2 がゴールを受け付けなかった')
            return
        with self.lock:
            if seq == self.goal_seq:
                self.goal_handle = handle
        self._set_goal(seq, status='active')
        handle.get_result_async().add_done_callback(lambda f: self.on_result(seq, f))

    def on_feedback(self, seq, msg):
        fb = msg.feedback
        self._set_goal(seq, distance=round(fb.distance_remaining, 2),
                       eta=fb.estimated_time_remaining.sec,
                       recoveries=fb.number_of_recoveries)

    def on_result(self, seq, future):
        status = future.result().status
        self._set_goal(seq, status=RESULT.get(status, f'status={status}'))
        with self.lock:
            if seq == self.goal_seq:
                self.goal_handle = None

    # ---------- HTTP から呼ばれる ----------
    def get_state(self):
        now = time.time()
        with self.lock:
            m = self.map[0] if self.map else None
            state = {
                'map': dict(m, age=round(now - self.map_received, 1)) if m else None,
                'pose': None,
                'plan': self.plan,
                'goal': dict(self.goal),
                'nav2': self.client.server_is_ready(),
                'frame': self.frame,
                'launcher_port': self.launcher_port,
                'costmaps': {name: (dict(c['meta'], age=round(now - c['received'], 1)) if c else None)
                             for name, c in self.costmaps.items()},
                'voxels': {'version': self.voxels.version, 'count': self.voxels.count(),
                           'size': self.voxels.size},
            }
            if self.pose:
                x, y, yaw, t, z = self.pose
                state['pose'] = {'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3),
                                 'yaw': round(yaw, 4), 'age': round(now - t, 1)}
        return state

    def get_map(self):
        with self.lock:
            return self.map

    def get_costmap(self, name):
        """(meta, png) か None. PNG は版ごとに 1 回だけ作る."""
        with self.lock:
            c = self.costmaps.get(name)
            if c is None:
                return None
            if c['png'] is None:
                m = c['meta']
                c['png'] = encode_png(occupancy_to_image(c['data'], m['width'], m['height']))
            return c['meta'], c['png']

    def get_voxels(self, since):
        with self.lock:
            return self.voxels.payload(since)

    def send_goal(self, x, y, yaw):
        if not self.client.server_is_ready():
            return False, 'Nav2 (navigate_to_pose) が見つからない'
        if yaw is None:
            with self.lock:
                yaw = self.pose[2] if self.pose else 0.0
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        q = goal.pose.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quat(yaw)
        with self.lock:
            self.goal_seq += 1
            seq = self.goal_seq
            self.goal = {'status': 'sending', 'x': round(x, 3), 'y': round(y, 3),
                         'yaw': round(yaw, 4)}
            self.goal_handle = None
        self.get_logger().info(f'ゴール送信: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}deg')
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda m: self.on_feedback(seq, m))
        future.add_done_callback(lambda f: self.on_accepted(seq, f))
        return True, '送信した'

    def clear_costmaps(self):
        """costmap の障害物をすべて消す. 今見えている障害物は次の点群ですぐ付き直る."""
        ready = [c for c in self.clear_clients if c.service_is_ready()]
        if not ready:
            return False, 'costmap の消去サービスが見つからない (Nav2 は起動している?)'
        for c in ready:
            c.call_async(ClearEntireCostmap.Request())
        self.get_logger().info(f'costmap の障害物を消す: {[c.srv_name for c in ready]}')
        return True, '障害物を消した. 見えている障害物はすぐ付き直ります'

    def cancel(self):
        with self.lock:
            handle = self.goal_handle
            if handle is None and self.goal.get('status') == 'sending':
                return False, 'まだ受け付け待ち. 少し待ってからもう一度'
        if handle is None:
            return False, '走行中のゴールは無い'
        handle.cancel_goal_async()
        self.get_logger().info('ゴールを取り消す')
        return True, '取り消しを送った'


def local_addresses():
    """このマシンの IPv4 アドレス (表示用)."""
    addrs = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addrs.add(info[4][0])
    except OSError:
        pass
    try:   # 既定経路のアドレス. UDP の connect はパケットを送らない
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1', 9))
            addrs.add(s.getsockname()[0])
    except OSError:
        pass
    return sorted(a for a in addrs if not a.startswith('127.'))


def main():
    # rclpy の SIGINT ハンドラは context を落としてゴールを取り消せなくなるので使わない (goal.py と同じ)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WebGoal()
    html_path = os.path.join(get_package_share_directory('robot_goal'), 'web', 'index.html')
    with open(html_path, 'rb') as f:
        html = f.read()
    try:
        server = make_server(node.host, node.port, node, html, node.token,
                             static_dir=os.path.join(os.path.dirname(html_path), 'vendor'))
    except OSError as e:
        node.get_logger().error(f'ポート {node.port} を開けない: {e}')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    threading.Thread(target=server.serve_forever, daemon=True).start()
    key = f'/?key={node.token}' if node.token else '/'
    for a in local_addresses() or ['<このマシンの IP>']:
        node.get_logger().info(f'スマホで開く: http://{a}:{node.port}{key}')

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        with node.lock:
            handle = node.goal_handle
        if handle is not None:   # Web を止めたらロボットも止める
            node.get_logger().info('走行中のゴールを取り消す')
            done = threading.Event()
            handle.cancel_goal_async().add_done_callback(lambda f: done.set())
            spin = threading.Thread(target=executor.spin, daemon=True)
            spin.start()
            done.wait(timeout=3.0)
            executor.shutdown(timeout_sec=1.0)
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
