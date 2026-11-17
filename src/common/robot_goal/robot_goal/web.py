"""スマホのブラウザから地図をタップして Nav2 に目的地を送る Web サーバー.

  ros2 run robot_goal web
  ros2 run robot_goal web --ros-args -p port:=8080 -p token:=abcd   # ?key=abcd を付けた人だけ操作できる

同じ Wi-Fi のスマホで http://<このマシンの IP>:8080 を開く.
地図は自分では作らず, robot_map が配信する /map を表示する.

購読: /map (2D 地図), /plan (Nav2 の経路), TF (map -> base_link)
送信: navigate_to_pose アクション (robot_goal goal と同じ)
"""
import math
import os
import socket
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
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
from robot_goal.web_server import encode_png, known_bounds, make_server, occupancy_to_image

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
        self.frame = p('frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        map_topic = p('map_topic', '/map').value
        plan_topic = p('plan_topic', '/plan').value

        self.lock = threading.Lock()
        self.map = None            # (meta, png)
        self.map_version = 0
        self.map_received = None   # time.time()
        self.pose = None           # (x, y, yaw, time.time())
        self.plan = []
        self.goal = {'status': 'idle'}
        self.goal_handle = None
        self.goal_seq = 0

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.create_subscription(Path, plan_topic, self.on_plan, 1)
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
            self.pose = (p.x, p.y, quat_to_yaw(t.transform.rotation), time.time())

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
            }
            if self.pose:
                x, y, yaw, t = self.pose
                state['pose'] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4),
                                 'age': round(now - t, 1)}
        return state

    def get_map(self):
        with self.lock:
            return self.map

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
        server = make_server(node.host, node.port, node, html, node.token)
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
