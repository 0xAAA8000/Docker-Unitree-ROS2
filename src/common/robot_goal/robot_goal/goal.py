"""コマンドラインから Nav2 (NavigateToPose) に目的地を送る.

  ros2 run robot_goal goal 3.0 1.5          # map 座標 (3.0, 1.5) へ. 向きは今の向きのまま
  ros2 run robot_goal goal 3.0 1.5 90       # 到着時の向き [deg] (map の x 軸から反時計回り)
  ros2 run robot_goal goal --rel 2.0 0      # 今の位置・向きから見て前に 2 m
  ros2 run robot_goal goal                  # 対話モード (下の help を参照)

1 回だけ送るモードでは, 到着・失敗まで進み具合を表示する. Ctrl-C でゴールを取り消して終了する.
"""
import argparse
import math
import sys
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
import tf2_ros

HELP = """コマンド
  X Y [YAW]      map 座標 (X, Y) [m] へ. YAW [deg] を省くと今の向き
  r DX DY [DYAW] 今の位置・向きから見て前 DX, 左 DY [m] へ
  c              走行中のゴールを取り消す
  p              今の位置と向きを表示
  h              このヘルプ
  q              取り消して終了"""

STATUS = {
    GoalStatus.STATUS_SUCCEEDED: '到着',
    GoalStatus.STATUS_ABORTED: '失敗 (Nav2 が中止)',
    GoalStatus.STATUS_CANCELED: '取り消し',
}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalClient(Node):

    def __init__(self, frame, base_frame):
        super().__init__('robot_goal')
        self.frame = frame
        self.base_frame = base_frame
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.goal_handle = None
        self.done = threading.Event()
        self.done.set()
        self.last_feedback = 0.0
        self.verbose = True

    def pose(self):
        """map 座標での今の (x, y, yaw). TF が無ければ None."""
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.base_frame, Time(),
                                                timeout=Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'{self.frame} -> {self.base_frame} の TF が取れない: {e}')
            return None
        p = t.transform.translation
        return p.x, p.y, quat_to_yaw(t.transform.rotation)

    def send(self, x, y, yaw):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose が見つからない. Nav2 は起動している?')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        q = goal.pose.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quat(yaw)
        print(f'ゴール送信: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}deg ({self.frame})')
        self.done.clear()
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_accepted)
        return True

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            print('ゴールが受け付けられなかった')
            self.done.set()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, msg):
        fb = msg.feedback
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.verbose or now - self.last_feedback < 2.0:
            return
        self.last_feedback = now
        eta = fb.estimated_time_remaining.sec
        print(f'  残り {fb.distance_remaining:.2f} m, 約 {eta} s, リカバリ {fb.number_of_recoveries} 回')

    def on_result(self, future):
        status = future.result().status
        print(f'結果: {STATUS.get(status, f"status={status}")}')
        self.goal_handle = None
        self.done.set()

    def cancel(self):
        if self.goal_handle is None:
            return False
        self.goal_handle.cancel_goal_async()
        return True


def parse_goal(node, nums, relative):
    """[X, Y, (YAW_deg)] -> map 座標の (x, y, yaw[rad]). 失敗なら None."""
    if len(nums) not in (2, 3):
        print('数値は X Y [YAW] の 2 個か 3 個')
        return None
    yaw_deg = nums[2] if len(nums) == 3 else None
    if not relative and yaw_deg is not None:
        return nums[0], nums[1], math.radians(yaw_deg)
    cur = node.pose()
    if cur is None:
        return None
    cx, cy, cyaw = cur
    if not relative:
        return nums[0], nums[1], cyaw
    dx, dy = nums[0], nums[1]
    x = cx + dx * math.cos(cyaw) - dy * math.sin(cyaw)
    y = cy + dx * math.sin(cyaw) + dy * math.cos(cyaw)
    return x, y, cyaw + math.radians(yaw_deg or 0.0)


def interactive(node):
    node.verbose = False   # 入力の邪魔をしないよう, 対話モードでは途中経過を出さない
    print(HELP)
    while rclpy.ok():
        try:
            line = input('goal> ').split()
        except EOFError:
            line = ['q']
        if not line:
            continue
        cmd, rest = line[0], line[1:]
        if cmd == 'q':
            node.cancel()
            return
        if cmd == 'h':
            print(HELP)
        elif cmd == 'c':
            print('取り消し要求を送った' if node.cancel() else '走行中のゴールは無い')
        elif cmd == 'p':
            cur = node.pose()
            if cur:
                print(f'x={cur[0]:.2f} y={cur[1]:.2f} yaw={math.degrees(cur[2]):.0f}deg')
        else:
            relative = cmd == 'r'
            try:
                nums = [float(v) for v in (rest if relative else line)]
            except ValueError:
                print('分からないコマンド. h でヘルプ')
                continue
            target = parse_goal(node, nums, relative)
            if target:
                node.send(*target)


def main():
    ap = argparse.ArgumentParser(description='Nav2 に目的地を送る (引数なしで対話モード)')
    ap.add_argument('values', nargs='*', type=float, metavar='X Y [YAW_DEG]')
    ap.add_argument('--rel', action='store_true', help='今の位置・向きから見た相対座標 (前 X, 左 Y)')
    ap.add_argument('--frame', default='map', help='ゴールの座標系 (既定: map)')
    ap.add_argument('--base-frame', default='base_link', help='ロボットの座標系 (既定: base_link)')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    # rclpy の SIGINT ハンドラは context を落としてゴールを取り消せなくなるので使わない
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GoalClient(args.frame, args.base_frame)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if args.values:
            target = parse_goal(node, args.values, args.rel)
            if target and node.send(*target):
                node.done.wait()
        else:
            interactive(node)
    except KeyboardInterrupt:
        if node.cancel():
            print('\nゴールを取り消す')
            node.done.wait(timeout=3.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
