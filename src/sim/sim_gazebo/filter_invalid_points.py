import array
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

# 1スキャン内に割り振る時刻の幅 [s]。gz は全点同時刻なので小さくする（グループ分けが目的）
TIME_SPREAD = 1.0e-3

# 自車体の点を除去する箱（LiDAR 座標系 [m]）は self_box_min / self_box_max パラメータで受け取る.
# bridge.launch.py が robot_config (robots/sim.yaml の body.self_box_*. base_link 座標) を LiDAR 座標に直して渡す


class FilterInvalidPoints(Node):
    """inf/NaN 点と自車体の点を除去し、方位角から time フィールドを付けて /lidar に再配信する。
    障害物の判定は実機と同じ obstacle_detector が /lidar から行う (bridge.launch.py が起動)。
    実機(L1, L2)では無効点は配信されず、また時間フィールドも一緒に送られてくるので、lidar_bridge は名前の付け替えだけ"""

    def __init__(self):
        super().__init__('filter_invalid_points')
        self.self_min = np.array(self.declare_parameter('self_box_min', [-0.85, -0.40, -0.45]).value)
        self.self_max = np.array(self.declare_parameter('self_box_max', [0.45, 0.40, 0.10]).value)
        self.sub = self.create_subscription(PointCloud2, '/lidar_raw', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/lidar', 10)

    def cb(self, msg: PointCloud2):
        fields = [f for f in msg.fields if f.name != 'time']
        dtype = np.dtype({
            'names': [f.name for f in fields],
            'formats': [_DTYPES[f.datatype] for f in fields],
            'offsets': [f.offset for f in fields],
            'itemsize': msg.point_step,
        })
        if msg.is_bigendian:
            dtype = dtype.newbyteorder('>')

        n_row = msg.width * msg.point_step
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.row_step)[:, :n_row]
        pts = np.frombuffer(buf.tobytes(), dtype=dtype)

        x, y, z = pts['x'], pts['y'], pts['z']
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        lo, hi = self.self_min, self.self_max
        in_self = ((x > lo[0]) & (x < hi[0]) &
                   (y > lo[1]) & (y < hi[1]) &
                   (z > lo[2]) & (z < hi[2]))
        pts = pts[valid & ~in_self]

        # 末尾に time(float32) を追加したレイアウト
        out_step = msg.point_step + 4
        out_dtype = np.dtype({
            'names': list(dtype.names) + ['time'],
            'formats': [dtype.fields[n][0] for n in dtype.names] + ['f4'],
            'offsets': [dtype.fields[n][1] for n in dtype.names] + [msg.point_step],
            'itemsize': out_step,
        })
        out_pts = np.zeros(len(pts), dtype=out_dtype)
        for name in dtype.names:
            out_pts[name] = pts[name]
        azimuth = np.arctan2(pts['y'], pts['x'])                     # [-pi, pi]
        out_pts['time'] = (azimuth + np.pi) / (2 * np.pi) * TIME_SPREAD  # [0, TIME_SPREAD] 秒

        out_fields = fields + [
            PointField(name='time', offset=msg.point_step, datatype=PointField.FLOAT32, count=1)
        ]
        self.pub.publish(self.make_cloud(msg, out_fields, out_step, out_pts))

    @staticmethod
    def make_cloud(src, fields, point_step, pts):
        out = PointCloud2()
        out.header = src.header
        out.fields = fields
        out.is_bigendian = src.is_bigendian
        out.point_step = point_step
        out.height = 1                      # 穴を詰めたので unorganized にする
        out.width = len(pts)
        out.row_step = out.width * point_step
        out.data = array.array('B', pts.tobytes())   # bytes を直接渡すと setter が遅い
        out.is_dense = True
        return out


def main():
    rclpy.init()
    node = FilterInvalidPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()