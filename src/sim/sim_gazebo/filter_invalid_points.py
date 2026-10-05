import array
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

# 1スキャン内に割り振る時刻の幅 [s]。gz は全点同時刻なので小さくする（グループ分けが目的）
TIME_SPREAD = 1.0e-3

# 自車体の点を除去する箱（LiDAR 座標系 [m]）
# 車体: base_link 基準で x ±0.6, y ±0.325。LiDAR は base_link の (+0.2, 0, +0.25) にあり、地面は LiDAR の -0.35
SELF_BOX_MIN = np.array([-0.85, -0.40, -0.45])
SELF_BOX_MAX = np.array([ 0.45,  0.40,  0.10])


class FilterInvalidPoints(Node):
    """inf/NaN 点を除去し、方位角から time フィールドを付けて再配信する"""

    def __init__(self):
        super().__init__('filter_invalid_points')
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
        in_self = ((x > SELF_BOX_MIN[0]) & (x < SELF_BOX_MAX[0]) &
                   (y > SELF_BOX_MIN[1]) & (y < SELF_BOX_MAX[1]) &
                   (z > SELF_BOX_MIN[2]) & (z < SELF_BOX_MAX[2]))
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

        out = PointCloud2()
        out.header = msg.header
        out.fields = fields + [
            PointField(name='time', offset=msg.point_step, datatype=PointField.FLOAT32, count=1)
        ]
        out.is_bigendian = msg.is_bigendian
        out.point_step = out_step
        out.height = 1
        out.width = len(out_pts)
        out.row_step = out.width * out_step
        out.data = array.array('B', out_pts.tobytes())   # bytes を直接渡すと setter が遅い
        out.is_dense = True
        self.pub.publish(out)


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