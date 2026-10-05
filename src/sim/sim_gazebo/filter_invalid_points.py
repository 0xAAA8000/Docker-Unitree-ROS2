import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# PointField.datatype -> numpy 型
_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


class FilterInvalidPoints(Node):
    """x/y/z が inf・NaN の点を除去し、is_dense=true の点群として再配信する"""

    def __init__(self):
        super().__init__('filter_invalid_points')
        self.sub = self.create_subscription(PointCloud2, '/lidar_raw', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/lidar', 10)

    def cb(self, msg: PointCloud2):
        # 受信したフィールド定義どおりの構造化 dtype を作る（パディングもそのまま保持）
        dtype = np.dtype({
            'names': [f.name for f in msg.fields],
            'formats': [_DTYPES[f.datatype] for f in msg.fields],
            'offsets': [f.offset for f in msg.fields],
            'itemsize': msg.point_step,
        })
        if msg.is_bigendian:
            dtype = dtype.newbyteorder('>')

        # 行ごとのパディングがあっても読めるよう row_step 単位で切り出す
        n_row = msg.width * msg.point_step
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.row_step)[:, :n_row]
        pts = np.frombuffer(buf.tobytes(), dtype=dtype)

        valid = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
        pts = pts[valid]

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.height = 1                      # 穴を詰めたので unorganized にする
        out.width = len(pts)
        out.row_step = out.width * out.point_step
        out.data = pts.tobytes()
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