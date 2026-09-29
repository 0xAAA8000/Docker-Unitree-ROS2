import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudTimeAdder(Node):
    def __init__(self):
        super().__init__('pointcloud_time_adder')
        
        # Gazeboからの入力トピック名
        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar',
            self.cb,
            10
        )
        
        # Point-LIOに入力する出力トピック名
        self.pub = self.create_publisher(
            PointCloud2,
            '/lidar_with_time',
            10
        )

    def cb(self, msg):
        # 既存フィールド名を取得
        field_names = [f.name for f in msg.fields]
        
        # 点データを読み込み
        points = list(pc2.read_points(msg, field_names=field_names, skip_nans=False))
        
        # 各点の末尾に time = 0.0 を追加
        new_points = [list(p) + [0.0] for p in points]
        
        # フィールド定義に 'time' (FLOAT32) を追加
        new_fields = list(msg.fields) + [
            PointField(name='time', offset=msg.point_step, datatype=PointField.FLOAT32, count=1)
        ]
        
        # 新しい PointCloud2 メッセージを作成して配信
        new_msg = pc2.create_cloud(msg.header, new_fields, new_points)
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = PointCloudTimeAdder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
