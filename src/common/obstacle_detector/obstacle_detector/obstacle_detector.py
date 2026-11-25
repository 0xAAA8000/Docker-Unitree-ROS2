"""LiDAR の点群から Nav2 の障害物とクリア用の点群を作るノード.

  /lidar_full (lidar_bridge: frame_id を付け替えただけの点群)
    --(自車体・近すぎる点を除き, 地面からの高さで抽出し, 坂を除く)--> /lidar_obstacles  (Nav2 の障害物)
    --(床と坂の点を少し奥へずらす)----------------------------------> /lidar_clear      (Nav2 のクリア用)
    --(坂として障害物から外した点)----------------------------------> /lidar_slope      (確認用)

判定は base_link 座標で行う. LiDAR の取付位置・姿勢と自車体の箱は lidar_bridge と同じ値を
real.launch.py が robot.yaml の lidar_bridge の項から渡す. 配信する点の座標は LiDAR 座標のまま.

高さによる判定
  地面 (ground_z) からの高さが obstacle_min_height より高く obstacle_max_height より低い点を障害物とする.
  LiDAR から obstacle_range_min 未満の点と, 自車体の箱 (self_box_*) の中の点は使わない.

坂の判定 (slope_filter が true のとき)
  高さだけで判定すると, 緩い坂も地面から obstacle_min_height 以上になった所から障害物になる.
  そこで車体の周り slope_range [m] の点を, 上から見た slope_cell [m] のマスに分けて
  マスごとの高さ (平均) と厚み (最高 - 最低) を求め, 「床から坂を伝ってたどり着ける」マスを床から広げる.
    * 出発点ははっきり床と言えるマス (点が slope_min_points 個以上, 高さ < obstacle_min_height の半分,
      厚みが小さい). 点が 1 つのマスは, ばらつきで低く出ると段の上でも床に見えるため使わない
    * 進めるのは隣 (8 方向) のマスで, 高さの差が step_max 以下 (段差の上限)
    * 進んだ先の床以外のマスは, 点が slope_min_points 個以上, 厚みが小さく (壁や箱の側面は厚い),
      傾きが slope_max_deg 以下
  マスの傾きは, マスごとに独立して求める: そのマスを中心とする 5 x 5 マスのうち使えるマス
  (床か, 点が slope_min_points 個以上で厚みが小さい) の (点の重心 x, y, 平均の高さ) に
  平面 z = a x + b y + d を最小二乗で当てはめ, 傾き = atan(hypot(a, b)). 隣のマスの傾きは使わない
  (窓が重なるので結果は滑らかになる). 使えるマスが 4 つ未満か一直線に並ぶときは傾きを決めず, 通れないとする.
  たどり着けるマスの障害物の点は坂として障害物から外す. 箱や台は上面が平らでも, 縁の段差で
  たどり着けないので障害物のまま. 点がまばらで途切れた坂も (安全側に) 障害物のまま. 範囲の外は高さだけで判定する.
  高さは base_link 基準 (車体が傾くと一緒に傾く) なので, 傾きは車体に対する値になる.

クリア用の点群 (/lidar_clear)
  Nav2 の costmap は LiDAR から点までの線上のセルを空きにするが, 点のあるセル自体は空きにしない.
  床の上に一度残った誤検知 (自己位置のずれなどで付いた障害物) は, その床の点が届いても消えない.
  そこで床と坂の点だけを, LiDAR から見た水平方向に ground_clear_extend [m] 奥へずらして配信する.
  本当の障害物は同じ更新で付け直されるので消えない (Nav2 はクリアのあとに障害物を付ける)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from lidar_bridge.lidar_bridge_node import cloud_to_array, make_cloud, rotation_rpy


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        p = self.declare_parameter
        cloud_in = p('cloud_in', '/lidar_full').value
        obstacles_out = p('obstacles_out', '/lidar_obstacles').value
        clear_out = p('clear_out', '/lidar_clear').value
        slope_out = p('slope_out', '/lidar_slope').value

        # base_link から見た LiDAR の取付位置 [m] と姿勢 [deg] (lidar_bridge と同じ値)
        t = [p('lidar_x', 0.2).value, p('lidar_y', 0.0).value, p('lidar_z', 0.25).value]
        rpy = [math.radians(p(f'lidar_{k}_deg', 0.0).value) for k in ('roll', 'pitch', 'yaw')]
        self.rot = rotation_rpy(*rpy)
        self.trans = np.array(t)
        self.self_min = np.array(p('self_box_min', [-0.4, -0.25, -0.2]).value, dtype=float)
        self.self_max = np.array(p('self_box_max', [0.4, 0.25, 0.4]).value, dtype=float)

        # 高さによる判定 (base_link 座標)
        self.ground_z = p('ground_z', -0.1).value                    # 地面の z [m]
        self.obstacle_min = p('obstacle_min_height', 0.15).value    # 地面からの高さ [m]
        self.obstacle_max = p('obstacle_max_height', 0.5).value
        self.range_min = p('obstacle_range_min', 0.08).value         # LiDAR からの距離 [m]
        self.ground_extend = p('ground_clear_extend', 0.1).value    # 床の点をずらす距離 [m] (0 で無効)

        # 坂の判定
        self.slope_filter = p('slope_filter', True).value
        self.slope_tan = math.tan(math.radians(p('slope_max_deg', 15.0).value))
        self.step_max = p('step_max', 0.06).value               # 隣のマスとの高さの差の上限 (段差) [m]
        self.slope_cell = p('slope_cell', 0.15).value            # マスの大きさ [m]
        self.slope_range = p('slope_range', 4.0).value           # 判定する範囲 (base_link から ±) [m]
        self.slope_min_pts = p('slope_min_points', 3).value

        self.pub_obs = self.create_publisher(PointCloud2, obstacles_out, 10)
        self.pub_clear = self.create_publisher(PointCloud2, clear_out, 10)
        self.pub_slope = self.create_publisher(PointCloud2, slope_out, 10)
        self.create_subscription(PointCloud2, cloud_in, self.on_cloud, 10)
        self.get_logger().info(
            f'{cloud_in} -> {obstacles_out}, {clear_out}  height {self.obstacle_min}..{self.obstacle_max} m  '
            f'range_min {self.range_min} m  slope_filter={self.slope_filter} '
            f'(max {math.degrees(math.atan(self.slope_tan)):.0f}deg, step {self.step_max} m)')

    def on_cloud(self, msg):
        pts = cloud_to_array(msg)
        xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        finite = np.isfinite(xyz).all(axis=1)
        xyz[~finite] = 0.0
        base = xyz @ self.rot.T + self.trans        # LiDAR 座標 -> base_link 座標
        in_self = np.all((base > self.self_min) & (base < self.self_max), axis=1)
        keep = finite & ~in_self & (np.einsum('ij,ij->i', xyz, xyz) >= self.range_min ** 2)

        h = base[:, 2] - self.ground_z              # 地面からの高さ
        obs = keep & (h > self.obstacle_min) & (h < self.obstacle_max)
        floor = h <= self.obstacle_min              # 床 (クリア用に奥へずらす点)
        if self.slope_filter:
            slope = obs & self.traversable(base, h, keep & (h < self.obstacle_max))
            obs &= ~slope
            floor |= slope
            if self.pub_slope.get_subscription_count() > 0:
                self.pub_slope.publish(make_cloud(msg, pts[slope], msg.header.frame_id))
        self.pub_obs.publish(make_cloud(msg, pts[obs], msg.header.frame_id))
        if self.pub_clear.get_subscription_count() > 0:
            self.pub_clear.publish(self.clear_cloud(msg, pts, base, keep, floor))

    def traversable(self, base, h, cand):
        """各点が「床から坂を伝ってたどり着けるマス」にあるか (ファイル先頭の説明)."""
        c, r = self.slope_cell, self.slope_range
        w = int(math.ceil(2.0 * r / c))
        sel = cand & (np.abs(base[:, 0]) < r) & (np.abs(base[:, 1]) < r)
        out = np.zeros(len(h), dtype=bool)
        if not sel.any():
            return out
        ix = np.clip(((base[sel, 0] + r) / c).astype(int), 0, w - 1)
        iy = np.clip(((base[sel, 1] + r) / c).astype(int), 0, w - 1)
        cell = ix * w + iy
        hs = h[sel]
        cnt = np.bincount(cell, minlength=w * w)
        div = np.maximum(cnt, 1)
        z = np.bincount(cell, weights=hs, minlength=w * w) / div               # マスの高さ
        x = np.bincount(cell, weights=base[sel, 0], minlength=w * w) / div     # 点の重心
        y = np.bincount(cell, weights=base[sel, 1], minlength=w * w) / div
        zmin = np.full(w * w, np.inf)
        zmax = np.full(w * w, -np.inf)
        np.minimum.at(zmin, cell, hs)
        np.maximum.at(zmax, cell, hs)
        thin = (cnt > 0) & (zmax - zmin <= c * math.sqrt(2.0) * self.slope_tan + self.step_max)
        ground = thin & (z < self.obstacle_min)
        dense = thin & (cnt >= self.slope_min_pts)
        z, x, y, ground, dense = (a.reshape(w, w) for a in (z, x, y, ground, dense))

        def shift(a, dx, dy, fill):
            """b[i, j] = a[i + dx, j + dy] (範囲外は fill)."""
            b = np.full_like(a, fill)
            b[max(0, -dx):w - max(0, dx), max(0, -dy):w - max(0, dy)] = \
                a[max(0, dx):w - max(0, -dx), max(0, dy):w - max(0, -dy)]
            return b

        # 周り 5 x 5 マスの (x, y, z) に平面 z = a x + b y + d を最小二乗で当てはめる
        usable = (ground | dense).astype(float)
        names = ('n', 'x', 'y', 'z', 'xx', 'xy', 'yy', 'xz', 'yz')
        x0, y0 = x * usable, y * usable     # 使わないマスは 0 にする (重み 0)
        terms = dict(zip(names, (usable, x0, y0, z * usable, x0 * x, x0 * y, y0 * y, x0 * z, y0 * z)))
        sums = {k: np.zeros((w, w)) for k in names}
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                for k in names:
                    sums[k] += shift(terms[k], dx, dy, 0.0)
        n_ = np.maximum(sums['n'], 1.0)
        mx, my, mz = sums['x'] / n_, sums['y'] / n_, sums['z'] / n_
        sxx = sums['xx'] / n_ - mx * mx
        sxy = sums['xy'] / n_ - mx * my
        syy = sums['yy'] / n_ - my * my
        sxz = sums['xz'] / n_ - mx * mz
        syz = sums['yz'] / n_ - my * mz
        det = sxx * syy - sxy * sxy
        ok_fit = (sums['n'] >= 4) & (det > (0.25 * c * c) ** 2)   # 点が一直線に並ぶときは傾きを決めない
        safe = np.where(ok_fit, det, 1.0)
        ga = (sxz * syy - syz * sxy) / safe
        gb = (syz * sxx - sxz * sxy) / safe
        gentle = ok_fit & (np.hypot(ga, gb) <= self.slope_tan)
        passable = ground | (dense & gentle)

        # 床から, 隣との段差が step_max 以下で通れるマスを広げる
        links = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx or dy:
                    ok = passable & shift(passable, dx, dy, False) & \
                        (np.abs(z - shift(z, dx, dy, np.inf)) <= self.step_max)
                    links.append((dx, dy, ok))
        reach = dense & (z < 0.5 * self.obstacle_min)   # 出発点
        for _ in range(w * 2):
            grown = reach.copy()
            for dx, dy, ok in links:
                grown |= ok & shift(reach, dx, dy, False)
            if np.array_equal(grown, reach):
                break
            reach = grown
        out[np.flatnonzero(sel)] = reach.ravel()[cell]
        return out

    def clear_cloud(self, msg, pts, base, mask, floor):
        """Nav2 のクリア用の点群. 床 (坂を含む) の点だけ LiDAR から水平方向に ground_clear_extend 奥へずらす."""
        sel = pts[mask].copy()
        if self.ground_extend > 0.0 and len(sel):
            b = base[mask].copy()
            ground = floor[mask]
            d = b[ground, :2] - self.trans[:2]
            n = np.linalg.norm(d, axis=1, keepdims=True)
            b[ground, :2] += d / np.maximum(n, 1e-6) * self.ground_extend
            xyz = (b - self.trans) @ self.rot          # base_link 座標 -> LiDAR 座標
            sel['x'], sel['y'], sel['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        return make_cloud(msg, sel, msg.header.frame_id)



def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
