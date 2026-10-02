#!/usr/bin/env python3
"""Estimate the start pose on a saved map and publish it as /initialpose.

While the sensor stands still for a few seconds, scans are accumulated and leveled
with the IMU's gravity. Cells with vertical structure 0.5-2.0 m above the local
ground (walls, furniture, poles; not floor or ceiling) form a top-view image, which
is matched against the same image of the map for every yaw (FFT correlation). The
best candidates are re-scored in 3D and the winner goes to lidar_localization, whose
NDT refines it.
"""
from concurrent.futures import ThreadPoolExecutor
import math
import multiprocessing
import os
import time

import numpy as np

CELL = 0.5             # [m] top-view image resolution
BAND = (0.5, 2.0)      # [m] structure height above local ground
VOXEL_3D = 0.3         # [m] map voxel size for the ICP re-scoring
ICP_POINTS = 5000      # scan points used for ICP


# --- geometry ---------------------------------------------------------------------
def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def level_rotation(gravity):
    """Rotation that turns the measured gravity direction (accelerometer, pointing up) into +z."""
    g = gravity / np.linalg.norm(gravity)
    axis = np.cross(g, [0.0, 0.0, 1.0])
    s, c = np.linalg.norm(axis), float(np.dot(g, [0.0, 0.0, 1.0]))
    if s < 1e-9:
        return np.eye(3)
    k = axis / s
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * kx + (1 - c) * kx @ kx


def quaternion(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


# --- top-view structure image -----------------------------------------------------
def _neighborhood_min(a, radius):
    out = a.copy()
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            out = np.minimum(out, np.roll(np.roll(a, di, 0), dj, 1))
    return out


def structure_image(points, origin, shape):
    """1 where a cell has points BAND above the local ground, dilated by one cell."""
    ij = np.floor((points[:, :2] - origin) / CELL).astype(np.int64)
    ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    ij, z = ij[ok], points[ok, 2]
    low = np.full(shape, np.inf)
    np.minimum.at(low, (ij[:, 0], ij[:, 1]), z)
    ground = _neighborhood_min(low, 2)                  # floor next to a wall counts as its ground
    h = z - ground[ij[:, 0], ij[:, 1]]
    keep = (h > BAND[0]) & (h < BAND[1])
    img = np.zeros(shape, np.float32)
    img[ij[keep, 0], ij[keep, 1]] = 1.0
    return dilate(img)


def dilate(img):
    return np.maximum.reduce([np.roll(np.roll(img, a, 0), b, 1)
                              for a in (-1, 0, 1) for b in (-1, 0, 1)])


def seen_image(points, origin, shape):
    """1 where the sensor saw anything (floor included): the area the local scan can vouch for."""
    ij = np.floor((points[:, :2] - origin) / CELL).astype(np.int64)
    ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    img = np.zeros(shape, np.float32)
    img[ij[ok, 0], ij[ok, 1]] = 1.0
    return dilate(img)



def _pack(k):
    k = k + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def voxel_downsample(points, size):
    _, i = np.unique(_pack(np.floor(points / size).astype(np.int64)), return_index=True)
    return points[i]


class VoxelNN:
    """Approximate nearest neighbour on voxel centroids (numpy only, searches the 27 neighbours)."""

    OFFSETS = np.array([(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)])

    def __init__(self, points, size):
        self.size = size
        keys = _pack(np.floor(points / size).astype(np.int64))
        order = np.argsort(keys)
        keys, points = keys[order], points[order]
        self.keys, start = np.unique(keys, return_index=True)
        counts = np.diff(np.append(start, len(keys)))
        self.centroids = np.add.reduceat(points, start, axis=0) / counts[:, None]

    def query(self, q):
        base = np.floor(q / self.size).astype(np.int64)
        best_d = np.full(len(q), np.inf)
        best_p = np.zeros_like(q)
        for off in self.OFFSETS:
            k = _pack(base + off)
            i = np.clip(np.searchsorted(self.keys, k), 0, len(self.keys) - 1)
            hit = self.keys[i] == k
            d = np.where(hit, np.linalg.norm(self.centroids[i] - q, axis=1), np.inf)
            better = d < best_d
            best_d[better], best_p[better] = d[better], self.centroids[i][better]
        return best_d, best_p


def icp(nn, src, r, t, iterations=(2.0, 2.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.3, 0.3, 0.3)):
    """Point-to-point ICP of src (sensor frame) onto the map. Returns r, t, inlier ratio at 0.3 m."""
    for max_d in iterations:
        p = src @ r.T + t
        d, q = nn.query(p)
        m = d < max_d
        if m.sum() < 10:
            break
        a, b = p[m], q[m]
        ma, mb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a - ma).T @ (b - mb))
        dr = (vt.T @ np.diag([1, 1, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T)
        r, t = dr @ r, dr @ (t - ma) + mb
    d, _ = nn.query(src @ r.T + t)
    return r, t, float((d < 0.3).mean())


# --- matcher ----------------------------------------------------------------------
class MapMatcher:
    def __init__(self, map_points, pad=40.0):
        self.map = map_points
        self.origin = map_points[:, :2].min(0) - pad
        self.shape = tuple(np.ceil((map_points[:, :2].max(0) + pad - self.origin) / CELL).astype(int))
        self.map_fft = np.fft.rfft2(structure_image(map_points, self.origin, self.shape))
        # lowest point within 2 m of each cell: the height the sensor's "ground" is measured from
        ij = np.floor((map_points[:, :2] - self.origin) / CELL).astype(np.int64)
        low = np.full(self.shape, np.inf)
        np.minimum.at(low, (ij[:, 0], ij[:, 1]), map_points[:, 2])
        self.ground = _neighborhood_min(low, int(2.0 / CELL))
        self.nn = VoxelNN(map_points, VOXEL_3D)

    def search(self, local, yaws, peaks=5):
        """local: leveled points around the sensor (sensor at the origin).

        Returns [(score, yaw, xy)] with up to `peaks` separated maxima per yaw.
        """
        sep = int(round(2.0 / CELL))

        def one(yaw):
            p = local @ rot_z(yaw).T
            low = p[:, :2].min(0)
            img = structure_image(p, low, self.shape)
            n = img.sum()
            if n == 0:
                return []
            seen = seen_image(p, low, self.shape)
            # IoU inside the seen area: map structure the scan should have seen also counts
            hit = np.fft.irfft2(self.map_fft * np.conj(np.fft.rfft2(img)), s=self.shape)
            map_in_seen = np.fft.irfft2(self.map_fft * np.conj(np.fft.rfft2(seen)), s=self.shape)
            iou = (hit / np.maximum(n + map_in_seen - hit, 1.0)).ravel()
            out, taken = [], []
            for k in np.argsort(iou)[::-1][:200]:
                ij = np.array(np.unravel_index(k, self.shape))
                if any(np.abs(ij - t).max() < sep for t in taken):
                    continue
                taken.append(ij)
                out.append((iou[k], yaw, ij * CELL + self.origin - low))   # map xy of the sensor
                if len(taken) == peaks:
                    break
            return out

        with ThreadPoolExecutor(4) as pool:              # FFTs release the GIL; more threads don't help
            return [c for cs in pool.map(one, yaws) for c in cs]

    def ground_height(self, xy):
        i, j = np.floor((xy - self.origin) / CELL).astype(int)
        if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1]) or np.isinf(self.ground[i, j]):
            return None
        return float(self.ground[i, j])

    def estimate(self, local, sensor_height, screen=300, top=5):
        """Returns (x, y, z, rotation, score_2d, inlier_ratio, runner_up_inlier_ratio) or None.

        local is leveled. Top-view candidates are screened with a quick ICP on few points, and
        the best few are refined with more points; the one whose scan lands on the map best wins.
        """
        local = voxel_downsample(local, 0.1)        # a few seconds of L2 is ~200k points
        cands = self.search(local, np.radians(np.arange(0, 360, 2.0)))
        cands.sort(key=lambda c: -c[0])
        rng = np.random.default_rng(0)
        few = local[rng.choice(len(local), min(800, len(local)), replace=False)]
        many = local[rng.choice(len(local), min(ICP_POINTS, len(local)), replace=False)]
        global _SCREEN
        _SCREEN = (self, few, sensor_height)
        # many small ICPs hold the GIL, so use forked processes (they share the map copy-on-write)
        n = min(os.cpu_count() or 1, 16)
        chunks = [cands[i:screen:n] for i in range(n)]
        with multiprocessing.get_context('fork').Pool(n) as pool:
            screened = [c for part in pool.map(_screen_chunk, chunks) for c in part]
        screened.sort(key=lambda c: -c[0])
        refined = []
        for _, s2, r, t in screened[:top]:
            r, t, inliers = icp(self.nn, many, r, t)
            refined.append((t[0], t[1], t[2], r, s2, inliers))
        if not refined:
            return None
        refined.sort(key=lambda c: -c[5])
        best = refined[0]
        # runner-up at a clearly different place: how sure are we?
        other = [c[5] for c in refined[1:] if math.hypot(c[0] - best[0], c[1] - best[1]) > 1.0]
        return best + (other[0] if other else 0.0,)


_SCREEN = None


def _screen_chunk(cands):
    """Quick ICP for a slice of candidates (runs in a forked worker)."""
    matcher, few, sensor_height = _SCREEN
    out = []
    for s2, yaw, xy in cands:
        g = matcher.ground_height(xy)
        if g is None:
            continue
        r, t, inliers = icp(matcher.nn, few, rot_z(yaw), np.array([xy[0], xy[1], g + sensor_height]),
                            iterations=(2.0, 1.0, 0.5, 0.3))
        out.append((inliers, s2, r, t))
    return out


# --- ROS node -----------------------------------------------------------------------
def load_pcd_xyz(path):
    """x y z from a binary / ascii PCD (any extra float fields)."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode(errors='ignore').strip()
            key, _, value = line.partition(' ')
            header[key] = value.split()
            if key == 'DATA':
                break
        data = f.read()
    fields, sizes = header['FIELDS'], [int(s) for s in header['SIZE']]
    n = int(header['POINTS'][0])
    if header['DATA'][0] == 'binary':
        if set(header['TYPE']) != {'F'} or set(sizes) != {4}:
            raise ValueError('only float32 PCD fields are supported')
        a = np.frombuffer(data[:n * 4 * len(fields)], dtype=np.float32).reshape(n, len(fields))
    elif header['DATA'][0] == 'ascii':
        a = np.loadtxt(data.decode().splitlines(), dtype=np.float32, ndmin=2)
    else:
        raise ValueError('binary_compressed PCD is not supported; save it as binary')
    return a[:, [fields.index('x'), fields.index('y'), fields.index('z')]].astype(np.float64)


def main():
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2

    class InitialPose(Node):
        def __init__(self):
            super().__init__('l2_initial_pose')
            self.map_path = self.declare_parameter('map_path', '').value
            self.duration = self.declare_parameter('accumulate_sec', 3.0).value
            self.frame_id = self.declare_parameter('global_frame_id', 'map').value
            self.min_score = self.declare_parameter('min_inlier_ratio', 0.5).value
            self.clouds, self.accels, self.start = [], [], None
            self.get_logger().info(f'loading map {self.map_path}')
            t = time.time()
            self.matcher = MapMatcher(load_pcd_xyz(self.map_path))
            self.get_logger().info(f'map ready ({time.time() - t:.1f} s). keep the sensor still '
                                   f'for {self.duration:.0f} s')
            self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
            self.create_subscription(PointCloud2, 'cloud', self.on_cloud, qos_profile_sensor_data)
            self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
            self.done = False

        def on_imu(self, m):
            if self.start is not None and not self.done:
                a = m.linear_acceleration
                self.accels.append((a.x, a.y, a.z))

        def on_cloud(self, m):
            if self.done:
                return
            p = point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
            self.clouds.append(np.stack([p['x'], p['y'], p['z']], 1).astype(np.float64))
            now = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.start = self.start or now
            if now - self.start >= self.duration:
                self.done = True
                self.solve()

        def solve(self):
            local = np.concatenate(self.clouds)
            local = local[np.linalg.norm(local, axis=1) > 0.3]
            if len(self.accels) > 10:
                level = level_rotation(np.mean(self.accels, 0))
            else:
                level = np.eye(3)
                self.get_logger().warn('no IMU data; assuming the sensor is level')
            local = local @ level.T
            near = local[np.hypot(local[:, 0], local[:, 1]) < 2.0, 2]
            sensor_height = -float(np.percentile(near, 2)) if len(near) > 20 else 0.0
            t = time.time()
            best = self.matcher.estimate(local, sensor_height)
            if best is None or best[5] < self.min_score:
                self.get_logger().error(
                    f'start pose not found (best: {best}). set it with 2D Pose Estimate in RViz')
                return
            x, y, z, r, s2, s3, runner_up = best
            yaw = math.atan2(r[1, 0], r[0, 0])
            self.get_logger().info(
                f'start pose: x={x:.2f} y={y:.2f} z={z:.2f} yaw={math.degrees(yaw):.1f} deg '
                f'(inliers {s3:.2f}, runner-up elsewhere {runner_up:.2f}, {time.time() - t:.1f} s)')
            if s3 - runner_up < 0.05:
                self.get_logger().warn('another place fits almost as well; check the pose in RViz '
                                       '(fix it with 2D Pose Estimate if wrong)')
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
            q = quaternion(r @ level)
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
            self.pub.publish(msg)

    rclpy.init()
    node = InitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
