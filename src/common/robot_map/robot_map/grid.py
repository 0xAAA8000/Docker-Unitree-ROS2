"""3D 点群から 2D 占有格子を作る部分 (ROS に依存しない).

各セルで「床に当たった点の数」と「障害物の高さに当たった点の数」を数え,
  * 障害物の点が occupied_min_hits 以上, かつ 床の点の occupied_ratio 倍以上 -> 占有 (100)
  * それ以外で床の点がある                                             -> 空き (0)
  * どちらも無い                                                       -> 未知 (-1)
とする. 人が横切った跡のような一時的な障害物は, その後に床がたくさん見えれば空きに戻る.
格子はロボットが動いた範囲に合わせて chunk [m] 単位で広がる.
"""
import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100

_COUNT_MAX = np.iinfo(np.uint16).max


class Grid:

    def __init__(self, resolution=0.05, chunk=10.0, occupied_min_hits=3, occupied_ratio=0.5):
        self.res = float(resolution)
        self.chunk = max(1, int(round(chunk / self.res)))
        self.min_hits = int(occupied_min_hits)
        self.ratio = float(occupied_ratio)
        # 配列 [0, 0] のセルの, 全体での番号 (x, y). 点が来るまで None
        self.ix0 = None
        self.iy0 = None
        self.floor = np.zeros((0, 0), np.uint16)   # [y, x]
        self.obs = np.zeros((0, 0), np.uint16)
        self.version = 0

    @property
    def shape(self):
        return self.floor.shape

    def origin(self):
        """配列 [0, 0] のセルの角の map 座標 (x, y)."""
        if self.ix0 is None:
            return 0.0, 0.0
        return self.ix0 * self.res, self.iy0 * self.res

    def _ensure(self, ix, iy):
        """全体でのセル番号 ix, iy (配列) がすべて入るように格子を広げる."""
        c = self.chunk
        x_lo, x_hi = (int(ix.min()) // c) * c, (int(ix.max()) // c + 1) * c
        y_lo, y_hi = (int(iy.min()) // c) * c, (int(iy.max()) // c + 1) * c
        if self.ix0 is not None:
            ny, nx = self.floor.shape
            if (x_lo >= self.ix0 and y_lo >= self.iy0
                    and x_hi <= self.ix0 + nx and y_hi <= self.iy0 + ny):
                return
            x_lo, y_lo = min(x_lo, self.ix0), min(y_lo, self.iy0)
            x_hi, y_hi = max(x_hi, self.ix0 + nx), max(y_hi, self.iy0 + ny)
        floor = np.zeros((y_hi - y_lo, x_hi - x_lo), np.uint16)
        obs = np.zeros_like(floor)
        if self.ix0 is not None:
            ny, nx = self.floor.shape
            sy, sx = self.iy0 - y_lo, self.ix0 - x_lo
            floor[sy:sy + ny, sx:sx + nx] = self.floor
            obs[sy:sy + ny, sx:sx + nx] = self.obs
        self.floor, self.obs = floor, obs
        self.ix0, self.iy0 = x_lo, y_lo

    def add(self, floor_xy, obs_xy):
        """床の点と障害物の点 (どちらも (N, 2) の map 座標) を数える."""
        parts = [p for p in (floor_xy, obs_xy) if len(p)]
        if not parts:
            return
        idx = [np.floor(np.asarray(p, np.float64) / self.res).astype(np.int64) for p in parts]
        allidx = np.concatenate(idx)
        self._ensure(allidx[:, 0], allidx[:, 1])
        for xy, counts in ((floor_xy, self.floor), (obs_xy, self.obs)):
            if not len(xy):
                continue
            i = np.floor(np.asarray(xy, np.float64) / self.res).astype(np.int64)
            flat = (i[:, 1] - self.iy0) * counts.shape[1] + (i[:, 0] - self.ix0)
            cells, n = np.unique(flat, return_counts=True)
            view = counts.reshape(-1)
            view[cells] = np.minimum(view[cells].astype(np.int64) + n, _COUNT_MAX)
        self.version += 1

    def occupancy(self):
        """(高さ, 幅) の int8 配列. 行 0 が y の小さい側 (nav_msgs/OccupancyGrid と同じ並び)."""
        out = np.full(self.floor.shape, UNKNOWN, np.int8)
        out[self.floor > 0] = FREE
        occ = (self.obs >= self.min_hits) & (self.obs >= self.ratio * self.floor)
        out[occ] = OCCUPIED
        return out


def classify(z, ground_z, floor_tolerance, obstacle_min_height, obstacle_max_height):
    """高さ z から (床か, 障害物か) の bool 配列を返す. 高さは ground_z からの差で判定する."""
    h = np.asarray(z) - ground_z
    floor = np.abs(h) <= floor_tolerance
    obs = (h >= obstacle_min_height) & (h < obstacle_max_height)
    return floor, obs


def estimate_ground(z, bin_size=0.05):
    """点の高さの分布で一番多い高さを床とみなす (保存済み PCD 用).

    LiDAR を前に傾けて付けているので, 床の点が一番多いはず.
    """
    z = np.asarray(z, np.float64)
    z = z[np.isfinite(z)]
    if not len(z):
        return 0.0
    lo = np.floor(z.min() / bin_size) * bin_size
    nbins = int(np.ceil((z.max() - lo) / bin_size)) + 1
    hist, edges = np.histogram(z, bins=nbins, range=(lo, lo + nbins * bin_size))
    k = int(np.argmax(hist))
    # 区切りの位置による誤差を減らすため, 一番多い区間の前後の点の中央値をとる
    near = z[(z >= edges[k] - bin_size) & (z < edges[k + 1] + bin_size)]
    return float(np.median(near))


# PointCloud2 の datatype: FLOAT32 = 7, FLOAT64 = 8
_DTYPES = {7: 'f4', 8: 'f8'}


def cloud_xyz(fields, point_step, data, is_bigendian=False):
    """PointCloud2 の中身から (N, 3) float64 の x y z を取り出す. 有限でない点は除く.

    fields は (name, offset, datatype) の並び.
    """
    info = {name: (off, dt) for name, off, dt in fields}
    buf = np.frombuffer(data, np.uint8)
    n = len(buf) // point_step if point_step else 0
    if n == 0:
        return np.zeros((0, 3))
    buf = buf[:n * point_step].reshape(n, point_step)
    out = np.empty((n, 3))
    for k, name in enumerate('xyz'):
        if name not in info:
            raise ValueError(f'点群に {name} が無い')
        off, dt = info[name]
        if dt not in _DTYPES:
            raise ValueError(f'{name} の型 (datatype={dt}) には対応していない')
        t = np.dtype(_DTYPES[dt]).newbyteorder('>' if is_bigendian else '<')
        out[:, k] = buf[:, off:off + t.itemsize].copy().view(t)[:, 0]
    return out[np.isfinite(out).all(axis=1)]


def read_pcd_xyz(path):
    """PCD (ascii / binary) から (N, 3) float64 の x y z を読む. binary_compressed は不可."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'{path}: DATA 行が無い')
            key, _, value = line.decode(errors='ignore').strip().partition(' ')
            header[key] = value.split()
            if key == 'DATA':
                break
        data = f.read()
    fields = header['FIELDS']
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
    sizes = [int(s) for s in header['SIZE']]
    types = header['TYPE']
    n = int(header['POINTS'][0])
    kind = header['DATA'][0]
    if kind == 'ascii':
        cols = []
        col = 0
        for name, c in zip(fields, counts):
            cols.append(col)
            col += c
        a = np.loadtxt(data.decode().splitlines(), dtype=np.float64, ndmin=2)
        out = np.stack([a[:, cols[fields.index(k)]] for k in 'xyz'], axis=1)
    elif kind == 'binary':
        step = sum(s * c for s, c in zip(sizes, counts))
        layout = []
        off = 0
        for name, s, t, c in zip(fields, sizes, types, counts):
            if name in ('x', 'y', 'z'):
                if t != 'F' or s not in (4, 8):
                    raise ValueError(f'{path}: {name} が float ではない')
                layout.append((name, off, 7 if s == 4 else 8))
            off += s * c
        out = cloud_xyz(layout, step, data[:n * step])
        return out
    else:
        raise ValueError(f'{path}: DATA {kind} には対応していない (ascii か binary で保存し直す)')
    return out[np.isfinite(out).all(axis=1)]
