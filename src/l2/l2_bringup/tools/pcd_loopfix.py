"""Approximate loop correction for a Point-LIO map saved without a recording.

Point-LIO has no loop closure, so after walking a loop the end of the map does not meet
the start (e.g. several metres in height). Without the trajectory, this uses the fact that
points are saved in time order:

  1. take the start (--start, % of the file) and the end (--end) of the map
  2. search the shift (yaw, x, y, z grid + ICP) that lays the end onto the start
  3. spread that shift over the file in proportion to time (each 1/N piece moved a bit)

The assumption "drift grows in proportion to time" is often wrong (drift can jump), and
then the corrected map is less consistent even if it looks flatter. The script therefore
compares the number of occupied 0.1 m voxels before and after (fewer = sharper) and says
whether to use the result.

  python3 pcd_loopfix.py TRIMMED.pcd -o OUT.pcd [--start 0:10] [--end 93:100]

Run it on the output of pcd_trim.py (order must be kept), before pcd_sor.sh.
"""
import argparse
import math
import multiprocessing
import os
import sys
import time

import numpy as np

from pcd_io import read_pcd, write_pcd

HERE = os.path.dirname(os.path.abspath(__file__))
for d in (os.path.join(HERE, '..', 'scripts'),                       # l2_bringup/tools
          os.path.join(HERE, '..', 'src', 'l2_bringup', 'scripts')):  # ~/ros2_ws/tools
    sys.path.insert(0, d)
import l2_initial_pose as ip  # noqa: E402  (voxel NN + ICP)

_JOB = None


def _score_block(args):
    yaw, dz = args
    nn, src, c, xs, ys = _JOB
    r = ip.rot_z(yaw)
    out = []
    for dx in xs:
        for dy in ys:
            d, _ = nn.query((src - c) @ r.T + c + [dx, dy, dz])
            out.append(((d < 0.3).mean(), yaw, dx, dy, dz))
    return out


def occupied_voxels(p, size=0.1):
    k = np.floor(p / size).astype(np.int64)
    k -= k.min(0)
    return len(np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]))


def rot_axis(axis, angle):
    k = axis
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def main():
    global _JOB
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--start', default='0:10', help='start part [%%] e.g. 0:10')
    ap.add_argument('--end', default='93:100', help='end part [%%] e.g. 93:100')
    ap.add_argument('--z-range', default='-10:10', help='[m] height shift to search')
    ap.add_argument('--xy-range', type=float, default=6.0, help='[m] +- horizontal shift to search')
    ap.add_argument('--yaw-range', type=float, default=10.0, help='[deg] +- yaw to search')
    ap.add_argument('--pieces', type=int, default=600)
    a = ap.parse_args()

    pts = read_pcd(a.map)
    p = pts[:, :3].astype(np.float64)
    n = len(p)
    s0, s1 = (float(v) / 100 for v in a.start.split(':'))
    e0, e1 = (float(v) / 100 for v in a.end.split(':'))
    start = ip.voxel_downsample(p[int(s0 * n):int(s1 * n)], 0.2)
    end = ip.voxel_downsample(p[int(e0 * n):int(e1 * n)], 0.2)
    src = end[np.random.default_rng(0).choice(len(end), min(4000, len(end)), replace=False)]
    nn = ip.VoxelNN(start, 0.2)
    c = np.median(src, 0)
    print(f'start part {a.start}% center {np.round(np.median(start, 0), 1)}, '
          f'end part {a.end}% center {np.round(c, 1)}')

    # 1. coarse grid search (parallel), 2. ICP refinement
    z0, z1 = (float(v) for v in a.z_range.split(':'))
    xs = np.arange(-a.xy_range, a.xy_range + 0.1, 1.0)
    grid = [(yaw, dz) for yaw in np.radians(np.arange(-a.yaw_range, a.yaw_range + 0.1, 2.0))
            for dz in np.arange(z0, z1 + 0.1, 0.5)]
    _JOB = (nn, src, c, xs, xs)
    t = time.time()
    with multiprocessing.get_context('fork').Pool(min(os.cpu_count() or 1, 16)) as pool:
        cands = [x for part in pool.map(_score_block, grid) for x in part]
    cands.sort(key=lambda x: -x[0])
    before = (nn.query(src)[0] < 0.3).mean()
    print(f'grid search {time.time() - t:.0f} s. end-on-start overlap: no shift {before:.3f}')
    for sc, yaw, dx, dy, dz in cands[:3]:
        print(f'  {sc:.3f}  yaw {math.degrees(yaw):5.1f} deg  dx {dx:4.1f}  dy {dy:4.1f}  dz {dz:5.1f} m')
    _, yaw, dx, dy, dz = cands[0]
    r0 = ip.rot_z(yaw)
    r, tr, inl = ip.icp(nn, src, r0, np.array([dx, dy, dz]) + c - r0 @ c)
    print(f'ICP: overlap {inl:.3f}, shift {np.round(tr, 2)} m, '
          f'yaw {math.degrees(math.atan2(r[1, 0], r[0, 0])):.1f} deg')
    if inl < 2 * before or inl < 0.1:
        print('WARNING: the end does not clearly match the start (did the walk return to it?)')

    # 3. spread the correction in proportion to time
    ang = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    axis = (np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(ang))
            if ang > 1e-9 else np.array([0.0, 0.0, 1.0]))
    f0, f1 = (s0 + s1) / 2, (e0 + e1) / 2
    out = np.empty_like(p)
    for k in range(a.pieces):
        i0, i1 = k * n // a.pieces, (k + 1) * n // a.pieces
        seg = p[i0:i1]
        m = np.median(seg, 0)
        al = max(0.0, ((i0 + i1) / 2 / n - f0) / (f1 - f0))
        out[i0:i1] = (seg - m) @ rot_axis(axis, al * ang).T + m + al * (r @ m + tr - m)

    v0, v1 = occupied_voxels(p), occupied_voxels(out)
    print(f'occupied 0.1 m voxels: before {v0:,}, after {v1:,} ({(v1 - v0) / v0 * 100:+.1f}%)')
    if v1 > v0 * 1.02:
        print('RESULT: the corrected map is LESS consistent (things doubled). Do not use it; '
              'the drift probably did not grow in proportion to time.')
    else:
        print('RESULT: consistency did not get worse. Check it in CloudCompare before using it.')
    pts[:, :3] = out
    write_pcd(a.out, pts)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
