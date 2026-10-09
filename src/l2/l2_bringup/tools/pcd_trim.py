"""Cut off the "comet tail" where Point-LIO diverged (points flying hundreds of m away).

Point-LIO saves points in the order it took them in, so position in the file ~ time.
The file is split into 0.1 % chunks. Chunk centers jump around when the view changes
(e.g. at corners), so they are smoothed over 5 chunks; divergence is where the smoothed
center moves more than --jump metres within 1 % of the file (faster than any walk) and
does not come back. Everything from a little before that is dropped.

  python3 pcd_trim.py MAP.pcd                 # report only
  python3 pcd_trim.py MAP.pcd -o OUT.pcd      # write the part before the divergence
  python3 pcd_trim.py MAP.pcd -o OUT.pcd --cut 96.5   # cut at a given percentage
"""
import argparse

import numpy as np

from pcd_io import read_pcd, write_pcd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('-o', '--out')
    ap.add_argument('--cut', type=float, help='cut at this percentage instead of detecting')
    ap.add_argument('--jump', type=float, default=25.0, help='[m] move within 1%% of the file')
    a = ap.parse_args()

    p = read_pcd(a.map)
    n = len(p)
    step = max(n // 1000, 1)
    centers = np.array([np.median(p[k * step:(k + 1) * step, :3], 0) for k in range(n // step)])
    smooth = np.array([np.median(centers[max(k - 2, 0):k + 3], 0) for k in range(len(centers))])
    W = 10                                            # 1 % of the file
    moved = np.zeros(len(smooth))
    stays = np.zeros(len(smooth), bool)
    for k in range(W, len(smooth)):
        moved[k] = np.linalg.norm(smooth[k] - smooth[k - W])
        later = smooth[k:k + W]
        stays[k] = (np.linalg.norm(later - smooth[k - W], axis=1) > a.jump).all()

    print(f'{n:,} points. chunk centers (position in file ~ time):')
    for k in range(0, len(centers), 50):
        print(f'  {k / 10:5.1f}%  {np.round(centers[k], 1)}')
    bad = np.where((moved > a.jump) & stays)[0]
    if a.cut is not None:
        cut = int(a.cut / 100 * n)
        print(f'cut at {a.cut:.1f}% (given)')
    elif len(bad):
        k = bad[0]                                   # run-away detected here
        print(f'divergence detected at {k / 10:.1f}% (moved {moved[k]:.0f} m within 1 %):')
        for j in range(max(k - W, 0), min(k + 3, len(centers)), 2):
            print(f'    {j / 10:5.1f}%  {np.round(centers[j], 1)}')
        cut = max(k - W - 2, 0) * step               # it started within the last 1 %; keep a margin
        print(f'cut at {cut / n * 100:.1f}%')
    else:
        print('no divergence found')
        cut = n
    kept = p[:cut]
    lo, hi = np.percentile(kept[:, :3], 0.5, 0), np.percentile(kept[:, :3], 99.5, 0)
    print(f'kept {len(kept):,} points, extent (0.5-99.5%) {np.round(lo, 1)} .. {np.round(hi, 1)}')
    if a.out:
        write_pcd(a.out, kept)
        print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
