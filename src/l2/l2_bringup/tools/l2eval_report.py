"""Compare localization runs with the Point-LIO reference trajectory.

  python3 l2eval_report.py EVAL_DIR NAME [NAME ...]

Reads EVAL_DIR/lio.csv (reference) and EVAL_DIR/NAME.csv / NAME.log (localization run).
There is no ground truth, so Point-LIO (smooth, no jumps, slow drift) is rigidly fitted
onto the localization trajectory and the residual is reported:
  RMSE / p95 / max  residual after the fit (includes Point-LIO drift, so a floor exists)
  jumps             consecutive poses faster than 3 m/s (impossible for a cart / hand)
  rejected          scans whose fitness score was over score_threshold
"""
import os
import re
import sys

import numpy as np

SKIP_SEC = 10.0      # ignore the first seconds while NDT converges
JUMP_SPEED = 3.0     # m/s


def load(path):
    a = np.loadtxt(path, delimiter=',', usecols=range(1, 9), ndmin=2)
    return a[np.argsort(a[:, 0])]


def report(d, name, lio):
    loc = load(f'{d}/{name}.csv')
    loc = loc[loc[:, 0] > loc[0, 0] + SKIP_SEC]
    idx = np.clip(np.searchsorted(lio[:, 0], loc[:, 0]), 0, len(lio) - 1)
    ok = np.abs(lio[idx, 0] - loc[:, 0]) < 0.05
    p, q = loc[ok, 1:4], lio[idx[ok], 1:4]
    # rigid fit (Umeyama without scale): Point-LIO frame -> map frame
    mp, mq = p.mean(0), q.mean(0)
    u, _, vt = np.linalg.svd((q - mq).T @ (p - mp))
    r = (u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt).T
    err = np.linalg.norm(p - ((q - mq) @ r.T + mp), axis=1)
    speed = (np.linalg.norm(np.diff(loc[:, 1:4], axis=0), axis=1)
             / np.maximum(np.diff(loc[:, 0]), 1e-3))
    head = (f'| {name} | {np.sqrt((err ** 2).mean()) * 100:.1f} | {np.percentile(err, 95) * 100:.1f} '
            f'| {err.max() * 100:.1f} | {int((speed > JUMP_SPEED).sum())} ')
    if not os.path.exists(f'{d}/{name}.log'):           # e.g. live.csv dumped from the bag
        return head + '| - | - | - |'
    log = open(f'{d}/{name}.log', errors='ignore').read()
    align = np.array([float(x) for x in re.findall(r'align time:([0-9.e-]+)', log)])
    fitness = np.array([float(x) for x in re.findall(r'fitness score: ([0-9.e-]+)', log)])
    rejected = log.count('fitness score is over')
    return head + (f'| {rejected / max(len(fitness), 1) * 100:.1f} | {np.median(fitness):.3f} '
                   f'| {align.mean() * 1000:.1f} |')


def main():
    d, names = sys.argv[1], sys.argv[2:]
    lio = load(f'{d}/lio.csv')
    print('| run | RMSE [cm] | p95 [cm] | max [cm] | jumps | rejected [%] | fitness (median) | align [ms] |')
    print('| --- | --- | --- | --- | --- | --- | --- | --- |')
    for name in names:
        print(report(d, name, lio))


if __name__ == '__main__':
    main()
