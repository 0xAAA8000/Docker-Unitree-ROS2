"""ボクセル地図 (robot_map.voxels) の試験 (pytest)."""
import numpy as np

from robot_map.grid import read_pcd_xyz
from robot_map.voxels import pack, unpack, VoxelMap


def test_pack_roundtrip():
    idx = np.random.default_rng(0).integers(-100000, 100000, (1000, 3))
    assert (unpack(pack(idx)) == idx).all()


def test_counts_match_naive():
    rng = np.random.default_rng(1)
    v, ref = VoxelMap(0.1), {}
    for it in range(30):
        pts = rng.uniform(-3, 3, (int(rng.integers(1, 500)), 3))
        h = int(rng.integers(1, 3))
        v.add(pts, hits=h)
        for k in np.unique(pack(np.floor(pts / 0.1))):
            ref[k] = ref.get(k, 0) + h
        if it % 4 == 0:
            v.merge()
    v.merge()
    assert list(v.keys) == sorted(ref)
    assert all(v.hits[i] == ref[k] for i, k in enumerate(v.keys))


def test_save_drops_rare_voxels(tmp_path):
    rng = np.random.default_rng(2)
    v = VoxelMap(0.1)
    wall = np.column_stack([rng.uniform(0, 5, 2000), np.full(2000, 2.05), rng.uniform(0, 2, 2000)])
    for _ in range(3):
        v.add(wall)
    v.add(np.array([[1.05, 1.05, 0.55]]))       # 1 回だけ見た点 (通った人)
    path = tmp_path / 'map.pcd'
    n = v.save_pcd(str(path), min_hits=2)
    xyz = read_pcd_xyz(str(path))
    assert len(xyz) == n > 0
    assert not np.any(np.all(np.abs(xyz - [1.05, 1.05, 0.55]) < 1e-6, axis=1))
