"""3D 表示に送るボクセルの差分 (robot_goal.voxel_store) の試験 (pytest)."""
import struct

import numpy as np

from robot_goal.voxel_store import VoxelStore


def decode(b):
    mg, ver, size, full, na, nr = struct.unpack('<4sIfB3xII', b[:24])
    o = 24
    add = np.frombuffer(b, '<i2', 3 * na, o).reshape(-1, 3)
    o += 6 * na
    lab = np.frombuffer(b, 'u1', na, o)
    o += na + (-(o + na)) % 4
    rem = np.frombuffer(b, '<i2', 3 * nr, o).reshape(-1, 3)
    return mg, ver, full, add.tolist(), lab.tolist(), rem.tolist()


def test_full_then_diff_then_forgotten():
    s = VoxelStore(0.1, history=3)
    assert decode(s.payload(0))[:3] == (b'VOX1', 0, 1)
    s.update(np.array([[0.05, 0.05, 0.05], [0.15, 0.05, 0.05], [-0.35, 1.25, 0.45]]), [0, 0, 2])
    _, ver, full, add, lab, rem = decode(s.payload(0))
    assert (ver, full) == (1, 1) and len(add) == 3
    s.update(np.array([[0.05, 0.05, 0.05], [-0.35, 1.25, 0.45], [0.95, 0.05, 0.05]]), [1, 2, 0])
    _, ver, full, add, lab, rem = decode(s.payload(1))
    assert (ver, full) == (2, 0)
    assert sorted(zip(map(tuple, add), lab)) == [((0, 0, 0), 1), ((9, 0, 0), 0)]   # 変わった・増えた
    assert rem == [[1, 0, 0]]                                                         # 消えた
    for _ in range(3):
        s.update(np.array([[0.05, 0.05, 0.05]]), [0])
    assert decode(s.payload(1))[2] == 1          # 覚えていない古い版からは全体
