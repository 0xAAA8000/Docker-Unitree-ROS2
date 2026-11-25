"""3D 点群をボクセル (立方体のマス) にまとめて貯める. 地図の保存と 3D 表示に使う.

同じボクセルに何度点が入っても 1 つにまとめるので, 同じ場所を何度見てもメモリは増えない.
ボクセルごとに「点が入ったスキャンの数」(hits) を数え, 一瞬だけの点 (通った人など) を
保存や表示から外すのに使う.

ボクセルは整数の番号 (ix, iy, iz) = floor(座標 / size) で表し, 1 つの int64 に詰めて
numpy の配列で持つ (Python の set より 1 個あたりのメモリがずっと少ない. 10 万個で約 1 MB).
"""
import os
import tempfile

import numpy as np

_BITS = 21                      # 1 軸あたりのビット数. 0.1 m のボクセルで ±約 100 km
_OFF = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1
_MAX_HITS = np.iinfo(np.uint16).max


def pack(idx):
    """(N, 3) の整数の番号 -> (N,) int64."""
    i = idx.astype(np.int64) + _OFF
    return (i[:, 0] << (2 * _BITS)) | (i[:, 1] << _BITS) | i[:, 2]


def unpack(keys):
    """(N,) int64 -> (N, 3) int64 の番号."""
    return np.stack([(keys >> (2 * _BITS)) & _MASK, (keys >> _BITS) & _MASK, keys & _MASK],
                    axis=1) - _OFF


class VoxelMap:

    def __init__(self, size):
        self.size = float(size)
        self.keys = np.empty(0, np.int64)      # 並べ替え済み・重複なし
        self.hits = np.empty(0, np.uint16)
        self._pending = []                     # まだ merge していないスキャンごとの番号
        self.version = 0

    def __len__(self):
        return len(self.keys)

    def add(self, xyz, hits=1):
        """1 スキャン分の点 (N, 3) を足す. 同じスキャンの中の重複は 1 回と数える."""
        if not len(xyz):
            return
        keys = np.unique(pack(np.floor(np.asarray(xyz, np.float64) / self.size)))
        self._pending.append((keys, hits))

    def merge(self):
        """足した分を本体にまとめる (毎スキャンではなく, 1 秒ごとなどにまとめて呼ぶ)."""
        if not self._pending:
            return False
        # 足した分だけをまとめる (本体の大きさに比例する並べ直しはしない)
        keys = np.concatenate([k for k, _ in self._pending])
        hits = np.concatenate([np.full(len(k), h, np.int64) for k, h in self._pending])
        self._pending = []
        keys, inv = np.unique(keys, return_inverse=True)
        hits = np.bincount(inv, weights=hits, minlength=len(keys)).astype(np.int64)
        # 本体にあるものは数を足し, 無いものは並び順を保って差し込む
        pos = np.searchsorted(self.keys, keys)
        found = pos < len(self.keys)
        found[found] = self.keys[pos[found]] == keys[found]
        if found.any():
            i = pos[found]
            self.hits[i] = np.minimum(self.hits[i].astype(np.int64) + hits[found], _MAX_HITS)
        new = ~found
        if new.any():
            self.keys = np.insert(self.keys, pos[new], keys[new])
            self.hits = np.insert(self.hits, pos[new], np.minimum(hits[new], _MAX_HITS))
        self.version += 1
        return True

    def centers(self, min_hits=1):
        """hits が min_hits 以上のボクセルの中心 (N, 3) float64."""
        keys = self.keys[self.hits >= min_hits]
        return (unpack(keys) + 0.5) * self.size

    def save_pcd(self, path, min_hits=1):
        """ボクセルの中心を PCD (binary, x y z float32) に書く. 書き終えてから置き換えるので,
        途中で止まっても前のファイルは壊れない. 書いた点の数を返す."""
        self.merge()
        xyz = self.centers(min_hits).astype(np.float32)
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        header = (
            '# .PCD v0.7 - robot_map のボクセル地図 (ボクセルの中心)\n'
            'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n')
        fd, tmp = tempfile.mkstemp(prefix='.voxels_', suffix='.pcd', dir=d)
        try:
            with os.fdopen(fd, 'wb') as f:
                f.write(header.encode())
                f.write(np.ascontiguousarray(xyz).tobytes())
            os.chmod(tmp, 0o644)        # mkstemp は本人だけ読める権限で作るので, 普通のファイルと同じにする
            os.replace(tmp, path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise
        return len(xyz)
