"""スマホの 3D 表示に送るボクセルの版を管理し, 差分を作る (ROS に依存しない).

robot_map が 1 秒ごとに配信する「見えている範囲のボクセル」(/voxel_map) を受け取るたびに版を 1 つ進め,
直近 history 版を覚えておく. スマホは「この版から」と問い合わせ, 差分 (増えた・分類が変わったものと,
消えたもの) だけを受け取る. 覚えていない古い版からなら全体を返す.

返すバイナリ (リトルエンディアン)
  0  'VOX1'
  4  uint32  版
  8  float32 ボクセルの大きさ [m]
  12 uint8   1 なら全体 (それまでのボクセルを捨てて置き換える), 0 なら差分
  13 3 バイト 空き
  16 uint32  増えた数 A
  20 uint32  消えた数 R
  24 int16 x 3 x A  増えたボクセルの番号 (ix, iy, iz). 中心 = (番号 + 0.5) x 大きさ
     uint8 x A      増えたボクセルの分類 (0 床, 1 坂, 2 障害物, 255 未分類)
     (4 バイト境界まで 0 で埋める)
     int16 x 3 x R  消えたボクセルの番号
"""
import struct

import numpy as np

_BITS = 21
_OFF = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1


def _pack(idx):
    i = idx.astype(np.int64) + _OFF
    return (i[:, 0] << (2 * _BITS)) | (i[:, 1] << _BITS) | i[:, 2]


def _unpack(keys):
    return np.stack([(keys >> (2 * _BITS)) & _MASK, (keys >> _BITS) & _MASK, keys & _MASK],
                    axis=1) - _OFF


class VoxelStore:

    def __init__(self, size, history=10):
        self.size = float(size)
        self.history = history
        self.versions = []        # [(版, keys 並べ替え済み, labels)]
        self.version = 0

    def update(self, centers, labels):
        """見えている範囲のボクセル全体 (中心 (N, 3) と分類 (N,)) で新しい版を作る."""
        idx = np.floor(np.asarray(centers, np.float64) / self.size).astype(np.int64)
        keys = _pack(idx)
        order = np.argsort(keys, kind='stable')
        keys, labels = keys[order], np.asarray(labels, np.uint8)[order]
        self.version += 1
        self.versions.append((self.version, keys, labels))
        del self.versions[:-self.history]

    def count(self):
        return len(self.versions[-1][1]) if self.versions else 0

    def payload(self, since=0):
        """版 since から今の版までの差分 (覚えていなければ全体) のバイナリ."""
        if not self.versions:
            return _encode(0, self.size, True, np.empty(0, np.int64), np.empty(0, np.uint8),
                           np.empty(0, np.int64))
        version, keys, labels = self.versions[-1]
        base = next((v for v in self.versions if v[0] == since), None)
        if base is None:
            return _encode(version, self.size, True, keys, labels, np.empty(0, np.int64))
        _, bkeys, blabels = base
        # 増えたもの・分類が変わったもの
        pos = np.searchsorted(bkeys, keys)
        inb = pos < len(bkeys)
        inb[inb] = bkeys[pos[inb]] == keys[inb]
        changed = ~inb
        changed[inb] = blabels[pos[inb]] != labels[inb]
        # 消えたもの
        pos2 = np.searchsorted(keys, bkeys)
        inc = pos2 < len(keys)
        inc[inc] = keys[pos2[inc]] == bkeys[inc]
        return _encode(version, self.size, False, keys[changed], labels[changed], bkeys[~inc])


def _encode(version, size, full, add_keys, add_labels, remove_keys):
    def idx16(keys):
        idx = _unpack(keys) if len(keys) else np.empty((0, 3), np.int64)
        return np.clip(idx, -32768, 32767).astype('<i2').tobytes()
    head = struct.pack('<4sIfB3xII', b'VOX1', version, size, 1 if full else 0,
                       len(add_keys), len(remove_keys))
    body = idx16(add_keys) + np.asarray(add_labels, np.uint8).tobytes()
    body += b'\0' * (-len(body) % 4)
    return head + body + idx16(remove_keys)
