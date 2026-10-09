"""Read / write PCD files as numpy arrays (x y z intensity, float32).

Reads binary and ascii PCDs with any extra fields (Point-LIO's normals, CloudCompare's
'_' padding fields). binary_compressed is not supported.

  python3 pcd_io.py IN.pcd OUT.pcd     # rewrite as a plain x y z intensity PCD
"""
import sys

import numpy as np


def read_pcd(path):
    """Returns an (N, 4) float32 array: x y z intensity (intensity 0 if missing). Order is kept."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            key, _, value = f.readline().decode(errors='ignore').strip().partition(' ')
            header[key] = value.split()
            if key == 'DATA':
                break
        data = f.read()
    fields = header['FIELDS']
    sizes = [int(s) * int(c) for s, c in zip(header['SIZE'], header.get('COUNT', ['1'] * len(fields)))]
    n = int(header['POINTS'][0])
    out = np.zeros((n, 4), np.float32)
    if header['DATA'][0] == 'binary':
        raw = np.frombuffer(data[:n * sum(sizes)], dtype=np.uint8).reshape(n, sum(sizes))
        offset = 0
        for name, size, typ in zip(fields, sizes, header['TYPE']):
            if name in ('x', 'y', 'z', 'intensity'):
                if typ != 'F' or size != 4:
                    raise ValueError(f'{name} must be float32')
                out[:, 'xyz'.find(name) if name != 'intensity' else 3] = \
                    raw[:, offset:offset + 4].copy().view(np.float32)[:, 0]
            offset += size
    elif header['DATA'][0] == 'ascii':
        a = np.loadtxt(data.decode().splitlines(), dtype=np.float32, ndmin=2)
        for name in ('x', 'y', 'z', 'intensity'):
            if name in fields:
                out[:, 'xyz'.find(name) if name != 'intensity' else 3] = a[:, fields.index(name)]
    else:
        raise ValueError('binary_compressed PCD is not supported')
    return out


def write_pcd(path, points):
    """points: (N, 3) or (N, 4) array. Writes binary x y z intensity."""
    p = np.zeros((len(points), 4), np.float32)
    p[:, :points.shape[1]] = points[:, :4]
    header = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              f'COUNT 1 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(p)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(p.tobytes())


if __name__ == '__main__':
    write_pcd(sys.argv[2], read_pcd(sys.argv[1]))
