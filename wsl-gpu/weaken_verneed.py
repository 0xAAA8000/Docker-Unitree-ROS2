#!/usr/bin/env python3
"""ELF の .gnu.version_r にある指定バージョン要求を weak にし、見つからなくてもロードできるようにする。

使い方: weaken_verneed.py <ELF> <バージョン名>...
"""
import struct
import sys

VER_FLG_WEAK = 0x2
SHT_GNU_VERNEED = 0x6FFFFFFE

path, versions = sys.argv[1], set(sys.argv[2:])
with open(path, 'r+b') as f:
    data = bytearray(f.read())
    assert data[:4] == b'\x7fELF' and data[4] == 2 and data[5] == 1, '64-bit LE ELF only'
    e_shoff, = struct.unpack_from('<Q', data, 0x28)
    e_shentsize, e_shnum = struct.unpack_from('<HH', data, 0x3A)
    shdrs = [struct.unpack_from('<IIQQQQIIQQ', data, e_shoff + i * e_shentsize) for i in range(e_shnum)]
    for sh in shdrs:
        if sh[1] != SHT_GNU_VERNEED:
            continue
        strtab = shdrs[sh[6]][4]
        off, count = sh[4], sh[7]
        for _ in range(count):
            _, vn_cnt, _, vn_aux, vn_next = struct.unpack_from('<HHIII', data, off)
            aux = off + vn_aux
            for _ in range(vn_cnt):
                _, flags, _, name, nxt = struct.unpack_from('<IHHII', data, aux)
                ver = data[strtab + name:data.index(0, strtab + name)].decode()
                if ver in versions:
                    struct.pack_into('<H', data, aux + 4, flags | VER_FLG_WEAK)
                aux += nxt
            off += vn_next
    f.seek(0)
    f.write(data)
