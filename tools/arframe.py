#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Decode AR8030 host-link frames out of a usbcap pcap.

Frame (both directions, one or more per bulk transfer):

  0      0xAA
  1..4   payload length, u32 little-endian
  5..8   reqid, u32 big-endian: domain (top 8 bits) | sub-command (24 bits)
  9..12  msgid, u32 big-endian
  13..16 sta,   i32 big-endian
  17     check: 0xFF XOR bytes 0..16
  18..   payload
  last   0xBB

Layout as seen in captures, and as arlink.ko and libarlink use it.

  arframe.py CAPTURE.pcap            summary by direction and reqid
  arframe.py CAPTURE.pcap --dump     every frame, one line each
"""
import struct, sys
from collections import Counter, namedtuple
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import pcapusb

Frame = namedtuple("Frame", "t dir domain sub msgid sta payload truncated")

def xor_check(b):
    x = 0xFF
    for c in b:
        x ^= c
    return x

def frames(path):
    """Yield frames from OUT submissions (ep 0x01) and IN completions (ep 0x81).

    usbcap may truncate long transfers; a frame whose payload was cut is still
    yielded, with truncated=True and the payload as far as it was captured."""
    for r in pcapusb.read(path):
        if r.type == "S" and r.ep == 0x01:
            d = "OUT"
        elif r.type == "C" and r.ep == 0x81 and r.len_urb > 0:
            d = "IN"
        else:
            continue
        b, o = r.data, 0
        while o + 18 <= len(b):
            if b[o] != 0xAA:
                o += 1
                continue
            ln, = struct.unpack_from("<I", b, o + 1)
            reqid, msgid, sta = struct.unpack_from(">IIi", b, o + 5)
            if xor_check(b[o:o + 17]) != b[o + 17]:
                o += 1
                continue
            end = o + 18 + ln
            cut = end + 1 > len(b)
            if not cut and b[end] != 0xBB:
                o += 1
                continue
            yield Frame(r.t, d, reqid >> 24, reqid & 0xFFFFFF, msgid, sta,
                        b[o + 18:min(end, len(b))], cut)
            o = end + 1

def main():
    path = sys.argv[1]
    fs = list(frames(path))
    t0 = fs[0].t if fs else 0
    if "--dump" in sys.argv:
        for f in fs:
            p = f.payload[:24].hex(" ")
            print(f"{f.t - t0:9.4f} {f.dir:3} dom={f.domain:#04x} sub={f.sub:#08x} "
                  f"msg={f.msgid:#010x} sta={f.sta:<6} len={len(f.payload)}{'+' if f.truncated else ''} {p}")
        return
    print(f"{len(fs)} frames over {fs[-1].t - t0:.1f} s" if fs else "no frames")
    c = Counter((f.dir, f.domain, f.sub) for f in fs)
    sizes = {}
    for f in fs:
        sizes.setdefault((f.dir, f.domain, f.sub), []).append(len(f.payload))
    for (d, dom, sub), n in sorted(c.items()):
        s = sizes[(d, dom, sub)]
        print(f"  {d:3} dom={dom:#04x} sub={sub:#08x}  x{n:<6} payload {min(s)}..{max(s)} bytes")

if __name__ == "__main__":
    main()
