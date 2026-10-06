#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Read a LINKTYPE_USB_LINUX_MMAPPED pcap (usbcap's output) into records."""
import struct, sys
from collections import namedtuple

Rec = namedtuple("Rec", "t id type xfer ep dev status len_urb len_cap data")

def read(path):
    f = open(path, "rb").read()
    magic, _, _, _, _, lt = struct.unpack_from("<IIIIII", f, 0)
    assert magic == 0xa1b2c3d4 and lt == 220, "not a usbmon pcap"
    o = 24
    while o + 16 <= len(f):
        ts, tu, incl, orig = struct.unpack_from("<IIII", f, o); o += 16
        h = f[o:o + 64]; d = f[o + 64:o + incl]; o += incl
        uid, typ, xt, ep, dev, bus, fs, fd, sec, usec, st, lu, lc = struct.unpack_from("<QBBBBHbbqiiII", h)
        yield Rec(ts + tu / 1e6, uid, chr(typ), xt, ep, dev, st, lu, lc, d)
