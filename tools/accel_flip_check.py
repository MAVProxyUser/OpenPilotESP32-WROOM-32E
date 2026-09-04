#!/usr/bin/env python3
"""
Decide whether an accelerometer's error is BIAS or SCALE, over UAVTalk serial.

A board sitting still in ONE orientation cannot tell you this. If |a| reads
10.93 where gravity is 9.807, that is equally consistent with

    a zero-g offset of -1.09 m/s^2      (ordinary, and what calibration is for)
    a gain error of +11.4%              (a broken or counterfeit part)

and both are perfectly steady, so "the reading is stable" proves nothing. The
two only separate when you flip the board, because a bias keeps its sign while
gravity changes its own:

    upright   z = -g*s + b
    inverted  z = +g*s + b
      =>  b = (z_up + z_dn) / 2        s = (z_dn - z_up) / (2g)

Usage:  accel_flip_check.py [port] [seconds]

Sit the board level, start the script, and turn it upside down partway through
the window. Orientations are detected automatically -- there is nothing to
press. Read-only: this sends one object request and never arms anything.
"""
import glob
import math
import os
import re
import struct
import sys
import time

import serial

GRAV = 9.80665
SYNTH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "NinjaPilot-15.02.ninja",
                     "build", "uavobject-synthetics", "flight")


def object_ids():
    ids = {}
    for header in glob.glob(os.path.join(SYNTH, "*.h")):
        with open(header) as fh:
            for m in re.finditer(r"#define\s+([A-Z0-9_]+)_OBJID\s+(0x[0-9A-Fa-f]+)",
                                 fh.read()):
                ids[int(m.group(2), 16)] = m.group(1)
    return ids


def crc8(data, crc=0):
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frames(buf, ids):
    i = 0
    while i < len(buf) - 10:
        if buf[i] != 0x3C:
            i += 1
            continue
        length = struct.unpack_from("<H", buf, i + 2)[0]
        if length < 10 or length > 800 or i + length + 1 > len(buf):
            i += 1
            continue
        if crc8(buf[i:i + length]) != buf[i + length]:
            i += 1
            continue
        oid = struct.unpack_from("<I", buf, i + 4)[0]
        yield ids.get(oid, ""), bytes(buf[i + 10:i + length])
        i += length + 1


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.wchusbserial8320"
    window = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
    ids = object_ids()

    print("Sampling %s for %.0fs. Start LEVEL, then turn the board UPSIDE DOWN "
          "partway through." % (port, window))

    link = serial.Serial(port, 57600, timeout=0.3)
    started = time.time()
    buf = bytearray()
    while time.time() - started < window:
        buf += link.read(4096)
    link.close()

    up, down = [], []
    for name, data in frames(buf, ids):
        if name != "ACCELSTATE" or len(data) < 12:
            continue
        x, y, z = struct.unpack_from("<3f", data, 0)
        (up if z < -8.0 else down if z > 8.0 else []).append((x, y, z))

    print("\n%d samples level, %d samples inverted" % (len(up), len(down)))
    if not up or not down:
        print("\nNeed BOTH orientations. Re-run and flip the board during the window.")
        return 1

    z_up = sum(s[2] for s in up) / len(up)
    z_dn = sum(s[2] for s in down) / len(down)
    bias = (z_up + z_dn) / 2.0
    scale = (z_dn - z_up) / (2.0 * GRAV)

    print("  mean z level    %+8.3f m/s^2" % z_up)
    print("  mean z inverted %+8.3f m/s^2" % z_dn)
    print("\n  Z zero-g offset  %+.3f m/s^2  (%+.0f mg)" % (bias, 1000.0 * bias / GRAV))
    print("  Z gain           %.4f  (%+.1f%%)" % (scale, 100.0 * (scale - 1.0)))

    off_mg = abs(1000.0 * bias / GRAV)
    gain_pct = abs(100.0 * (scale - 1.0))
    print()
    if gain_pct < 3.0 and off_mg > 40.0:
        print("  -> Plain zero-g OFFSET. Gain is within the +/-3%% datasheet")
        print("     tolerance, so the part is fine; a six-point accel calibration")
        print("     removes this and it is a normal pre-flight step.")
    elif gain_pct >= 5.0:
        print("  -> Real GAIN error, outside datasheet tolerance. Check that")
        print("     ACCEL_CONFIG really holds the range the driver scales for")
        print("     before blaming the part.")
    else:
        print("  -> Both terms small. Calibrate and fly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
