#!/usr/bin/env python3
"""
esc_test - drive each ESC directly over UAVObjects, one at a time.

Uses the flight code's own servo-configuration mechanism (the same one the
GCS output tab uses): ActuatorCommand is switched to flight-read-only, at
which point actuator.c's own writes are refused and it passes OUR channel
values straight to the pins -- no mixer, no stabilization, no arming.
That is the point: the last session proved stabilization reshapes motor
commands on the bench (integral windup), so a fair per-ESC test must bypass
the flight controller entirely.

Each channel in turn: hold min so the ESC arms, ramp to peak, hold, ramp
back down. Then the next. Watch each motor; they should all behave
identically. One that stays silent while its position is announced -- with
the identical command its neighbours spun on -- is a hardware fault.

         !!  PROPS OFF. THIS SPINS MOTORS TO FULL THROTTLE.  !!

ActuatorCommand access and channel values are restored on ANY exit,
including Ctrl+C.

Usage:
    python3 tools/esc_test.py [--serial DEV] [--peak US] [--motor N]
"""

import argparse
import os
import struct
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
NINJAPILOT_ROOT = os.environ.get(
    "NINJAPILOT_ROOT",
    os.path.abspath(os.path.join(_HERE, "..", "..", "NinjaPilot-15.02.ninja")))
sys.path.insert(0, os.path.join(NINJAPILOT_ROOT, "ground", "pyuavtalk"))
sys.path.insert(0, _HERE)

import uavtalk                                                # noqa: E402
from uavtalk_client import UAVTalkClient, default_xml_dir     # noqa: E402
from imu_bringup import Esp32SerialTransport                  # noqa: E402

POS = {0: "M1 front-left  (pin 15)", 1: "M2 front-right (pin 33)",
       2: "M3 rear-right  (pin 27)", 3: "M4 rear-left   (pin 12)"}
MIN_US = 1000


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", default="/dev/cu.usbserial-210")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--peak", type=int, default=2000,
                    help="top of the ramp in us (default 2000 = full)")
    ap.add_argument("--motor", type=int, choices=[1, 2, 3, 4],
                    help="test only this motor (1-4); default all four")
    args = ap.parse_args()

    db = uavtalk.UAVObjectDB(default_xml_dir())
    lock = threading.Lock()

    class Locked(object):
        def __init__(self, inner):
            self.inner = inner

        def send(self, data):
            with lock:
                self.inner.send(data)

        def poll_recv(self, timeout):
            return self.inner.poll_recv(timeout)

    print("opening %s (board keeps running -- attach does not reset it)..."
          % args.serial, flush=True)
    client = UAVTalkClient(Locked(Esp32SerialTransport(args.serial, args.baud)), db)
    got = {}
    connected = threading.Event()

    def on_object(objdef, inst_id, decoded):
        got[objdef.name] = decoded

    threading.Thread(target=client.run,
                     kwargs=dict(duration=100000, on_object=on_object,
                                 on_connected=connected.set),
                     daemon=True).start()
    if not connected.wait(30):
        sys.exit("no link -- is something else holding the port?")

    def fresh(name, timeout=2.0):
        got.pop(name, None)
        client.request_object(name)
        end = time.time() + timeout
        while time.time() < end:
            if name in got:
                return got[name]
            time.sleep(0.02)
        return None

    # ---- safety gates ----------------------------------------------------
    f = fresh("FlightStatus")
    if not f or f.get("Armed") != "Disarmed":
        sys.exit("Board is %s -- disarm first. This test runs DISARMED by "
                 "design; the passthrough bypasses arming entirely."
                 % (f.get("Armed") if f else "unreachable"))

    print("""
  !! This will spin each motor up to %dus -- FULL THROTTLE by default !!
  !! PROPS OFF. Secure the airframe. Battery connected to the ESCs.  !!
""" % args.peak)
    if input("  Type SPIN to continue: ").strip() != "SPIN":
        print("aborted, nothing sent.")
        return 1

    ac_meta_id = db["ActuatorCommand"].obj_id + 1

    def get_meta():
        client.meta_payloads.pop(ac_meta_id, None)
        client.send_raw(uavtalk.TYPE_OBJ_REQ, ac_meta_id)
        end = time.time() + 3.0
        while time.time() < end and ac_meta_id not in client.meta_payloads:
            time.sleep(0.02)
        return client.meta_payloads.get(ac_meta_id)

    ac = fresh("ActuatorCommand")
    if not ac:
        sys.exit("cannot read ActuatorCommand")
    orig_meta = get_meta()
    if not orig_meta or len(orig_meta) < 8:
        sys.exit("cannot read ActuatorCommand metadata")

    def send_channels(us_list):
        ac2 = dict(ac)
        ch = list(ac2["Channel"])
        for i in range(4):
            ch[i] = us_list[i]
        ac2["Channel"] = ch
        client.send_object("ActuatorCommand", ac2)

    def set_flight_access(readonly):
        flags = struct.unpack_from("<H", orig_meta, 0)[0]
        flags = (flags | 1) if readonly else (flags & ~1)   # bit0 = flight access
        new = bytearray(orig_meta)
        struct.pack_into("<H", new, 0, flags)
        client.send_raw(uavtalk.TYPE_OBJ, ac_meta_id, 0, bytes(new))
        time.sleep(0.2)

    try:
        # Take the output over, then PROVE the passthrough is live before
        # anything can spin: command a distinctive sub-idle value and read it
        # back. If the mixer were still in charge it would overwrite us.
        set_flight_access(True)
        probe = [1000, 1000, 1000, 1000]
        probe[0] = 1011
        send_channels(probe)
        time.sleep(0.4)
        back = fresh("ActuatorCommand")
        if not back or int(back["Channel"][0]) != 1011:
            raise RuntimeError(
                "passthrough not engaged (read back %s) -- aborting before "
                "anything spins" % (back["Channel"][:4] if back else None))
        print("  passthrough engaged (flight code released the outputs)\n")

        motors = [args.motor - 1] if args.motor else [0, 1, 2, 3]
        for m in motors:
            print("  === %s ===" % POS[m], flush=True)
            base = [MIN_US] * 4
            send_channels(base)
            print("      min %dus for 2s (ESC should arm-beep if it just "
                  "got signal)" % MIN_US, flush=True)
            end = time.time() + 2.0
            while time.time() < end:
                send_channels(base)
                time.sleep(0.1)

            print("      ramp up to %dus..." % args.peak, flush=True)
            for us in range(MIN_US, args.peak + 1, 20):
                base[m] = us
                send_channels(base)
                time.sleep(0.07)
            print("      hold peak 1s -- IS IT SPINNING?", flush=True)
            end = time.time() + 1.0
            while time.time() < end:
                send_channels(base)
                time.sleep(0.1)
            print("      ramp down...", flush=True)
            for us in range(args.peak, MIN_US - 1, -20):
                base[m] = us
                send_channels(base)
                time.sleep(0.03)
            send_channels([MIN_US] * 4)
            time.sleep(0.5)
        print("\n  done. Every motor got an identical profile.")
    finally:
        # Whatever happened above: outputs to minimum, then hand the object
        # back to the flight code. Three sends, belt and braces -- a motor
        # left spinning by a dead test tool is not a bug report, it is an
        # injury.
        for _ in range(3):
            send_channels([MIN_US] * 4)
            time.sleep(0.1)
        set_flight_access(False)
        back = get_meta()
        restored = back and (struct.unpack_from("<H", back, 0)[0] & 1) == 0
        print("  outputs released back to the flight code: %s"
              % ("verified" if restored else "UNVERIFIED -- power-cycle the "
                 "board before flying"), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
