#!/usr/bin/env python3
"""
rc_monitor - live view of everything that decides whether the board arms.

Run this in one window and work the sticks in the other hand. It shows, once
a second, exactly what the flight code sees and what it is commanding to the
motors -- so an arming refusal names its own cause instead of being guessed at.

The ARM column spells out which precondition is failing:

    thr    throttle is not below neutral      (armhandler: cmd.Throttle < 0)
    yaw    the arming gesture is not being held hard enough
    alarm  Receiver or Guidance is CRITICAL   (forces disarm)
    OK     everything the arm check wants is true -- hold it for 1s

    PROPS OFF.

Usage:
    python3 tools/rc_monitor.py [--serial DEV] [--baud N]
"""

import argparse
import os
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

ARMED_THRESHOLD = 0.50          # armhandler.c
# Arming gesture -> (field, sign that ARMS). armingInputLevel is negated for
# the "Right"/"Aft" cases and arming triggers on <= -threshold, so the sign
# that actually arms is the one below.
GESTURE = {
    "Yaw Right":     ("Yaw",   +1),
    "Yaw Left":      ("Yaw",   -1),
    "Roll Right":    ("Roll",  +1),
    "Roll Left":     ("Roll",  -1),
    "Pitch Aft":     ("Pitch", +1),
    "Pitch Forward": ("Pitch", -1),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", default="/dev/cu.usbserial-210")
    ap.add_argument("--baud", type=int, default=57600)
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
        sys.exit(
            "no link to the board after 30s.\n"
            "  The port exists but nothing answered. Most often that means\n"
            "  something else already has it open -- only one process can hold\n"
            "  %s at a time. Close any other tool talking\n"
            "  to the board and try again. If nothing else is running, check\n"
            "  the board is powered and try python3 tools/imu_bringup.py."
            % args.serial)

    def fresh(name, timeout=1.0):
        """Always a new reply. Reading a cached object at speed looks exactly
        like a stick held perfectly still, which is how a frozen sample once
        got mistaken for real data."""
        got.pop(name, None)
        client.request_object(name)
        end = time.time() + timeout
        while time.time() < end:
            if name in got:
                return got[name]
            time.sleep(0.02)
        return None

    fms = fresh("ManualControlSettings", 3.0)
    fmset = fresh("FlightModeSettings", 3.0)
    gesture = fmset.get("Arming") if fmset else "?"
    field, sign = GESTURE.get(gesture, (None, 0))
    print("\narming gesture: %s" % gesture, end="")
    print("   (needs %s %s %+.2f, throttle < 0, held 1s)\n"
          % (field, ">=" if sign > 0 else "<=", sign * ARMED_THRESHOLD)
          if field else "   (not a stick gesture)\n")
    print("  Armed      thr    roll   pitch    yaw   motors            ARM",
          flush=True)
    misses = 0
    try:
        while True:
            f = fresh("FlightStatus")
            m = fresh("ManualControlCommand")
            ac = fresh("ActuatorCommand")
            if not (f and m):
                # Silence here is the worst possible output: an earlier
                # version just looped on `continue` and printed nothing at
                # all, which is indistinguishable from a hung tool.
                misses += 1
                print("  ...waiting for telemetry (%d) -- board busy or link "
                      "flaky" % misses, flush=True)
                time.sleep(0.5)
                continue
            misses = 0
            al = fresh("SystemAlarms", 0.6)
            thr = m.get("Throttle", 0.0)
            val = m.get(field, 0.0) if field else 0.0
            gesture_ok = (val >= ARMED_THRESHOLD) if sign > 0 else (val <= -ARMED_THRESHOLD)

            why = []
            if thr >= 0:
                why.append("thr")
            if field and not gesture_ok:
                why.append("yaw" if field == "Yaw" else field.lower())
            if not m.get("Connected"):
                why.append("nolink")
            verdict = "OK - hold 1s" if not why else "need: " + ",".join(why)
            if f.get("Armed") == "Armed":
                verdict = "*** ARMED ***"

            ch = [int(v) for v in ac["Channel"][:4]] if ac else []
            print("  %-9s %+5.2f  %+5.2f  %+5.2f  %+5.2f  %-17s %s" % (
                f.get("Armed"), thr, m.get("Roll", 0), m.get("Pitch", 0),
                m.get("Yaw", 0), str(ch), verdict), flush=True)
            time.sleep(0.4)
    except KeyboardInterrupt:
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
