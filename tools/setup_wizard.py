#!/usr/bin/env python3
"""
setup_wizard - the vehicle setup flow this board never got.

Three steps, each optional, each saved to flash immediately:

  1. LEVEL      measure the sensor's mounting tilt and cancel it with
                AttitudeSettings.BoardRotation. This is the standing -2.8
                pitch error that wound up the integrators on the bench and
                held two motors at minimum until the craft was tilted.
  2. ESC RANGE  classic all-at-once endpoint calibration: every ESC learns
                the same 1000-2000 range, so they all start at the same
                commanded pulse instead of each guessing its own.
  3. IDLE       per-motor spin-up points: each output ramps slowly, you
                press Enter the instant that motor starts, and its
                ChannelNeutral is set just above that -- so all four leave
                the ground together.

Steps 2 and 3 use the flight code's servo-configuration passthrough
(ActuatorCommand flight-read-only), run DISARMED, and restore everything on
any exit.

    PROPS OFF THROUGHOUT.
    Bench power: pull the BEC's red 5V lead off VUSB, power the board from
    USB, battery to the ESCs only. Never BEC-5V and USB together.

Usage:
    python3 tools/setup_wizard.py [--serial DEV] [--baud N]
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
MAX_US = 2000
LEVEL_OK_DEG = 0.5


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
    print("[link] up\n")

    def fresh(name, timeout=2.0):
        got.pop(name, None)
        client.request_object(name)
        end = time.time() + timeout
        while time.time() < end:
            if name in got:
                return got[name]
            time.sleep(0.02)
        return None

    def save(name):
        client.send_object("ObjectPersistence", {
            "Operation": "Save", "Selection": "SingleObject",
            "ObjectID": db[name].obj_id, "InstanceID": 0})
        end = time.time() + 5.0
        while time.time() < end:
            op = fresh("ObjectPersistence", 1.0)
            if op and op.get("Operation") == "Completed":
                return True
        return False

    def avg_attitude(secs):
        rs, ps = [], []
        end = time.time() + secs
        while time.time() < end:
            at = fresh("AttitudeState", 0.8)
            if at:
                rs.append(at.get("Roll", 0.0))
                ps.append(at.get("Pitch", 0.0))
            time.sleep(0.2)
        if not rs:
            return None, None
        return sum(rs) / len(rs), sum(ps) / len(ps)

    f = fresh("FlightStatus")
    if not f or f.get("Armed") != "Disarmed":
        sys.exit("board is %s -- disarm first" % (f.get("Armed") if f else "?"))

    # ================= STEP 1: LEVEL ====================================
    if input("STEP 1 - LEVEL CALIBRATION. Run it? [y/N] ").strip().lower() == "y":
        input("  Put the FRAME (not the sensor) dead level and still, then "
              "press Enter: ")
        r0, p0 = avg_attitude(3.0)
        print("  sensor reads roll %+.2f pitch %+.2f" % (r0, p0))
        if abs(r0) < LEVEL_OK_DEG and abs(p0) < LEVEL_OK_DEG:
            print("  already level within %.1f deg -- nothing to do.\n" % LEVEL_OK_DEG)
        else:
            ats = fresh("AttitudeSettings", 3.0)
            rot = list(ats["BoardRotation"])          # [Roll, Pitch, Yaw]
            base = rot[:]

            def apply(sign):
                ats2 = dict(ats)
                ats2["BoardRotation"] = [base[0] + sign * r0,
                                         base[1] + sign * p0, base[2]]
                client.send_object("AttitudeSettings", ats2)
                # The complementary filter reconverges at its own pace; the
                # correction is invisible for several seconds. Waiting less
                # than this reads the OLD attitude and any sign logic built
                # on it is garbage.
                print("  correction written (sign %+d) -- waiting 15s for "
                      "the filter to reconverge..." % sign)
                time.sleep(15.0)
                return avg_attitude(3.0)

            # Sign conventions for rotation corrections are exactly the kind
            # of thing that gets silently inverted between codebases, so
            # determine it empirically: try minus, and if the tilt GREW,
            # use plus instead.
            r1, p1 = apply(-1)
            print("  now reads roll %+.2f pitch %+.2f" % (r1, p1))
            if abs(r1) + abs(p1) > abs(r0) + abs(p0):
                print("  worse -- flipping sign.")
                r1, p1 = apply(+1)
                print("  now reads roll %+.2f pitch %+.2f" % (r1, p1))
            ok = abs(r1) < LEVEL_OK_DEG and abs(p1) < LEVEL_OK_DEG
            print("  level calibration %s; saving to flash..."
                  % ("GOOD" if ok else "IMPROVED (re-run to refine)"))
            print("  saved: %s\n" % save("AttitudeSettings"))

    # ============== passthrough plumbing for steps 2 and 3 ==============
    ac_meta_id = db["ActuatorCommand"].obj_id + 1

    def get_meta():
        client.meta_payloads.pop(ac_meta_id, None)
        client.send_raw(uavtalk.TYPE_OBJ_REQ, ac_meta_id)
        end = time.time() + 3.0
        while time.time() < end and ac_meta_id not in client.meta_payloads:
            time.sleep(0.02)
        return client.meta_payloads.get(ac_meta_id)

    ac = fresh("ActuatorCommand")
    orig_meta = get_meta()

    def send_channels(us):
        ac2 = dict(ac)
        ch = list(ac2["Channel"])
        for i in range(4):
            ch[i] = us[i]
        ac2["Channel"] = ch
        client.send_object("ActuatorCommand", ac2)

    def set_access(readonly):
        flags = struct.unpack_from("<H", orig_meta, 0)[0]
        flags = (flags | 1) if readonly else (flags & ~1)
        new = bytearray(orig_meta)
        struct.pack_into("<H", new, 0, flags)
        client.send_raw(uavtalk.TYPE_OBJ, ac_meta_id, 0, bytes(new))
        time.sleep(0.2)

    def hold(us, secs):
        end = time.time() + secs
        while time.time() < end:
            send_channels(us)
            time.sleep(0.1)

    took_over = False
    try:
        # ============= STEP 2: ESC ENDPOINT CALIBRATION =================
        if input("STEP 2 - ESC RANGE CALIBRATION (needs battery). Run it? "
                 "[y/N] ").strip().lower() == "y":
            print("""
    PROPS OFF. This is the classic all-at-once calibration:
      a. DISCONNECT the flight battery now.
      b. This tool outputs MAX (2000us) on all four channels.
      c. You connect the battery; every ESC sings its max-throttle tone.
      d. Tool drops to MIN (1000us); ESCs confirm and arm.
""")
            input("    Battery DISCONNECTED? Press Enter to output MAX: ")
            if not orig_meta or not ac:
                raise RuntimeError("no ActuatorCommand access")
            set_access(True)
            took_over = True
            probe = [MIN_US] * 4
            probe[0] = 1011
            send_channels(probe)
            time.sleep(0.4)
            back = fresh("ActuatorCommand")
            if not back or int(back["Channel"][0]) != 1011:
                raise RuntimeError("passthrough not engaged -- aborting")
            hold([MAX_US] * 4, 1.0)
            input("    MAX is live on all four. CONNECT the battery, wait "
                  "for the max-cal tones, then press Enter: ")
            print("    dropping to MIN...")
            hold([MIN_US] * 4, 3.0)
            input("    ESCs should have confirmed and armed. Press Enter: ")
            print("    endpoint calibration done.\n")

        # ============= STEP 3: PER-MOTOR IDLE POINTS ====================
        if input("STEP 3 - PER-MOTOR IDLE POINTS (spins motors, battery "
                 "connected, PROPS OFF). Run it? [y/N] ").strip().lower() == "y":
            if not took_over:
                set_access(True)
                took_over = True
                probe = [MIN_US] * 4
                probe[0] = 1011
                send_channels(probe)
                time.sleep(0.4)
                back = fresh("ActuatorCommand")
                if not back or int(back["Channel"][0]) != 1011:
                    raise RuntimeError("passthrough not engaged -- aborting")
            starts = {}
            for m in range(4):
                print("  === %s ===" % POS[m])
                print("      ramping slowly from %d. Press Enter the INSTANT "
                      "it starts spinning." % MIN_US)
                state = {"us": MIN_US, "run": True}

                def ramp():
                    us = [MIN_US] * 4
                    while state["run"] and state["us"] < 1500:
                        us[m] = state["us"]
                        send_channels(us)
                        time.sleep(0.25)
                        state["us"] += 5
                t = threading.Thread(target=ramp, daemon=True)
                t.start()
                input()
                state["run"] = False
                t.join()
                starts[m] = state["us"]
                send_channels([MIN_US] * 4)
                print("      starts at ~%dus\n" % starts[m])
                time.sleep(0.8)
            acts = fresh("ActuatorSettings", 3.0)
            nu = list(acts["ChannelNeutral"])
            for m, v in starts.items():
                nu[m] = min(v + 20, 1200)
            acts["ChannelNeutral"] = nu
            client.send_object("ActuatorSettings", acts)
            time.sleep(1.0)
            print("  ChannelNeutral set to %s" % [nu[i] for i in range(4)])
            print("  saved: %s\n" % save("ActuatorSettings"))
    finally:
        if took_over:
            for _ in range(3):
                send_channels([MIN_US] * 4)
                time.sleep(0.1)
            set_access(False)
            back = get_meta()
            ok = back and (struct.unpack_from("<H", back, 0)[0] & 1) == 0
            print("outputs released to flight code: %s"
                  % ("verified" if ok else "UNVERIFIED -- power-cycle before flying"))

    print("\nWizard done. Suggested check: tools/motor_watch.py, arm, raise")
    print("throttle -- with level calibrated, the four commands should now")
    print("stay close together instead of splitting front/rear.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
