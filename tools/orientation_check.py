#!/usr/bin/env python3
"""
orientation_check - does the board's idea of "nose" match the airframe's?

Read-only. Prints, several times a second, what the flight side believes:

    board says: NOSE DOWN 31 deg   LEFT SIDE DOWN 4 deg     motors high: M1 M2

Hold the quad with the AIRFRAME's nose (the end you fly forward, where the
old CC3D arrow pointed) tipped down. The line must say NOSE DOWN. If it says
NOSE UP, the board's +x axis points at the tail: set
AttitudeSettings.BoardRotation Yaw = 180 (or fix the mount) BEFORE flying -
a 180-degree board on a correct mixer is positive feedback on both axes and
flips the instant the wheels get light. Same test for roll: dip the LEFT
arm (left as seen from behind, flying forward) and it must say LEFT SIDE
DOWN. The 2026-09-02 00:59 paced bench read every commanded tilt with the
opposite sign on both axes, which is either a 180-degree mount or the quad
held nose-toward-you at the keyboard; this settles which in ten seconds.

"motors high" only appears while armed (the tool never arms - use
bench_test for that); it names the pair above the mean so you can see the
mixer raise the low side.

    python3 orientation_check.py [--host 192.168.0.45] [--seconds 60]
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
import uavtalk  # noqa: E402
from uavtalk_client import UdpTransport, UAVTalkClient, default_xml_dir  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.45")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    db = uavtalk.UAVObjectDB(default_xml_dir())
    client = UAVTalkClient(UdpTransport(args.host, args.port), db)
    latest = {}
    lock = threading.Lock()

    def on_object(objdef, inst_id, decoded):
        with lock:
            latest[objdef.name] = dict(decoded)
    threading.Thread(target=lambda: client.run(on_object=on_object, duration=100000),
                     daemon=True).start()
    time.sleep(1.5)
    att_o, act_o, fs_o, bs_o = db["AttitudeState"], db["ActuatorCommand"], db["FlightStatus"], db["AttitudeSettings"]
    client.transport.send(uavtalk.build_packet(uavtalk.TYPE_OBJ_REQ, bs_o.obj_id, 0))
    t_end = time.time() + args.seconds
    shown_rot = False
    while time.time() < t_end:
        for o in (att_o, act_o, fs_o):
            client.transport.send(uavtalk.build_packet(uavtalk.TYPE_OBJ_REQ, o.obj_id, 0))
        time.sleep(0.25)
        with lock:
            att = latest.get("AttitudeState")
            act = latest.get("ActuatorCommand")
            fs = latest.get("FlightStatus")
            rot = latest.get("AttitudeSettings")
        if rot and not shown_rot:
            shown_rot = True
            print("AttitudeSettings.BoardRotation (Roll,Pitch,Yaw) = %s" % (rot.get("BoardRotation"),))
        if not att:
            print("waiting for AttitudeState...", flush=True)
            continue
        pitch, roll = float(att["Pitch"]), float(att["Roll"])
        p_txt = ("NOSE DOWN %2.0f deg" % -pitch) if pitch < -3 else ("NOSE UP   %2.0f deg" % pitch) if pitch > 3 else "pitch level      "
        r_txt = ("LEFT SIDE DOWN  %2.0f deg" % -roll) if roll < -3 else ("RIGHT SIDE DOWN %2.0f deg" % roll) if roll > 3 else "roll level          "
        motors = ""
        if fs and fs.get("Armed") == "Armed" and act:
            ch = [int(c) for c in act.get("Channel", [])[:4]]
            if ch and max(ch) - min(ch) > 30:
                mean = sum(ch) / 4.0
                motors = "   motors high: " + " ".join("M%d" % (i + 1) for i, c in enumerate(ch) if c > mean)
        print("board says: %s   %s%s" % (p_txt, r_txt, motors), flush=True)


if __name__ == "__main__":
    main()
