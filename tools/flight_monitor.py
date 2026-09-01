#!/usr/bin/env python3
"""
flight_monitor - passive telemetry recorder + live watch for a REAL flight.

Connects to the board over UDP as the ONE UAVTalk client (close the GCS
before launching this - the firmware latches a single UDP peer, and two
clients silently steal each other's replies; that trap has burned this
project before). Then it mostly LISTENS: the flight side already pushes
AttitudeState at 25Hz, ManualControlCommand at 4Hz, SystemAlarms on
change, SystemStats at 1Hz. A gentle round-robin poll (one single-frame
request every 150ms, never back-to-back) fills in the objects telemetry
does not push: FlightStatus, StabilizationDesired, RateDesired,
ActuatorDesired, ActuatorCommand, GyroState.

Everything received is recorded to ~/NinjaPilot-logs/flightmon_<ts>.jsonl
with wall-clock timestamps, and a 1Hz status line shows the flight live:
arm state, mode, attitude, throttle, worst alarm, packet rate, and LINK
STALL warnings whenever the AttitudeState stream gaps >1s (the signature
of WiFi trouble, distinct from anything the vehicle does).

Usage:
    python3 flight_monitor.py [--host 192.168.0.45] [--port 9000]

Fly, land, disarm, then Ctrl+C: a post-flight summary prints arm/disarm
events, every alarm transition, attitude extremes, and stall windows.
The jsonl holds the full record for deeper analysis.
"""

import argparse
import json
import math
import os
import select
import socket
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
NINJAPILOT_ROOT = os.environ.get(
    "NINJAPILOT_ROOT",
    os.path.abspath(os.path.join(_HERE, "..", "..", "NinjaPilot-15.02.ninja")))
_PYUAVTALK = os.path.join(NINJAPILOT_ROOT, "ground", "pyuavtalk")
if not os.path.isdir(_PYUAVTALK):
    sys.exit("pyuavtalk not found at %s (set NINJAPILOT_ROOT)" % _PYUAVTALK)
sys.path.insert(0, _PYUAVTALK)
import uavtalk  # noqa: E402

GCS_DISCONNECTED, GCS_HANDSHAKEREQ, GCS_HANDSHAKEACK, GCS_CONNECTED = 0, 1, 2, 3

# One request per POLL_GAP, cycling this list: ~6.7 single frames per second
# total - far below anything that could crowd the telemetry loop, and never
# two sends back-to-back (that corrupts/drops one; learned the hard way).
POLL_OBJECTS = ["FlightStatus", "StabilizationDesired", "RateDesired",
                "ActuatorDesired", "ActuatorCommand", "GyroState"]
POLL_GAP = 0.15

ALARM_NAMES = ["SystemConfiguration", "BootFault", "OutOfMemory", "StackOverflow",
               "CPUOverload", "EventSystem", "Telemetry", "Receiver", "ManualControl",
               "Actuator", "Attitude", "Sensors", "Magnetometer", "Airspeed",
               "Stabilization", "Guidance", "PathPlan", "Battery", "FlightTime",
               "I2C", "GPS"]
ALARM_RANK = {"OK": 0, "Uninitialised": 0, "Warning": 1, "Error": 2, "Critical": 3}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.45")
    ap.add_argument("--port", type=int, default=9000)
    args = ap.parse_args()

    db = uavtalk.UAVObjectDB(os.path.join(NINJAPILOT_ROOT, "shared", "uavobjectdefinition"))
    gcs_stats = db["GCSTelemetryStats"]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    addr = (args.host, args.port)
    parser = uavtalk.UAVTalkParser()

    outdir = os.path.expanduser("~/NinjaPilot-logs")
    os.makedirs(outdir, exist_ok=True)
    outpath = os.path.join(outdir, "flightmon_%s.jsonl" % time.strftime("%Y-%m-%d_%H-%M-%S"))
    out = open(outpath, "w")

    print("flight_monitor: %s:%d -> %s" % (args.host, args.port, outpath))
    print("close the GCS if it is open (one UAVTalk client at a time), then fly.")
    print("Ctrl+C after landing for the summary.\n")

    state = {
        "gcs": GCS_HANDSHAKEREQ, "connected_at": None,
        "last_handshake": 0.0, "last_poll": 0.0, "poll_i": 0,
        "last_status": 0.0, "n_rx": 0, "n_rx_win": 0,
        "last_att": None, "att": (0.0, 0.0, 0.0), "thr": None, "pwm": None,
        "armed": None, "mode": None, "alarms": None,
        "stall_open": None,
    }
    events = []          # (wall, kind, text)
    stalls = []          # (start, end)
    att_min = [1e9, 1e9]
    att_max = [-1e9, -1e9]
    t0 = time.time()

    def record(name, inst, d, now):
        out.write(json.dumps({"t": round(now - t0, 3), "o": name, "i": inst, "d": d}) + "\n")

    def event(now, kind, text):
        events.append((now - t0, kind, text))
        print("[%7.1fs] %s: %s" % (now - t0, kind, text), flush=True)

    def send_gcs(status):
        payload = gcs_stats.pack({"Status": status})
        sock.sendto(uavtalk.build_packet(uavtalk.TYPE_OBJ, gcs_stats.obj_id, 0, payload), addr)

    try:
        while True:
            now = time.time()
            # handshake keepalive (also holds the firmware's UDP peer latch)
            if now - state["last_handshake"] > 1.0:
                state["last_handshake"] = now
                send_gcs(state["gcs"])
            # round-robin poll, one frame at a time
            if state["gcs"] == GCS_CONNECTED and now - state["last_poll"] > POLL_GAP:
                state["last_poll"] = now
                name = POLL_OBJECTS[state["poll_i"] % len(POLL_OBJECTS)]
                state["poll_i"] += 1
                o = db[name]
                sock.sendto(uavtalk.build_packet(uavtalk.TYPE_OBJ_REQ, o.obj_id, 0), addr)

            r, _, _ = select.select([sock], [], [], 0.05)
            if r:
                try:
                    data, _from = sock.recvfrom(65536)
                except OSError:
                    data = b""
                if data:
                    parser.feed(data)
            for mt, oid, iid, payload in parser.packets():
                if mt in (uavtalk.TYPE_ACK, uavtalk.TYPE_NACK, uavtalk.TYPE_OBJ_REQ):
                    continue
                o = db.by_id.get(oid)
                if o is None:
                    continue
                try:
                    d = o.describe(o.unpack(payload))
                except Exception:
                    continue
                # acked pushes (settings etc.) BLOCK the flight side's telemetry
                # until we reply; a client that stays silent turns the link into
                # a retry storm (measured: packet counts climbing without bound)
                if mt in (uavtalk.TYPE_OBJ_ACK, uavtalk.TYPE_OBJ_ACK_TS):
                    sock.sendto(uavtalk.build_packet(uavtalk.TYPE_ACK, oid, iid), addr)
                now = time.time()
                state["n_rx"] += 1
                state["n_rx_win"] += 1
                record(o.name, iid, d, now)

                if o.name == "FlightTelemetryStats":
                    fl = d.get("Status")
                    if fl == "HandshakeAck" and state["gcs"] != GCS_CONNECTED:
                        state["gcs"] = GCS_CONNECTED
                        send_gcs(state["gcs"])
                    elif fl == "Connected":
                        if state["connected_at"] is None:
                            state["connected_at"] = now
                            event(now, "LINK", "connected to flight side")
                    elif fl == "Disconnected" and state["gcs"] == GCS_CONNECTED:
                        state["gcs"] = GCS_HANDSHAKEREQ
                        event(now, "LINK", "flight side dropped to Disconnected")
                elif o.name == "AttitudeState":
                    if state["last_att"] is not None and now - state["last_att"] > 1.0:
                        stalls.append((state["last_att"] - t0, now - t0))
                        event(now, "STALL", "AttitudeState gap %.1fs" % (now - state["last_att"]))
                    state["last_att"] = now
                    state["att"] = (d["Roll"], d["Pitch"], d["Yaw"])
                    for i in range(2):
                        v = (d["Roll"], d["Pitch"])[i]
                        att_min[i] = min(att_min[i], v)
                        att_max[i] = max(att_max[i], v)
                elif o.name == "ManualControlCommand":
                    state["thr"] = d.get("Throttle")
                elif o.name == "ActuatorCommand":
                    state["pwm"] = d.get("Channel", [])[:4]
                elif o.name == "FlightStatus":
                    if d["Armed"] != state["armed"]:
                        event(now, "ARM", "%s (mode %s)" % (d["Armed"], d["FlightMode"]))
                        state["armed"] = d["Armed"]
                    if d["FlightMode"] != state["mode"]:
                        if state["mode"] is not None:
                            event(now, "MODE", str(d["FlightMode"]))
                        state["mode"] = d["FlightMode"]
                elif o.name == "SystemAlarms":
                    al = d.get("Alarm")
                    if al != state["alarms"]:
                        prev = state["alarms"]
                        for i, v in enumerate(al):
                            pv = prev[i] if prev else None
                            if pv is not None and v != pv and ALARM_RANK.get(v, 0) != ALARM_RANK.get(pv, 0):
                                nm = ALARM_NAMES[i] if i < len(ALARM_NAMES) else str(i)
                                event(now, "ALARM", "%s: %s -> %s" % (nm, pv, v))
                        state["alarms"] = al

            now = time.time()
            # live status line
            if now - state["last_status"] > 1.0:
                state["last_status"] = now
                al = state["alarms"]
                worst = ""
                if al:
                    bad = [(ALARM_NAMES[i] if i < len(ALARM_NAMES) else str(i)) + "=" + v
                           for i, v in enumerate(al) if ALARM_RANK.get(v, 0) >= 1]
                    worst = " ".join(bad[:4]) if bad else "all-OK"
                r_, p_, y_ = state["att"]
                pwm = state["pwm"]
                pwm_s = ("[" + " ".join("%4d" % c for c in pwm) + "]") if pwm else "[---- ---- ---- ----]"
                print("[%7.1fs] %s rp(%6.1f,%6.1f) yaw %6.1f thr %s pwm %s pkts/s %3d  %s"
                      % (now - t0,
                         "LINK" if state["gcs"] == GCS_CONNECTED else "....",
                         r_, p_, y_,
                         ("%.2f" % state["thr"]) if state["thr"] is not None else "  - ",
                         pwm_s, state["n_rx_win"], worst), flush=True)
                state["n_rx_win"] = 0
                # stall detection also from the quiet side
                if state["last_att"] and now - state["last_att"] > 1.0 and state["stall_open"] is None:
                    state["stall_open"] = state["last_att"]
                    event(now, "STALL", "AttitudeState silent since t=%.1fs" % (state["last_att"] - t0))
                if state["stall_open"] and state["last_att"] and now - state["last_att"] < 1.0:
                    state["stall_open"] = None
    except KeyboardInterrupt:
        pass
    finally:
        out.close()

    dur = time.time() - t0
    print("\n===== flight_monitor summary =====")
    print("duration %.1fs, %d packets, log: %s" % (dur, state["n_rx"], outpath))
    if att_min[0] < 1e8:
        print("attitude extremes: roll [%.1f, %.1f]  pitch [%.1f, %.1f]"
              % (att_min[0], att_max[0], att_min[1], att_max[1]))
    if stalls:
        print("link stalls (>1s AttitudeState gaps):")
        for a, b in stalls:
            print("  t=%.1fs -> %.1fs (%.1fs)" % (a, b, b - a))
    else:
        print("no link stalls")
    print("events:")
    for t, kind, text in events:
        print("  [%7.1fs] %-6s %s" % (t, kind, text))
    print("analyze with: the jsonl above (wall-stamped, every object received)")


if __name__ == "__main__":
    main()
