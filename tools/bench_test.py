#!/usr/bin/env python3
"""
bench_test - supervised props-OFF bench characterization, driven by the tool.

You hold the quad and tilt/roll it gently in all directions; this tool owns
the throttle: it applies the recommended settings, temporarily takes control
(GCS receiver mapping + Always Armed, in RAM ONLY - a power cycle restores
your DSM/arming no matter what), arms, walks the throttle from idle to max
over ~2 minutes with periodic HOLD-STILL windows, and samples the whole
chain: gyro, attitude, accel, and per-motor PWM.

What it grades:
  gyro vs attitude     the estimate must follow the gyro (sign + gain)
  accel vs attitude    the estimate must agree with gravity - the exact
                       divergence that caused the 2026-09-01 flips, live
  motor response       tilt a side down -> that side's motors must rise
                       (pitch: front pair vs rear; roll: left pair vs right)
  vibration vs rpm     gyro/accel high-frequency noise per throttle step,
                       measured in the HOLD-STILL windows (flags resonant
                       throttle bands; with the 41Hz DLPF this stays low)

  PROPS OFF. Battery power (never USB with battery). No GCS, no
  flight_monitor running. Type PROPS OFF at the prompt to begin.

    python3 bench_test.py [--host 192.168.0.45] [--max 0.90] [--time 100]
"""

import argparse
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time

# --- voice guidance (macOS `say`): your hands are on the quad ---------------
_say_q = queue.Queue()
_say_enabled = [True]


def _say_worker():
    while True:
        msg = _say_q.get()
        if msg is None:
            return
        if _say_enabled[0]:
            try:
                subprocess.run(["say", msg], timeout=15)
            except Exception:
                _say_enabled[0] = False


def say(msg):
    print("[voice] %s" % msg, flush=True)
    _say_q.put(msg)


threading.Thread(target=_say_worker, daemon=True).start()

_HERE = os.path.dirname(os.path.abspath(__file__))
NINJAPILOT_ROOT = os.environ.get(
    "NINJAPILOT_ROOT",
    os.path.abspath(os.path.join(_HERE, "..", "..", "NinjaPilot-15.02.ninja")))
sys.path.insert(0, os.path.join(NINJAPILOT_ROOT, "ground", "pyuavtalk"))
import uavtalk  # noqa: E402
from uavtalk_client import UdpTransport, UAVTalkClient, default_xml_dir  # noqa: E402
import flight_config as bov  # noqa: E402


ALARM_NAMES = ["SystemConfiguration", "BootFault", "OutOfMemory", "StackOverflow",
               "CPUOverload", "EventSystem", "Telemetry", "Receiver", "ManualControl",
               "Actuator", "Attitude", "Sensors", "Magnetometer", "Airspeed",
               "Stabilization", "Guidance", "PathPlan", "Battery", "FlightTime",
               "I2C", "GPS"]


def corr(xs, ys):
    n = len(xs)
    if n < 8:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx < 1e-9 or sy < 1e-9:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.45")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--max", type=float, default=0.90)
    ap.add_argument("--time", type=float, default=100.0)
    ap.add_argument("--skip-apply", action="store_true",
                    help="skip applying the recommended settings first")
    ap.add_argument("--quiet", action="store_true", help="no voice prompts")
    args = ap.parse_args()
    if args.quiet:
        _say_enabled[0] = False

    print(__doc__.split("What it grades")[0])
    if input("type PROPS OFF to begin: ").strip().upper() != "PROPS OFF":
        sys.exit("aborted - props confirmation not given")

    db = uavtalk.UAVObjectDB(default_xml_dir())
    client = UAVTalkClient(UdpTransport(args.host, args.port), db)
    latest = {}
    lock = threading.Lock()
    connected = {"ok": False}

    def on_object(objdef, inst_id, decoded):
        with lock:
            latest[objdef.name] = (time.time(), decoded)

    threading.Thread(target=lambda: client.run(on_object=on_object,
                                               on_connected=lambda: connected.update(ok=True),
                                               duration=100000), daemon=True).start()
    deadline = time.time() + 10
    while not connected["ok"] and time.time() < deadline:
        time.sleep(0.1)
    if not connected["ok"]:
        sys.exit("no link - board up? GCS closed?")
    print("[link] connected")

    def get(name):
        with lock:
            e = latest.get(name)
        return dict(e[1]) if e else None

    def fetch(name, timeout=5.0):
        with lock:
            latest.pop(name, None)
        o = db[name]
        end = time.time() + timeout
        while time.time() < end:
            client.transport.send(uavtalk.build_packet(uavtalk.TYPE_OBJ_REQ, o.obj_id, 0))
            for _ in range(10):
                v = get(name)
                if v is not None:
                    return v
                time.sleep(0.05)
        sys.exit("could not read %s" % name)

    def _approx(a, b):
        if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
            return len(a) == len(b) and all(_approx(x, y) for x, y in zip(a, b))
        try:
            fa, fb = float(a), float(b)
            return abs(fa - fb) <= max(1e-9, 1e-4 * max(abs(fa), abs(fb)))
        except (TypeError, ValueError):
            return str(a) == str(b)

    def write_verify(name, values, check_keys):
        client.send_object(name, values)
        time.sleep(0.15)
        client.send_object(name, values)
        time.sleep(0.25)
        back = fetch(name)
        for k in check_keys:
            if not _approx(back.get(k), values.get(k)):
                sys.exit("VERIFY FAILED %s.%s: wrote %r, board has %r"
                         % (name, k, values.get(k), back.get(k)))
        return back

    def save(name):
        o = db[name]
        with lock:
            latest.pop("ObjectPersistence", None)
        client.send_object("ObjectPersistence", {"Operation": "Save",
                                                 "Selection": "SingleObject",
                                                 "ObjectID": o.obj_id, "InstanceID": 0})
        end = time.time() + 5.0
        while time.time() < end:
            got = get("ObjectPersistence")
            if got and got.get("Operation") in ("Completed", "Error"):
                if got["Operation"] == "Error":
                    sys.exit("board ERROR saving %s" % name)
                return
            time.sleep(0.1)
        sys.exit("no save confirmation for %s" % name)

    # ---- 1. apply the recommended settings (persisted) -------------------
    if not args.skip_apply:
        print("[apply] recommended settings...")
        bank = fetch("StabilizationSettingsBank1")
        bank["RollRatePID"] = [0.0032, 0.0075, 0.00005, bank["RollRatePID"][3]]
        bank["PitchRatePID"] = [0.0032, 0.0075, 0.00005, bank["PitchRatePID"][3]]
        bank["RollPI"] = [3.2, bank["RollPI"][1], bank["RollPI"][2]]
        bank["PitchPI"] = [3.2, bank["PitchPI"][1], bank["PitchPI"][2]]
        write_verify("StabilizationSettingsBank1", bank, ["RollRatePID", "PitchPI"])
        save("StabilizationSettingsBank1")
        fms0 = fetch("FlightModeSettings")
        for slot in (1, 2, 3):
            m = list(fms0["Stabilization%dSettings" % slot])
            if m[2] == "AxisLock":
                m[2] = "Rate"
            fms0["Stabilization%dSettings" % slot] = m
        write_verify("FlightModeSettings", fms0, ["Stabilization1Settings"])
        save("FlightModeSettings")
        act = fetch("ActuatorSettings")
        act["MotorsSpinWhileArmed"] = "TRUE"
        write_verify("ActuatorSettings", act, ["MotorsSpinWhileArmed"])
        save("ActuatorSettings")
        print("[apply] done (persisted)")

    # ---- 2. take control: GCS mapping + Always Armed, RAM ONLY -----------
    mcs_orig = fetch("ManualControlSettings")
    fms_orig = fetch("FlightModeSettings")
    print("[ctrl] remapping receiver to GCS + Always Armed (RAM only - a")
    print("       power cycle restores your DSM/arming unconditionally)")
    mcs = dict(mcs_orig)
    mcs["ChannelGroups"] = ["GCS"] * 5 + list(mcs_orig["ChannelGroups"][5:])
    mcs["ChannelNumber"] = [1, 2, 3, 4, 5] + list(mcs_orig["ChannelNumber"][5:])
    mcs["ChannelMin"] = [1000] * 5 + list(mcs_orig["ChannelMin"][5:])
    mcs["ChannelNeutral"] = [1050, 1500, 1500, 1500, 1500] + list(mcs_orig["ChannelNeutral"][5:])
    mcs["ChannelMax"] = [2000] * 5 + list(mcs_orig["ChannelMax"][5:])
    write_verify("ManualControlSettings", mcs, ["ChannelGroups"])

    outpath = os.path.expanduser("~/NinjaPilot-logs/bench_%s.jsonl"
                                 % time.strftime("%Y-%m-%d_%H-%M-%S"))
    out = open(outpath, "w")
    t0 = time.time()
    throttle = [0.0]
    stop = threading.Event()
    n_modes = 6 if True else 5

    def gcs_channels():
        thr_us = int(1000 + max(0.0, throttle[0]) * 1000)
        return [thr_us, 1500, 1500, 1500, bov.flight_mode_channel(0, 6), 1500, 1500, 1500]

    def stick_pump():
        # GCSReceiver keepalive at 20Hz (the 100ms receiver supervisor) with
        # sensor polls interleaved, never back-to-back
        polls = ["GyroState", "AccelState", "ActuatorCommand", "FlightStatus"]
        i = 0
        while not stop.is_set():
            client.send_object("GCSReceiver", {"Channel": gcs_channels()})
            time.sleep(0.025)
            o = db[polls[i % len(polls)]]
            client.transport.send(uavtalk.build_packet(uavtalk.TYPE_OBJ_REQ, o.obj_id, 0))
            i += 1
            time.sleep(0.025)
    pump = threading.Thread(target=stick_pump, daemon=True)
    pump.start()
    time.sleep(1.0)

    # ---- 3. the ramp, voice-directed --------------------------------------
    samples = []
    say("Bench test ready.")
    time.sleep(2.5)
    say("Hold the quad level. Props off?")
    time.sleep(3.5)

    # Confirm the GCS receiver is actually feeding control BEFORE arming -
    # this is the check whose absence let the old version narrate a state
    # that never happened. If throttle is not reaching the flight side,
    # arming (Always Armed = arm when throttle low) cannot occur.
    mcc = fetch("ManualControlCommand")
    thr_in = mcc.get("Throttle")
    conn = mcc.get("Connected")
    print("[check] ManualControlCommand: Throttle=%s Connected=%s" % (thr_in, conn), flush=True)
    if conn not in ("True", True) or thr_in is None or float(thr_in) >= 0:
        alarms = fetch("SystemAlarms").get("Alarm", [])
        bad = [ALARM_NAMES[i] + "=" + v for i, v in enumerate(alarms)
               if i < len(ALARM_NAMES) and v not in ("OK", "Uninitialised")]
        say("Stop. The board is not receiving my throttle. Aborting.")
        print("[ABORT] GCS control not reaching the flight side.")
        print("        Throttle=%s (need < 0), Connected=%s" % (thr_in, conn))
        print("        alarms: %s" % (", ".join(bad) or "none"))
        print("        This board needs the GCS-receiver firmware "
              "(firmware_normal_41hz.bin rebuilt 2026-09-01 or later).")
        raise SystemExit(1)

    # mixer/curve must be real or arming spins nothing (the simwroom sim's
    # own failure mode - a zero mixer produces zero PWM regardless)
    mix = fetch("MixerSettings")
    curve = mix.get("ThrottleCurve1", [])
    m1 = mix.get("Mixer1Vector", [0, 0, 0, 0, 0])
    if not any(float(c) > 0 for c in curve) or not any(v != 0 for v in m1[2:5]):
        say("Stop. The mixer is not configured. Aborting.")
        print("[ABORT] MixerSettings not usable - ThrottleCurve1=%s Mixer1Vector=%s"
              % ([round(float(c), 2) for c in curve], list(m1)))
        write_verify("ManualControlSettings", mcs_orig, ["ChannelGroups"])
        raise SystemExit(1)

    say("Receiver check good. Arming.")
    time.sleep(2.0)
    fms = dict(fms_orig)
    fms["Arming"] = "Always Armed"
    write_verify("FlightModeSettings", fms, ["Arming"])

    # VERIFY the board actually armed - poll FlightStatus, do not assume
    armed = False
    end = time.time() + 5.0
    while time.time() < end:
        fs = get("FlightStatus")
        if fs and fs.get("Armed") == "Armed":
            armed = True
            break
        time.sleep(0.1)
    if not armed:
        alarms = fetch("SystemAlarms").get("Alarm", [])
        bad = [ALARM_NAMES[i] + "=" + v for i, v in enumerate(alarms)
               if i < len(ALARM_NAMES) and v not in ("OK", "Uninitialised")]
        say("The board did not arm. Aborting.")
        print("[ABORT] FlightStatus never reached Armed. alarms: %s"
              % (", ".join(bad) or "none"))
        write_verify("FlightModeSettings", fms_orig, ["Arming"])
        write_verify("ManualControlSettings", mcs_orig, ["ChannelGroups"])
        raise SystemExit(1)

    # confirm motors actually idle (MotorsSpinWhileArmed) before ramping
    time.sleep(0.5)
    ac = fetch("ActuatorCommand").get("Channel", [0] * 4)[:4]
    print("[check] armed. idle ActuatorCommand: %s" % ac, flush=True)
    if not any(c > 1050 for c in ac):
        say("Armed, but motors are not idling. Continuing, watch closely.")
        print("[WARN] no idle PWM > 1050 - MotorsSpinWhileArmed may be off, "
              "or outputs disabled. PWM: %s" % ac)
    else:
        say("Armed. Motors idling. Beginning the ramp.")
    say("Follow my directions. Gentle tilts.")

    # repeating voice-directed motion cycle; each phase labels its samples
    CYCLE = [("nose_down", "Nose down.", 3.5),
             ("nose_up", "Nose up.", 3.5),
             ("roll_left", "Roll left.", 3.5),
             ("roll_right", "Roll right.", 3.5),
             ("level", "Back to level.", 2.0),
             ("still", None, 4.0)]  # announced with throttle percentage
    ramp_t0 = time.time()
    phase = ["level"]
    cyc_i = [0]
    phase_until = [0.0]
    try:
        last_status = 0.0
        while True:
            now = time.time()
            el = now - ramp_t0
            if el > args.time + 4.0:
                break
            throttle[0] = min(args.max, args.max * el / args.time)
            if now >= phase_until[0]:
                name, prompt, dur = CYCLE[cyc_i[0] % len(CYCLE)]
                cyc_i[0] += 1
                phase[0] = name
                phase_until[0] = now + dur
                if name == "still":
                    say("Hold still at %d percent. Three. Two. One." % int(throttle[0] * 100))
                else:
                    say(prompt)
            att = get("AttitudeState") or {}
            gyro = get("GyroState") or {}
            acc = get("AccelState") or {}
            pwm = (get("ActuatorCommand") or {}).get("Channel", [0] * 4)[:4]
            row = {"t": round(now - t0, 3), "thr": round(throttle[0], 3),
                   "phase": phase[0],
                   "att": [att.get("Roll", 0), att.get("Pitch", 0), att.get("Yaw", 0)],
                   "gyro": [gyro.get("x", 0), gyro.get("y", 0), gyro.get("z", 0)],
                   "acc": [acc.get("x", 0), acc.get("y", 0), acc.get("z", 0)],
                   "pwm": pwm}
            samples.append(row)
            out.write(json.dumps(row) + "\n")
            if now - last_status > 2.0:
                last_status = now
                print("[%5.1fs] thr %4.0f%% att(%6.1f,%6.1f) gyro(%6.1f,%6.1f,%6.1f) pwm %s"
                      % (el, throttle[0] * 100, row["att"][0], row["att"][1],
                         row["gyro"][0], row["gyro"][1], row["gyro"][2], pwm), flush=True)
            time.sleep(0.04)
    except KeyboardInterrupt:
        say("Interrupted. Shutting down safely.")
    finally:
        say("Ramp complete. Throttle down.")
        throttle[0] = 0.0
        time.sleep(1.0)
        say("Disarming and restoring your radio.")
        stop.set()
        time.sleep(0.2)
        # restore original control ownership (and disarm via original Arming)
        try:
            write_verify("FlightModeSettings", fms_orig, ["Arming"])
            write_verify("ManualControlSettings", mcs_orig, ["ChannelGroups"])
            say("Your radio is back in control. Test done. You can put the quad down.")
        except SystemExit:
            say("Warning. Restore failed. Power cycle the board to get your radio back.")
        out.close()
        time.sleep(4.0)  # let the last voice prompts finish

    # ---- 4. grades --------------------------------------------------------
    print("\n===== bench report ===== (%d samples, log %s)" % (len(samples), outpath))
    move = [s for s in samples if s["phase"] not in ("still",) and s["thr"] > 0.05]
    # per-stimulus compliance + mixer direction, from the labeled phases
    def phase_mean(ph, fn):
        vals = [fn(s) for s in samples if s["phase"] == ph and 0.15 < s["thr"] < 0.85
                and all(p > 900 for p in s["pwm"])]
        return (sum(vals) / len(vals)) if len(vals) > 10 else None
    checks = [
        ("nose_down", lambda s: s["att"][1], "<", -3, "estimate sees nose down"),
        ("nose_up", lambda s: s["att"][1], ">", 3, "estimate sees nose up"),
        ("roll_left", lambda s: s["att"][0], "<", -3, "estimate sees roll left"),
        ("roll_right", lambda s: s["att"][0], ">", 3, "estimate sees roll right"),
        ("nose_down", lambda s: (s["pwm"][0] + s["pwm"][1]) - (s["pwm"][2] + s["pwm"][3]),
         ">", 5, "front motors rise when nose drops"),
        ("nose_up", lambda s: (s["pwm"][0] + s["pwm"][1]) - (s["pwm"][2] + s["pwm"][3]),
         "<", -5, "rear motors rise when nose lifts"),
        ("roll_left", lambda s: (s["pwm"][0] + s["pwm"][3]) - (s["pwm"][1] + s["pwm"][2]),
         ">", 5, "left motors rise when left side drops"),
        ("roll_right", lambda s: (s["pwm"][0] + s["pwm"][3]) - (s["pwm"][1] + s["pwm"][2]),
         "<", -5, "right motors rise when right side drops"),
    ]
    for ph, fn, op, thresh, desc in checks:
        m = phase_mean(ph, fn)
        if m is None:
            print("  [ ?? ] %-38s insufficient samples" % desc)
        else:
            good = (m < thresh) if op == "<" else (m > thresh)
            print("  [ %s ] %-38s mean %+.1f during '%s'"
                  % ("ok".center(4) if good else "FAIL", desc, m, ph))
    if len(move) > 50:
        d_att = []
        for a, b in zip(move, move[1:]):
            dt = b["t"] - a["t"]
            if 0.01 < dt < 0.2:
                d_att.append(((b["att"][1] - a["att"][1]) / dt, a["gyro"][1],
                              (b["att"][0] - a["att"][0]) / dt, a["gyro"][0]))
        c_pg = corr([x[1] for x in d_att], [x[0] for x in d_att])
        c_rg = corr([x[3] for x in d_att], [x[2] for x in d_att])
        for name, c in (("gyro.y vs d(pitch)/dt", c_pg), ("gyro.x vs d(roll)/dt", c_rg)):
            if c is None:
                print("  [ ?? ] %-24s insufficient motion" % name)
            else:
                lvl = "ok" if c > 0.5 else ("FAIL" if c < -0.2 else "WARN")
                print("  [ %s ] %-24s r=%+.2f %s" % (lvl.center(4), name, c,
                      "(estimate follows gyro)" if c > 0.5 else "(SIGN/COUPLING PROBLEM)" if c < -0.2 else "(weak - tilt more next run)"))
        # accel agreement: pitch-from-accel vs estimate, in slow motion only
        pa = [(math.degrees(math.asin(max(-1, min(1, s["acc"][0] / 9.81)))), s["att"][1])
              for s in move if abs(s["gyro"][1]) < 30 and 8.5 < math.sqrt(sum(v * v for v in s["acc"])) < 11.0]
        c_aa = corr([p[0] for p in pa], [p[1] for p in pa]) if len(pa) > 30 else None
        if c_aa is None:
            print("  [ ?? ] %-24s insufficient clean samples" % "accel vs estimate")
        else:
            lvl = "ok" if c_aa > 0.7 else ("FAIL" if c_aa < 0.3 else "WARN")
            print("  [ %s ] %-24s r=%+.2f %s" % (lvl.center(4), "accel vs estimate", c_aa,
                  "(no vibration-divergence)" if c_aa > 0.7 else "(THE FLIP SIGNATURE - estimate not tracking gravity)"))
        # motor response: does the mixer raise the dropped side
        mr = [(s["att"][1], (s["pwm"][0] + s["pwm"][1]) - (s["pwm"][2] + s["pwm"][3]),
               s["att"][0], (s["pwm"][0] + s["pwm"][3]) - (s["pwm"][1] + s["pwm"][2]))
              for s in move if 0.15 < s["thr"] < 0.85 and all(p > 900 for p in s["pwm"])]
        c_pm = corr([x[0] for x in mr], [x[1] for x in mr])
        c_rm = corr([x[2] for x in mr], [x[3] for x in mr])
        for name, c in (("pitch vs front-rear PWM", c_pm), ("roll vs left-right PWM", c_rm)):
            if c is None:
                print("  [ ?? ] %-24s insufficient data" % name)
            else:
                lvl = "ok" if c < -0.3 else ("FAIL" if c > 0.3 else "WARN")
                print("  [ %s ] %-24s r=%+.2f %s" % (lvl.center(4), name, c,
                      "(raises the dropped side)" if c < -0.3 else "(WRONG DIRECTION - would amplify a tilt!)" if c > 0.3 else "(weak)"))
    else:
        print("  not enough motion samples for direction grades")

    still = [s for s in samples if s["phase"] == "still"]
    if still:
        print("  vibration in HOLD-STILL windows (sample-to-sample deltas):")
        buckets = {}
        for a, b in zip(still, still[1:]):
            if b["t"] - a["t"] < 0.2:
                key = int(a["thr"] * 100 // 20) * 20
                dg = max(abs(b["gyro"][i] - a["gyro"][i]) for i in range(3))
                da = abs(math.sqrt(sum(v * v for v in b["acc"])) - math.sqrt(sum(v * v for v in a["acc"])))
                buckets.setdefault(key, []).append((dg, da))
        for key in sorted(buckets):
            vals = buckets[key]
            mg = sum(v[0] for v in vals) / len(vals)
            ma = sum(v[1] for v in vals) / len(vals)
            flag = "" if mg < 15 else "  <-- noisy band"
            print("    %2d-%2d%% thr: gyro delta ~%5.1f deg/s, |a| delta ~%4.2f m/s2%s"
                  % (key, key + 19, mg, ma, flag))
    print("\nsend me the jsonl for the deep pass: %s" % outpath)


if __name__ == "__main__":
    main()
