#!/usr/bin/env python
"""Pre-check and calibrate the Go2 before running policies. Run it at the start of every session.

It takes you through each step, says what to do with the robot, and runs the test:

  connection  go2/check_connection.py: the link works and the state looks sane. Read-only.
  passive     go2/sysid.py passive: state rate, timing and sensor noise, IMU bias at rest.
              Read-only. Robot standing still in sport mode.
  inputs      go2/watch.py while you walk the robot with the remote in sport mode, then
              go2/diagnose.py's input check: each of the 37 policy inputs against a second
              source, and the velocity and height estimate against Unitree's own.
              Read-only.
  hang        go2/sysid.py hang: latency, torque scale, friction, tracking, and each foot
              sensor's unloaded reading. MOVES THE LEGS. Robot hanging, feet clear.
  stand       go2/sysid.py stand: standing sag and torque, standing height, and each foot
              sensor's loaded reading. MOVES THE ROBOT. Robot on the floor.

The steps that need sport mode come first, because hang releases it until the robot
is restarted. Results go to go2/logs/calibration/latest.json, which go2/run_policy.py
reads before every run: it refuses to drive the robot without one, if the inputs
don't mean what the policy expects, or if it's from another robot. Each step updates
the file as soon as it finishes, so rerun one step with --steps.

    python go2/calibrate.py --robot "$IFACE"                     # all steps
    python go2/calibrate.py --robot "$IFACE" --steps hang,stand  # redo some
    python go2/calibrate.py --show                               # print the current calibration

Exit status: 0 = ready to run policies, 1 = not ready (see the summary), 2 = stopped.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CALIBRATION_DIR = HERE / "logs" / "calibration"
LATEST = CALIBRATION_DIR / "latest.json"
STEPS = ("connection", "passive", "inputs", "hang", "stand")
INPUT_WALK_S = 120.0
MAX_AGE_H = 24.0  # run_policy.py warns about an older calibration
MIN_FORCE_SPAN = 5.0  # raw units between hanging and standing for a foot sensor to count as working
# Inputs that must mean the same as in S-M-ICRL before any policy runs (diagnose.CORE_INPUT_CHECKS).
CORE_INPUT_CHECKS = ("joint_angles", "joint_velocities", "rpy", "gyro", "gravity")
LATENCY_OK_MS = 10.0  # the argon sweep: class_argon_walk is unaffected up to here

INSTRUCTIONS = {
    "connection": "Robot on, standing in sport mode after start-up, cable connected.",
    "passive": "Robot standing still in sport mode. Stand back and don't touch it for 10 s.",
    "inputs": (f"Robot in sport mode, in the clear lane, remote in hand. When it starts, over "
               f"{INPUT_WALK_S:.0f} s: stand still 10 s; walk forward slowly about 2 m and back; "
               f"sideways about 1 m left and right; a half turn on the spot each way; then stand "
               f"still until it stops."),
    "hang": "Robot HANGING in the harness, all four feet clear of the ground. It will move the legs "
            "for about 2.5 minutes. Keep B ready.",
    "stand": "Robot ON THE FLOOR, harness slack. It will stand up and hold for 20 s. Keep B ready.",
}


# --------------------------------------------------------------------------- the file


def load_calibration(path=LATEST):
    """The calibration dict, or None if there is none."""
    path = Path(path)
    if not path.exists():
        return None
    return json.loads(path.read_text())


def calibration_problems(cal, serial=None, need_estimate=False):
    """Reasons not to drive the robot with this calibration: (refusals, warnings)."""
    refuse, warn = [], []
    if cal is None:
        return ["there is no calibration: run go2/calibrate.py first"], warn
    if serial is not None and cal.get("robot", {}).get("serial") not in (None, serial):
        refuse.append(f"the calibration is from robot {cal['robot']['serial']}, this is {serial}: "
                      f"run go2/calibrate.py")
    age_h = (time.time() - cal.get("updated", 0)) / 3600.0
    if age_h > MAX_AGE_H:
        warn.append(f"the calibration is {age_h:.0f} h old")
    flags = cal.get("inputs", {}).get("flags")
    if flags is None:
        warn.append("the calibration has no input check (step inputs)")
    else:
        bad = [k for k in CORE_INPUT_CHECKS if flags.get(k) is False]
        unknown = [k for k in CORE_INPUT_CHECKS if flags.get(k) is None]
        if bad:
            refuse.append(f"the input check failed for {', '.join(bad)}: those inputs don't mean on the "
                          f"robot what they mean in S-M-ICRL")
        if unknown:
            refuse.append(f"the input check couldn't verify {', '.join(unknown)}: redo step inputs, standing "
                          f"still, walking and turning as it says")
        if need_estimate:
            est_bad = [k for k, v in flags.items() if k.startswith(("velocity_vs_sport", "height_vs_sport")) and v is False]
            if est_bad:
                refuse.append(f"the velocity/height estimate disagreed with sport mode ({', '.join(est_bad)}): "
                              f"run with --fill-unmeasured or --no-estimate instead")
    return refuse, warn


def estimator_calibration(cal):
    """The section of the calibration go2/state_estimator.py takes."""
    return (cal or {}).get("estimator") or None


# --------------------------------------------------------------------------- steps


HUNG_UP = {"yes": False}


def _child_running_signal(signum, frame):
    """While a step runs, Ctrl-C and a closed terminal are the step's to handle: it stops
    and damps the robot. Interrupting here would kill it mid-damping. A handler, unlike
    SIG_IGN, is reset in the child, so the signal still reaches it."""
    if signum == signal.SIGHUP:
        HUNG_UP["yes"] = True
        try:
            fd = os.open(os.devnull, os.O_WRONLY)
            os.dup2(fd, 1)
            os.dup2(fd, 2)
        except OSError:
            pass


def run(cmd, log=None):
    """Run a step's script with this terminal's input, so its own prompts and B button work."""
    print(f"\n$ {' '.join(str(c) for c in cmd)}", flush=True)
    previous = {sig: signal.signal(sig, _child_running_signal) for sig in (signal.SIGINT, signal.SIGHUP)}
    try:
        if log is None:
            return subprocess.Popen([str(c) for c in cmd]).wait()
        with open(log, "w") as out:
            proc = subprocess.Popen([str(c) for c in cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                out.write(line)
                try:
                    sys.stdout.write(line)
                except OSError:
                    pass
            return proc.wait()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if HUNG_UP["yes"]:
            raise KeyboardInterrupt


def robot_args(args):
    return ["--robot", args.robot] if args.robot else []


def read_identity(args):
    """Serial number and firmware version from one rt/lowstate."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

    ChannelFactoryInitialize(0 if args.robot else 1, args.robot or "lo")
    # No handler: a listener callback outlives this function in DDS and segfaults once Python frees it.
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()
    msg = sub.Read(3.0)
    if msg is None:
        return None
    return {"serial": "-".join(str(x) for x in msg.sn), "version": "-".join(str(x) for x in msg.version)}


def step_connection(args, out):
    if args.robot:
        code = run([sys.executable, HERE / "check_connection.py", args.robot], log=out / "connection_check.txt")
        if code != 0:
            return None, "check_connection.py failed"
    identity = read_identity(args)
    if identity is None:
        return None, "no rt/lowstate received"
    return {"robot": identity}, None


def _load_npz(path):
    with np.load(path, allow_pickle=False) as d:
        return {k: d[k] for k in d.files}


def step_passive(args, out):
    log = out / "passive.npz"
    if run([sys.executable, HERE / "sysid.py", "passive", *robot_args(args), "--log", log]) != 0:
        return None, "sysid.py passive failed"
    d = _load_npz(log)
    results = json.loads(log.with_suffix(".json").read_text())["results"]
    accel = d["accel"]
    imu = {"gyro_bias": np.median(d["gyro"], axis=0).round(5).tolist(),
           "gyro_noise_std": d["gyro"].std(axis=0).round(5).tolist(),
           "accel_norm": round(float(np.median(np.linalg.norm(accel, axis=1))), 3),
           "accel_noise_std": accel.std(axis=0).round(4).tolist(),
           "rest_roll_pitch": np.median(d["rpy"][:, :2], axis=0).round(4).tolist()}
    return {"passive": {"files": [str(log)], "results": results, "imu": imu,
                        "foot_force_sport_stand": np.median(d["foot_force"], axis=0).round(1).tolist()}}, None


def step_inputs(args, out):
    log = out / "input_walk.npz"
    cmd = ["nice", "-n", "10", sys.executable, HERE / "watch.py", *robot_args(args),
           "--seconds", f"{args.walk_seconds:g}", "--log", log]
    if shutil.which("nice") is None:
        cmd = cmd[3:]
    if run(cmd) not in (0,) or not log.exists():
        return None, "watch.py recorded nothing"
    sys.path.insert(0, str(HERE))
    import diagnose

    w, _ = diagnose.load(log)
    lines, results, flags = diagnose.check_inputs(w)
    text = "\n".join("  " + line for line in lines)
    print("\nInput check:\n" + text, flush=True)
    (out / "inputs_check.txt").write_text(text + "\n")
    force = w["state_foot_force"].astype(float)
    accel_used = float(np.mean(w["state_est_accel_used"])) if "state_est_accel_used" in w else None
    return {"inputs": {"files": [str(log)], "flags": flags, "results": results,
                       "foot_force_walk_p5_p95": [np.percentile(force, 5, axis=0).round(1).tolist(),
                                                  np.percentile(force, 95, axis=0).round(1).tolist()],
                       "accel_used_share": None if accel_used is None else round(accel_used, 3)}}, None


def _stage_rows(d, *prefixes):
    stage = d["stage"].astype(str)
    return np.flatnonzero(np.logical_or.reduce([np.char.startswith(stage, p) for p in prefixes]))


def step_hang(args, out):
    log = out / "hang.npz"
    if run([sys.executable, HERE / "sysid.py", "hang", *robot_args(args), "--log", log]) != 0:
        return None, "sysid.py hang stopped early"
    d = _load_npz(log)
    results = json.loads(log.with_suffix(".json").read_text())["results"]
    rows = _stage_rows(d, "rest", "settling", "step", "sine", "ramp")
    unloaded = np.median(d["foot_force"][rows], axis=0) if rows.size else None
    return {"hang": {"files": [str(log)], "results": results,
                     "foot_force_unloaded": None if unloaded is None else unloaded.round(1).tolist()}}, None


def step_stand(args, out):
    log = out / "stand.npz"
    if run([sys.executable, HERE / "sysid.py", "stand", *robot_args(args), "--log", log]) != 0:
        return None, "sysid.py stand stopped early"
    d = _load_npz(log)
    results = json.loads(log.with_suffix(".json").read_text())["results"]
    rows = _stage_rows(d, "stand:hold")
    hold = rows[len(rows) // 4:] if rows.size else rows  # past the first settling
    standing = np.median(d["foot_force"][hold], axis=0) if hold.size else None
    height = float(np.median(d["est_height"][hold])) if hold.size and "est_height" in d else None
    return {"stand": {"files": [str(log)], "results": results,
                      "foot_force_standing": None if standing is None else standing.round(1).tolist(),
                      "standing_height_m": None if height is None else round(height, 4)}}, None


STEP_FUNCTIONS = {"connection": step_connection, "passive": step_passive, "inputs": step_inputs,
                  "hang": step_hang, "stand": step_stand}


# --------------------------------------------------------------------------- summary


def derive(cal):
    """Values run_policy.py and the estimator use, from whichever steps have run."""
    hang, stand = cal.get("hang") or {}, cal.get("stand") or {}
    inputs, passive = cal.get("inputs") or {}, cal.get("passive") or {}
    unloaded, standing = hang.get("foot_force_unloaded"), stand.get("foot_force_standing")
    if unloaded is None and inputs.get("foot_force_walk_p5_p95"):
        unloaded = inputs["foot_force_walk_p5_p95"][0]  # swing feet are unloaded
    if standing is None:
        standing = passive.get("foot_force_sport_stand")
    use_force = None
    if unloaded is not None and standing is not None:
        span = np.asarray(standing) - np.asarray(unloaded)
        use_force = bool(np.sum(span > MIN_FORCE_SPAN) >= 3)
    cal["estimator"] = {"foot_force_unloaded": unloaded, "foot_force_standing": standing, "use_force": use_force}
    latency = (hang.get("results") or {}).get("torque_fit", {}).get("delay_ms")
    cal["latency_ms"] = latency
    cal["standing_height_m"] = stand.get("standing_height_m")
    worst = max(latency.values()) if latency else None
    cal["recommended_policy"] = (None if worst is None else
                                 "class_argon_walk" if worst <= LATENCY_OK_MS else "func_argon_walk")
    return cal


def summary(cal):
    lines = []
    robot = cal.get("robot") or {}
    lines.append(f"Robot: serial {robot.get('serial', '?')}, firmware {robot.get('version', '?')}")
    for step in STEPS:
        info = (cal.get("steps") or {}).get(step)
        lines.append(f"  {step:<11} " + (f"done {info['time']}" + (f"  FAILED: {info['error']}" if info.get("error") else "")
                                         if info else "not run"))
    flags = (cal.get("inputs") or {}).get("flags") or {}
    if flags:
        lines.append("Input check: " + ", ".join(f"{k} {'OK' if v else ('CHECK' if v is False else 'n/a')}"
                                                for k, v in flags.items()))
    est = cal.get("estimator") or {}
    lines.append(f"Foot force sensors: {'used' if est.get('use_force') else 'not used' if est.get('use_force') is False else 'unknown'}"
                 f" (hanging {est.get('foot_force_unloaded')}, standing {est.get('foot_force_standing')})")
    if cal.get("latency_ms"):
        lines.append("Latency: " + ", ".join(f"{g} {v:.0f} ms" for g, v in cal["latency_ms"].items()))
    if cal.get("standing_height_m"):
        lines.append(f"Standing height (estimated): {cal['standing_height_m']:.3f} m (S-M-ICRL starts at 0.27)")
    if cal.get("recommended_policy"):
        lines.append(f"Run first (of the argon walks): {cal['recommended_policy']}")
    refuse, warn = calibration_problems(cal)
    missing = [s for s in ("inputs", "hang") if s not in (cal.get("steps") or {}) or cal["steps"][s].get("error")]
    if missing:
        refuse.append(f"not run yet: {', '.join(missing)}")
    lines.append("")
    lines.append("READY to run policies." if not refuse else "NOT READY: " + "; ".join(refuse))
    for w in warn:
        lines.append(f"Note: {w}")
    return "\n".join(lines), not refuse


def save(cal, out):
    cal["updated"] = time.time()
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    text = json.dumps(cal, indent=1, default=str)
    (out / "calibration.json").write_text(text)
    LATEST.write_text(text)


# --------------------------------------------------------------------------- main


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", metavar="INTERFACE", help="the robot's network interface; without it, the simulator")
    parser.add_argument("--steps", default=",".join(STEPS),
                        help="comma-separated steps to run, in this order: %(default)s")
    parser.add_argument("--walk-seconds", type=float, default=INPUT_WALK_S,
                        help="how long the inputs step records (default: %(default)s)")
    parser.add_argument("--fresh", action="store_true", help="start a new calibration instead of updating latest.json")
    parser.add_argument("--show", action="store_true", help="print the current calibration's summary and exit")
    args = parser.parse_args()
    args.steps = [s.strip() for s in args.steps.split(",") if s.strip()]
    unknown = set(args.steps) - set(STEPS)
    if unknown:
        parser.error(f"unknown steps: {', '.join(sorted(unknown))}")
    args.steps = [s for s in STEPS if s in args.steps]
    return args


def main():
    args = parse_args()
    if args.show:
        cal = load_calibration()
        if cal is None:
            print("No calibration yet: run go2/calibrate.py.")
            return 1
        text, ready = summary(derive(cal))
        print(text)
        return 0 if ready else 1
    cal = (None if args.fresh else load_calibration()) or {}
    if cal and cal.get("interface") not in (None, args.robot or "lo"):
        cal = {}  # a calibration from another setup
    cal.update(interface=args.robot or "lo", created=cal.get("created", time.time()))
    cal.setdefault("steps", {})
    out = CALIBRATION_DIR / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out.mkdir(parents=True, exist_ok=True)
    print(f"Calibration. Results: {LATEST}. Logs: {out}\n"
          f"Steps: {', '.join(args.steps)}. Press B or Ctrl-C at any time to stop the robot.", flush=True)
    try:
        for step in args.steps:
            print(f"\n=== {step} ===\n{INSTRUCTIONS[step]}", flush=True)
            answer = input("Press Enter to start, s to skip, q to stop: ").strip().lower()
            if answer == "q":
                break
            if answer == "s":
                continue
            result, error = STEP_FUNCTIONS[step](args, out)
            if result:
                if "robot" in result and cal.get("robot") not in (None, result["robot"]):
                    print("This is a different robot from the last calibration; starting over.", flush=True)
                    cal = {"interface": cal["interface"], "created": time.time(), "steps": {}}
                cal.update(result)
            cal["steps"][step] = {"time": datetime.now().isoformat(timespec="seconds"), "error": error,
                                  "log_dir": str(out)}
            save(derive(cal), out)
            if error:
                print(f"\n{step} FAILED: {error}", flush=True)
                if input("Carry on with the next step? [y/N] ").strip().lower() != "y":
                    break
    except (KeyboardInterrupt, EOFError):  # EOFError: the terminal closed at a prompt
        print("\nStopped.", flush=True)
        save(derive(cal), out)
        return 2
    text, ready = summary(derive(cal))
    save(cal, out)
    print("\n" + text)
    (out / "summary.txt").write_text(text + "\n")
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())
