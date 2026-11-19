#!/usr/bin/env python
"""Find out why a policy that works in S-M-ICRL doesn't work on the robot.

  reference  Run the policy file through S-M-ICRL's own trial code (its env, RobotAPI,
             latency and frequencies, from a task config) and record it in the same
             form as a robot run. Run it in S-M-ICRL's Python environment.
  report     Compare a go2/run_policy.py log with that reference, and with a
             go2/watch.py recording of the same run if there is one. Prints the
             S-M-ICRL sweep's columns for both, the diagnostics behind them, and the
             likely causes, and saves them as .json next to the log.
  inputs     Check, from a go2/watch.py recording, that each of the 37 robot_state
             inputs means on the robot what it means in S-M-ICRL: every input is
             compared with a second, independent source (the IMU's own rpy, the
             quaternion's rate, gravity in the accelerometer, Unitree's foot positions,
             and sport mode's velocity and height). Record it with the robot walking
             in sport mode on the remote, so Unitree's estimator is running.

    python go2/diagnose.py reference go2/policies/walk.py --smicrl ../S-MICRL \\
        --config func_config/go2/walk/walk.yaml               # -> go2/logs/reference/<policy>.npz
    python go2/diagnose.py report go2/logs/<time>.npz --watch go2/logs/watch/<time>.npz \\
        --reference go2/logs/reference/walk.npz
    python go2/diagnose.py inputs go2/logs/watch/<time>.npz

Robot run, in three terminals: sim or robot connection as usual, then
    nice -n 10 python go2/watch.py --robot
    python go2/run_policy.py POLICY.py --robot --policy-hz 10 --seconds 10

Metrics, the same for the robot and the reference:
  x, y      displacement over the walk, in the starting heading's frame (m); the robot's
            comes from rt/utlidar/robot_odom
  vx        x / walk duration (m/s)
  yaw       heading change (rad)
  minh      lowest base height (m); the robot's is robot_odom's z
  sat%      share of joint samples with |torque| >= 95% of the limit
  sw/call   share of policy calls whose target moved more than 0.1 rad on some joint
  reach     per policy call, how much of the commanded joint change was done by the
            next call, for changes over 0.1 rad: the joints' speed under load, in the
            policy's own terms
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
GROUPS = {"hip": [0, 3, 6, 9], "thigh": [1, 4, 7, 10], "calf": [2, 5, 8, 11]}
TORQUE_LIMIT = np.array([23.7, 23.7, 45.43] * 4)  # as run_policy.py and the MuJoCo model
SAT_SHARE = 0.95
SWITCH = 0.1  # rad, as run_policy.py counts target changes
LEGS = ("FR", "FL", "RR", "RL")
STATE_NAMES = ([f"q_{l}_{p}" for l in LEGS for p in ("hip", "thigh", "calf")]
               + [f"dq_{l}_{p}" for l in LEGS for p in ("hip", "thigh", "calf")]
               + ["roll", "pitch", "yaw_dev", "gyro_x", "gyro_y", "gyro_z", "vx", "vy", "vz",
                  "height", "up_x", "up_y", "up_z"])
UNMEASURED = slice(30, 34)  # vx, vy, vz, height: the robot feeds NaN or constants

# Thresholds for the likely-causes list.
LATENCY_WARN_MS = 15.0  # the walk sweep: fine at 10 ms, half the distance at 20 ms
SW_DROP = 0.1
REACH_DROP = 0.15
SAT_RISE_PCT = 5.0
LATE_CALL = 1.5  # a call this many periods after the last counts as late
INPUT_SHIFT = 2.0  # in reference standard deviations
INPUT_MIN_DIFF = 0.03  # rad, rad/s or unitless; smaller mean differences don't matter


# --------------------------------------------------------------------------- reference


def reference(args):
    """Run the policy with S-M-ICRL's _run_single_trial, recording every step and call.

    RobotAPI._build_robot_state and RobotAPI._step are wrapped, not replaced, so the run
    goes through S-M-ICRL's own env.step, PD law, latency and frequencies.
    """
    root = args.smicrl.resolve()
    code = args.policy.resolve().read_text()
    out = (args.out or HERE / "logs" / "reference" / f"{args.policy.stem}.npz").resolve()
    config_path = Path(args.config)
    os.chdir(root)  # its configs give paths relative to the repo
    sys.path.insert(0, str(root))
    from config import load_config
    from runner import primitivellm
    from world.robot_api import RobotAPI

    cfg = load_config(config_path)
    rows, calls = [], []
    pending = {"call": None}
    build_state, step = RobotAPI._build_robot_state, RobotAPI._step

    def recording_build_state(self, *a, **k):
        state = build_state(self, *a, **k)
        pending["call"] = (float(self.env.data.time), np.array(state[:37], dtype=float))
        return state

    def recording_step(self, action, *a, **k):
        target = np.array(action, dtype=float)[:12].copy()
        if pending["call"] is not None:
            calls.append(pending["call"] + (target,))
            pending["call"] = None
        step(self, action, *a, **k)
        data, env = self.env.data, self.env
        rows.append((float(data.time), target, data.qpos[env._qpos_idx][:12].copy(),
                     data.qvel[env._qvel_idx][:12].copy(), np.array(data.ctrl[:12]),
                     data.qpos[:7].copy()))

    RobotAPI._build_robot_state, RobotAPI._step = recording_build_state, recording_step
    try:
        result = primitivellm._run_single_trial(cfg, code, out.with_suffix(".mp4"), render=False)
    finally:
        RobotAPI._build_robot_state, RobotAPI._step = build_state, step
    reward, error = result[0], result[2]
    if not rows:
        sys.exit(f"the policy didn't run: {error}")
    n_walk = min(len(rows), int(cfg.task.rollout_steps)) if getattr(cfg.task, "rollout_steps", None) else len(rows)
    t, target, q, dq, tau, base = (np.array(c) for c in zip(*rows))
    call_t, call_state, call_target = (np.array(c) for c in zip(*calls))
    stage = np.array(["walking"] * n_walk + ["settling"] * (len(rows) - n_walk))
    freq = cfg.frequencies
    gains = cfg.stiffness_modes.get(getattr(cfg.runner, "stiffness_mode", "normal"),
                                    list(cfg.stiffness_modes.values())[0])
    meta = {"source": "smicrl", "policy": str(args.policy), "config": str(config_path),
            "smicrl": str(root), "reward": None if reward is None else float(reward), "error": error or None,
            "policy_hz": float(freq.policy_hz) if freq.enabled else None,
            "env_hz": float(freq.env_hz) if freq.enabled else None,
            "kp": float(gains.kp), "kd": float(gains.kd), "rollout_steps": n_walk}
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, meta=json.dumps(meta), t=t, stage=stage, target=target, q_cmd=target,
                        q=q, dq=dq, tau_est=tau, base_pose=base, call_t=call_t, call_state=call_state,
                        call_target=call_target)
    print(f"Reference: reward {meta['reward']}, {len(calls)} policy calls, {n_walk} walking steps")
    print(f"Saved: {out}")
    return 0


# --------------------------------------------------------------------------- loading


def load(path):
    with np.load(path, allow_pickle=False) as npz:
        d = {k: npz[k] for k in npz.files}
    meta = json.loads(str(d.pop("meta"))) if "meta" in d else {}
    if "stage" in d:
        d["stage"] = d["stage"].astype(str)
    return d, meta


def yaw_of(qw, qx, qy, qz):
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def tilt_of(qw, qx, qy, qz):
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    return np.maximum(np.abs(roll), np.abs(pitch))


def walk_window(d):
    rows = np.flatnonzero(d["stage"] == "walking")
    if rows.size == 0:
        rows = np.flatnonzero(d["stage"] == "playing")
    return rows


def pose_track(d, t0, t1):
    """(t, x, y, z, yaw) of the base during [t0, t1]: from the sim, from robot_odom, or,
    without either, by integrating the estimator's velocity in the logged robot_state."""
    if "base_pose" in d:
        sel = (d["t"] >= t0) & (d["t"] <= t1)
        p, t = d["base_pose"][sel], d["t"][sel]
    elif "odom_pose" in d and len(d["odom_pose"]):
        sel = (d["odom_t"] >= t0) & (d["odom_t"] <= t1)
        p, t = d["odom_pose"][sel], d["odom_t"][sel]
    elif "robot_state" in d:
        sel = (d["t"] >= t0) & (d["t"] <= t1)
        t, rs = d["t"][sel], d["robot_state"][sel]
        if len(t) < 2 or not np.isfinite(rs[:, 30:34]).all():
            return None
        dt = np.diff(t, prepend=t[0])
        # robot_state velocity is in the walk's starting heading frame, as in S-M-ICRL
        return t, np.cumsum(rs[:, 30] * dt), np.cumsum(rs[:, 31] * dt), rs[:, 33], rs[:, 26]
    else:
        return None
    if len(t) < 2:
        return None
    return t, p[:, 0], p[:, 1], p[:, 2], yaw_of(p[:, 3], p[:, 4], p[:, 5], p[:, 6])


# --------------------------------------------------------------------------- metrics


def r(x, n=3):
    return None if x is None or not np.isfinite(x) else round(float(x), n)


def group_median(values):
    values = np.asarray(values, dtype=float)
    out = {}
    for g, idx in GROUPS.items():
        v = values[idx]
        v = v[np.isfinite(v)]
        out[g] = r(np.median(v)) if v.size else None
    return out


def headline(d, meta):
    rows = walk_window(d)
    if rows.size < 2:
        return None
    t0, t1 = d["t"][rows[0]], d["t"][rows[-1]]
    out = {"duration_s": r(t1 - t0, 2)}
    track = pose_track(d, t0, t1)
    out["pose_source"] = ("sim" if "base_pose" in d else "odom" if len(d.get("odom_pose", [])) else
                          "estimate" if track else None)
    if track:
        t, x, y, z, yaw = track
        c, s = math.cos(yaw[0]), math.sin(yaw[0])
        dx, dy = x[-1] - x[0], y[-1] - y[0]
        out.update(x=r(c * dx + s * dy), y=r(-s * dx + c * dy), vx=r((c * dx + s * dy) / (t1 - t0)),
                   yaw=r(wrap(yaw[-1] - yaw[0])), minh=r(z.min()))
    if "base_pose" in d:
        tilt = tilt_of(*d["base_pose"][rows, 3:7].T)
    else:
        tilt = np.abs(d["rpy"][rows, :2]).max(axis=1) if "rpy" in d else np.zeros(1)
    outcome = str(meta.get("outcome", ""))
    out["fell"] = bool(tilt.max() > 1.0 or "tilt" in outcome)
    out["max_tilt"] = r(tilt.max())
    tau = np.abs(d["tau_est"][rows])
    out["sat_pct"] = r(100.0 * np.mean(tau >= SAT_SHARE * TORQUE_LIMIT), 2)
    out["sat_pct_by_group"] = {g: r(100.0 * np.mean(tau[:, i] >= SAT_SHARE * TORQUE_LIMIT[i]), 2)
                               for g, i in GROUPS.items()}
    if "call_target" in d and len(d["call_target"]) > 1:
        steps = np.abs(np.diff(d["call_target"], axis=0)).max(axis=1)
        out["sw_per_call"] = r(np.mean(steps > SWITCH))
    return out


def reach(d):
    """Per policy call: share of each commanded joint change done by the next call."""
    if "call_state" not in d or len(d["call_state"]) < 3:
        return None
    q = d["call_state"][:, :12]
    want = d["call_target"][:-1] - q[:-1]
    got = q[1:] - q[:-1]
    big = np.abs(want) > SWITCH
    share = np.where(big, got / np.where(big, want, 1.0), np.nan)
    med = np.array([np.nanmedian(share[:, j]) if big[:, j].any() else np.nan for j in range(12)])
    return {"median": group_median(med),
            "share_of_calls_under_half": group_median(
                [np.mean(share[big[:, j], j] < 0.5) if big[:, j].any() else np.nan for j in range(12)])}


def call_timing(d, meta):
    if "call_t" not in d or len(d["call_t"]) < 3:
        return None
    period = np.diff(d["call_t"])
    hz = meta.get("policy_hz") or 1.0 / np.median(period)
    out = {"policy_hz": r(hz, 2), "calls": int(len(d["call_t"])),
           "period_ms": {"median": r(np.median(period) * 1000, 1), "max": r(period.max() * 1000, 1)},
           "late_calls": int(np.sum(period > LATE_CALL / hz))}
    if "call_state_age" in d:
        age = d["call_state_age"] * 1000.0
        out["state_age_at_call_ms"] = {"median": r(np.median(age), 1), "max": r(age.max(), 1)}
    return out


def loop_timing(d):
    rows = walk_window(d)
    if "base_pose" in d or rows.size < 10:
        return None
    gaps = np.diff(d["t"][rows]) * 1000.0
    return {"cycle_ms": {"median": r(np.median(gaps), 2), "p99": r(np.percentile(gaps, 99), 2),
                         "max": r(gaps.max(), 1)},
            "share_over_4ms": r(np.mean(gaps > 4.0), 4)}


def cap_clipping(d):
    rows = walk_window(d)
    if "base_pose" in d or "q_cmd" not in d:
        return None
    moved = np.abs(d["q_cmd"][rows] - d["target"][rows]) > 1e-6
    return {g: r(100.0 * np.mean(moved[:, i]), 2) for g, i in GROUPS.items()}


def tracking(d):
    rows = walk_window(d)
    if rows.size < 50:
        return None
    err = d["target"][rows] - d["q"][rows]
    return {"rms_error_rad": group_median(np.sqrt(np.mean(err ** 2, axis=0))),
            "peak_error_rad": group_median(np.abs(err).max(axis=0)),
            "joint_range_rad": group_median(np.ptp(d["q"][rows], axis=0)),
            "target_range_rad": group_median(np.ptp(d["target"][rows], axis=0))}


def contacts(d):
    """Duty factor per foot from foot_force, which only the robot reports."""
    rows = walk_window(d)
    if "foot_force" not in d or rows.size < 50:
        return None
    f = d["foot_force"][rows].astype(float)
    lo, hi = np.percentile(f, 10, axis=0), np.percentile(f, 90, axis=0)
    if np.all(hi - lo < 1.0):
        return {"note": "foot_force barely changed; no contact pattern to read"}
    down = f > (lo + hi) / 2.0
    duty = down.mean(axis=0)
    same = lambda a, b: r(np.mean(down[:, a] == down[:, b]))  # noqa: E731
    return {"duty": dict(zip(LEGS, [r(v) for v in duty])),
            "diagonal_sync": {"FR-RL": same(0, 3), "FL-RR": same(1, 2)},
            "lateral_sync": {"FR-FL": same(0, 1), "RR-RL": same(2, 3)}}


def input_shift(robot, ref, estimated=False):
    """Policy inputs the robot fed that the reference never saw.

    Velocity and height (30:34) are compared only when they came from the estimator.
    """
    if "call_state" not in robot or "call_state" not in ref:
        return None
    a, b = robot["call_state"], ref["call_state"]
    out = []
    for i, name in enumerate(STATE_NAMES):
        if UNMEASURED.start <= i < UNMEASURED.stop and not estimated:
            continue
        ra, rb = a[:, i], b[:, i]
        ra, rb = ra[np.isfinite(ra)], rb[np.isfinite(rb)]
        if not ra.size or not rb.size:
            continue
        sd = max(float(np.std(rb)), 1e-3)
        shift = abs(ra.mean() - rb.mean()) / sd
        outside = float(np.mean((ra < rb.min()) | (ra > rb.max())))
        diff = abs(ra.mean() - rb.mean())
        if diff > INPUT_MIN_DIFF and (shift > INPUT_SHIFT or (outside > 0.25 and shift > 0.5)):
            out.append({"input": name, "index": i, "robot_mean": r(ra.mean()), "ref_mean": r(rb.mean()),
                        "ref_std": r(sd), "shift_sd": r(shift, 1), "share_outside_ref_range": r(outside, 2)})
    return sorted(out, key=lambda e: -e["shift_sd"])


def watch_metrics(w, clock_start, t0, t1):
    """From go2/watch.py: latency during the walk, and topic timing."""
    sys.path.insert(0, str(HERE))
    from watch import fit_delay

    out = {}
    st = w.get("state_t")
    if st is not None and len(st) > 2:
        sel = (st >= clock_start + t0) & (st <= clock_start + t1)
        gaps = np.diff(st[sel]) * 1000.0 if sel.sum() > 2 else np.diff(st) * 1000.0
        out["state_gap_ms"] = {"median": r(np.median(gaps), 2), "max": r(gaps.max(), 1)}
    ct = w.get("cmd_t")
    if ct is not None and len(ct) > 2:
        sel = (ct >= clock_start + t0) & (ct <= clock_start + t1)
        gaps = np.diff(ct[sel]) * 1000.0 if sel.sum() > 2 else np.diff(ct) * 1000.0
        out["cmd_gap_ms"] = {"median": r(np.median(gaps), 2), "max": r(gaps.max(), 1)}
        ssel = (st >= clock_start + t0) & (st <= clock_start + t1)
        csel = (ct >= clock_start + t0 - 0.1) & (ct <= clock_start + t1)
        fit = fit_delay(ct[csel], w["cmd_q"][csel], w["cmd_kp"][csel], w["cmd_kd"][csel],
                        st[ssel], w["state_q"][ssel], w["state_dq"][ssel], w["state_tau_est"][ssel])
        if fit:
            out["latency_ms"] = {g: r(v[0], 1) for g, v in fit.items()}
            out["torque_scale"] = {g: r(v[1]) for g, v in fit.items()}
            out["fit_r2"] = {g: r(v[2]) for g, v in fit.items()}
    if "sport_velocity" in w and len(w["sport_velocity"]):
        sel = (w["sport_t"] >= clock_start + t0) & (w["sport_t"] <= clock_start + t1)
        if sel.any():
            out["sport_velocity_mean"] = [r(v) for v in w["sport_velocity"][sel].mean(axis=0)]
    out["topics"] = {k: int(len(w[f"{k}_t"])) if f"{k}_t" in w else 0 for k in ("state", "cmd", "sport", "odom")}
    return out


def frequencies(d, meta, w=None, t0=None, t1=None):
    """The rates and gains the run used, set and measured: control loop, policy, env vector,
    robot state and commands (from watch.py), kp, kd and torque fraction."""
    out = {}
    rows = walk_window(d)
    if rows.size > 10:
        t = d["t"][rows]
        out["control_loop_hz"] = r(1.0 / np.median(np.diff(t)), 1)
    out["policy_hz_set"] = meta.get("policy_hz")
    if "call_t" in d and len(d["call_t"]) > 2:
        out["policy_hz_measured"] = r(1.0 / np.median(np.diff(d["call_t"])), 2)
    out["env_hz_set"] = meta.get("env_hz")
    sim = meta.get("source") == "smicrl"
    # Live: the probes follow the body height (S-M-ICRL always; the robot with the estimate).
    out["env_live"] = True if sim else bool(meta.get("estimate"))
    out["kp"], out["kd"] = meta.get("kp"), meta.get("kd")
    out["torque_fraction"] = 1.0 if sim else meta.get("torque_fraction")  # S-M-ICRL caps only at the limits
    if w is not None and t0 is not None:
        for name, key in (("robot_state_hz", "state_t"), ("commands_hz", "cmd_t")):
            if key in w and len(w[key]) > 2:
                tt = w[key]
                sel = (tt >= t0) & (tt <= t1)
                tt = tt[sel] if sel.sum() > 2 else tt
                out[name] = r(len(tt) / max(tt[-1] - tt[0], 1e-9), 1)
    return {k: v for k, v in out.items() if v is not None}


def diagnose(d, meta, w=None):
    out = {"headline": headline(d, meta), "calls": call_timing(d, meta), "reach": reach(d),
           "loop": loop_timing(d), "cap_clipped_pct": cap_clipping(d), "tracking": tracking(d),
           "contacts": contacts(d)}
    window = None
    if w is not None and "clock_start" in meta:
        rows = walk_window(d)
        out["watch"] = watch_metrics(w, meta["clock_start"], d["t"][rows[0]], d["t"][rows[-1]])
        window = (meta["clock_start"] + d["t"][rows[0]], meta["clock_start"] + d["t"][rows[-1]])
    out["frequencies"] = frequencies(d, meta, w, *(window or (None, None)))
    return {k: v for k, v in out.items() if v is not None}


# --------------------------------------------------------------------------- causes


def get(res, *keys):
    for k in keys:
        if not isinstance(res, dict) or k not in res:
            return None
        res = res[k]
    return res


def causes(robot, ref, meta, ref_meta, shift, latency_warn):
    """Plain-language findings, most likely first."""
    found = []
    outcome = str(meta.get("outcome", ""))
    if outcome and outcome != "finished":
        found.append(f"The run didn't finish: {outcome}.")
    lat = get(robot, "watch", "latency_ms")
    if lat:
        worst = max(v for v in lat.values() if v is not None)
        if worst > latency_warn:
            found.append(f"Command-to-torque latency during the walk was {worst:.0f} ms. The sim sweep "
                         f"degrades past about {latency_warn:g} ms. Put this latency in S-M-ICRL's training.")
    hz, ref_hz = meta.get("policy_hz"), ref_meta.get("policy_hz")
    if hz and ref_hz and abs(hz - ref_hz) > 1e-6:
        found.append(f"The robot called the policy at {hz:g} Hz and the reference at {ref_hz:g} Hz. "
                     f"This gait depends on the rate: rerun with --policy-hz {ref_hz:g}.")
    f, ref_f = robot.get("frequencies") or {}, ref.get("frequencies") or {}
    if f.get("env_hz_set") and ref_f.get("env_hz_set") and abs(f["env_hz_set"] - ref_f["env_hz_set"]) > 1e-6:
        found.append(f"The env vector was rebuilt at {f['env_hz_set']:g} Hz on the robot and {ref_f['env_hz_set']:g} Hz "
                     f"in the reference (--env-hz); this matters if the policy has gains on the probe columns.")
    if f.get("control_loop_hz") and f["control_loop_hz"] < 450:
        found.append(f"The control loop ran at {f['control_loop_hz']:g} Hz, below its 500 Hz.")
    measured = f.get("policy_hz_measured")
    if measured and hz and abs(measured - hz) / hz > 0.05:
        found.append(f"Policy calls came at {measured:g} Hz against the {hz:g} Hz asked for.")
    for name, value in (("robot_state_hz", f.get("robot_state_hz")), ("commands_hz", f.get("commands_hz"))):
        if value and value < 450:
            found.append(f"{'Robot state' if name == 'robot_state_hz' else 'Commands'} arrived at {value:g} Hz "
                         f"during the walk (go2/watch.py), below 500 Hz.")
    for key, label in (("kp", "kp"), ("kd", "kd")):
        if f.get(key) is not None and ref_f.get(key) is not None and abs(f[key] - ref_f[key]) > 1e-6:
            found.append(f"The robot used {label} {f[key]:g} and the reference {ref_f[key]:g}.")
    if f.get("torque_fraction") is not None and f["torque_fraction"] < 1.0:
        found.append(f"Torque was capped at {f['torque_fraction']:.0%} of the limits (--torque-fraction); "
                     f"S-M-ICRL caps only at the limits.")
    late = get(robot, "calls", "late_calls")
    if late:
        found.append(f"{late} policy calls came late (over {LATE_CALL}x the period). The policy's "
                     f"timing differed from training.")
    age = get(robot, "calls", "state_age_at_call_ms", "max")
    if age and age > 10:
        found.append(f"The policy saw state up to {age:.0f} ms old.")
    sw, ref_sw = get(robot, "headline", "sw_per_call"), get(ref, "headline", "sw_per_call")
    if sw is not None and ref_sw is not None and ref_sw - sw > SW_DROP:
        found.append(f"Targets switched on {sw:.0%} of calls against {ref_sw:.0%} in the reference. "
                     f"The joints didn't reach the policy's switching points in time.")
    for g in GROUPS:
        a, b = get(robot, "reach", "median", g), get(ref, "reach", "median", g)
        if a is not None and b is not None and b - a > REACH_DROP:
            found.append(f"{g.capitalize()}s covered {a:.0%} of each commanded move per call, against "
                         f"{b:.0%} in the reference: the joints are slower on the robot (latency, "
                         f"torque limit, friction).")
    clip = get(robot, "cap_clipped_pct")
    if clip:
        g = max(clip, key=lambda k: clip[k] or 0)
        if (clip[g] or 0) > 5:
            found.append(f"run_policy.py's torque cap moved the {g} targets on {clip[g]:.0f}% of cycles "
                         f"(--torque-fraction {meta.get('torque_fraction')}); S-M-ICRL allows the full limit.")
    sat, ref_sat = get(robot, "headline", "sat_pct"), get(ref, "headline", "sat_pct")
    if sat is not None and ref_sat is not None and sat - ref_sat > SAT_RISE_PCT:
        found.append(f"Torque saturated on {sat:.1f}% of samples against {ref_sat:.1f}% in the reference.")
    scale = get(robot, "watch", "torque_scale")
    if scale:
        off = {g: v for g, v in scale.items() if v is not None and abs(v - 1.0) > 0.15}
        if off:
            found.append(f"tau_est doesn't match the PD law for {', '.join(off)} "
                         f"(scale {', '.join(f'{v:.2f}' for v in off.values())}): the motors give less "
                         f"torque than commanded, or tau_est is off.")
    for e in (shift or [])[:5]:
        found.append(f"Policy input {e['input']} averaged {e['robot_mean']} on the robot against "
                     f"{e['ref_mean']} ± {e['ref_std']} in the reference "
                     f"({e['share_outside_ref_range']:.0%} of calls outside anything the reference saw).")
    if meta.get("fill_height") is not None:
        found.append("The policy reads body velocity and height, which were fed as constants "
                     "(--fill-unmeasured), so any correction it makes with them is missing.")
    x, ref_x = get(robot, "headline", "x"), get(ref, "headline", "x")
    dur, ref_dur = get(robot, "headline", "duration_s"), get(ref, "headline", "duration_s")
    if meta.get("inputs_30_33") == "estimator":
        found.append("Body velocity and height came from go2/state_estimator.py. A foot that slips makes it "
                     "read slow; `diagnose.py inputs` on a sport-mode recording shows how far it is from "
                     "Unitree's own estimate.")
    if x is None:
        found.append("No rt/utlidar/robot_odom during the walk, and no estimate to integrate, so distance "
                     "and heading are unknown.")
    elif get(robot, "headline", "pose_source") == "estimate":
        found.append("Distance and heading come from integrating the estimated velocity "
                     "(no rt/utlidar/robot_odom), so they read short if the feet slipped.")
    elif ref_x and dur and ref_dur and abs(dur - ref_dur) / ref_dur > 0.1:
        found.append(f"The robot walked {dur:g} s and the reference {ref_dur:g} s; compare vx, not x.")
    yaw = get(robot, "headline", "yaw")
    if yaw is not None and abs(yaw) > 0.3:
        found.append(f"The heading drifted {yaw:+.2f} rad.")
    sync = get(robot, "contacts", "diagonal_sync")
    if sync and min(v for v in sync.values() if v is not None) < 0.6:
        found.append(f"Diagonal feet were in the same contact state only {min(sync.values()):.0%} of the "
                     f"time: the trot pairs weren't moving together.")
    return found


# --------------------------------------------------------------------------- report


COLUMNS = (("x", "x(m)", "{:6.2f}"), ("y", "y(m)", "{:6.2f}"), ("vx", "vx", "{:5.2f}"),
           ("yaw", "yaw", "{:6.2f}"), ("minh", "minh", "{:5.2f}"), ("fell", "fell", "{!s:>5}"),
           ("sat_pct", "sat%", "{:5.1f}"), ("sw_per_call", "sw/call", "{:7.2f}"))


def row(name, h):
    cells = []
    for key, label, fmt in COLUMNS:
        v = (h or {}).get(key)
        cells.append(fmt.format(v) if v is not None else f"{'-':>{len(label) + 1}}")
    return f"{name:<24}" + " ".join(cells)


def side_by_side(title, a, b):
    if a is None and b is None:
        return
    print(f"\n{title}")
    keys = sorted(set((a or {}).keys()) | set((b or {}).keys()))
    for k in keys:
        print(f"  {k:<28} robot {json.dumps((a or {}).get(k))}   ref {json.dumps((b or {}).get(k))}")


def report(args):
    d, meta = load(args.run)
    if "call_t" not in d:
        print("Note: this log has no per-call records (run_policy.py from before they were added); "
              "sw/call, reach and input checks need a new run.")
    w, w_meta = load(args.watch) if args.watch else (None, {})
    ref, ref_meta = load(args.reference) if args.reference else (None, {})
    robot = diagnose(d, meta, w)
    reference_ = diagnose(ref, ref_meta) if ref is not None else {}
    shift = input_shift(d, ref, meta.get("inputs_30_33") == "estimator") if ref is not None else None

    print(f"{'':24}" + " ".join(f"{label:>{len(label) + 1}}" for _, label, _ in COLUMNS))
    if ref is not None:
        print(row(f"reference ({ref_meta.get('source', 'sim')})", reference_.get("headline")))
    print(row("robot" if not meta.get("interface") in (None, "lo") else "this run", robot.get("headline")))
    if ref is not None and ref_meta.get("reward") is not None:
        print(f"(reference reward {ref_meta['reward']:.3f})")

    side_by_side("Frequencies and gains", robot.get("frequencies"), reference_.get("frequencies"))
    side_by_side("Reach: share of each commanded move done by the next call", get(robot, "reach", "median"),
                 get(reference_, "reach", "median"))
    side_by_side("Torque saturation by joint (%)", get(robot, "headline", "sat_pct_by_group"),
                 get(reference_, "headline", "sat_pct_by_group"))
    side_by_side("Tracking", robot.get("tracking"), reference_.get("tracking"))
    side_by_side("Policy calls", robot.get("calls"), reference_.get("calls"))
    if robot.get("watch"):
        side_by_side("Measured during the walk (go2/watch.py)", robot["watch"], None)
    if robot.get("loop"):
        side_by_side("run_policy.py control loop", robot["loop"], None)
    if w_meta.get("usage"):
        robot["usage"] = w_meta["usage"]
        print("\nUsage (go2/watch.py)")
        for key, value in w_meta["usage"].items():
            print(f"  {key:<24} {json.dumps(value)}")
    if robot.get("contacts"):
        side_by_side("Foot contacts", robot["contacts"], None)
    if shift:
        print("\nPolicy inputs outside the reference's range")
        for e in shift:
            print(f"  {e['input']:<14} robot {e['robot_mean']:+.3f}  ref {e['ref_mean']:+.3f} ± {e['ref_std']:.3f}"
                  f"  ({e['shift_sd']:.1f} sd, {e['share_outside_ref_range']:.0%} outside)")

    found = causes(robot, reference_, meta, ref_meta, shift, args.latency_warn)
    print("\nLikely causes" if found else "\nNothing stands out.")
    for i, f in enumerate(found, 1):
        print(f"  {i}. {f}")
    out = args.run.with_suffix(".diagnosis.json")
    out.write_text(json.dumps({"run": str(args.run), "watch": args.watch and str(args.watch),
                               "reference": args.reference and str(args.reference), "robot": robot,
                               "reference_results": reference_, "input_shift": shift, "causes": found},
                              indent=1, default=str))
    print(f"\nSaved: {out}")
    return 0


# --------------------------------------------------------------------------- inputs


def _fit(x, y):
    """slope, offset, r2 and rms residual of y ~ slope * x + offset."""
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if x.size < 20 or np.std(x) < 1e-6:
        return None
    A = np.column_stack([x, np.ones_like(x)])
    (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ [a, b]
    return {"slope": r(a), "offset": r(b), "r2": r(1.0 - res.var() / max(y.var(), 1e-12)),
            "rms": r(np.sqrt(np.mean(res ** 2))), "rms_raw": r(np.sqrt(np.mean((y - x) ** 2)))}


def _verdict(ok, text):
    return ("OK   " if ok else ("CHECK" if ok is not None else "n/a  ")) + " " + text


def _quat_mul(a, b):
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw], axis=1)


def check_inputs(w):
    """Check each robot_state input against an independent source, from a watch.py recording.

    Returns (lines, results, flags): printable verdicts, the numbers behind them, and
    {check: True (OK) / False (CHECK) / None (couldn't tell)}.
    """
    sys.path.insert(0, str(HERE))
    import state_estimator as se

    t = w["state_t"]
    # Rates are taken on the robot's own clock (tick, in ms) over +-SPAN samples: arrival
    # times carry this computer's scheduling jitter, which would swamp a 2 ms difference.
    tick = w["state_tick"].astype(float) / 1000.0 if "state_tick" in w else t
    if not (np.diff(tick) >= 0).all() or tick[-1] - tick[0] < 0.5 * (t[-1] - t[0]):
        tick = t  # tick wrapped or isn't counting
    SPAN = 5
    lo, hi = np.arange(len(t)) - SPAN, np.arange(len(t)) + SPAN
    lo, hi = np.clip(lo, 0, len(t) - 1), np.clip(hi, 0, len(t) - 1)
    span_t = tick[hi] - tick[lo]
    good = span_t > 0
    q, dq, quat = w["state_q"], w["state_dq"], w["state_quaternion"]
    gyro, accel = w["state_gyro"], w["state_accel"]
    rs = w.get("state_robot_state")
    lines, results, flags = [], {}, {}

    def verdict(key, ok, text):
        flags[key] = None if ok is None else bool(ok)
        return _verdict(ok, text)

    # 0-11, 12-23: joint angles and velocities
    limits = np.array([[-1.0472, 1.0472], [-1.5708, 3.4907], [-2.7227, -0.83776]] * 2
                      + [[-1.0472, 1.0472], [-0.5236, 4.5379], [-2.7227, -0.83776]] * 2)
    outside = float(np.mean((q < limits[:, 0] - 0.05) | (q > limits[:, 1] + 0.05)))
    results["joint_angles_outside_model_limits"] = r(outside, 4)
    lines.append(verdict("joint_angles", outside < 0.001, f"joint angles 0-11: {outside:.2%} of samples outside the model's limits"))
    dq_num = (q[hi] - q[lo]) / np.where(good, span_t, np.nan)[:, None]
    dq_avg = np.cumsum(np.vstack([np.zeros(12), dq]), axis=0)
    dq_mean = (dq_avg[hi + 1] - dq_avg[lo]) / (hi - lo + 1)[:, None]  # dq averaged over the same span
    moving = np.abs(dq).max(axis=0) > 0.5
    fits = [_fit(dq_mean[:, j], dq_num[:, j]) for j in range(12)]
    slopes = [f["slope"] for j, f in enumerate(fits) if f and moving[j]]
    results["dq_vs_dq_from_angles_slope"] = slopes
    lines.append(verdict("joint_velocities", (min(slopes) > 0.8 and max(slopes) < 1.2) if slopes else None,
                          "joint velocities 12-23 vs the rate of the joint angles: slope "
                          + (f"{min(slopes):.2f}-{max(slopes):.2f} (1 = same sign and units)" if slopes
                             else "not measured (no joint moved)")))

    # 24-26: roll, pitch, yaw from the quaternion vs the IMU's own rpy
    if "state_imu_rpy" in w:
        mine = np.array([se_rpy for se_rpy in (_rpy(*row) for row in quat)])
        diff = np.abs(np.angle(np.exp(1j * (mine - w["state_imu_rpy"]))))
        worst = diff.max(axis=0)
        results["rpy_vs_imu_rpy_max_diff"] = [r(x) for x in worst]
        lines.append(verdict("rpy", worst[:2].max() < 0.02, f"roll/pitch/yaw 24-26 vs the IMU's own rpy: max difference "
                              f"{worst[0]:.3f}, {worst[1]:.3f}, {worst[2]:.3f} rad (checks quaternion order)"))

    # 27-29: gyro vs the quaternion's rate, in the body frame
    inv = quat * np.array([1.0, -1.0, -1.0, -1.0])
    dqt = _quat_mul(inv[lo], quat[hi])
    omega = 2.0 * dqt[:, 1:] / np.where(good, span_t, np.nan)[:, None]
    g_avg = np.cumsum(np.vstack([np.zeros(3), gyro]), axis=0)
    g = (g_avg[hi + 1] - g_avg[lo]) / (hi - lo + 1)[:, None]
    gfits = [_fit(g[:, k], omega[:, k]) if np.std(g[:, k]) > 0.05 else None for k in range(3)]
    results["gyro_vs_quaternion_rate"] = gfits
    gs = [f["slope"] for f in gfits if f]
    lines.append(verdict("gyro", (min(gs) > 0.8 and max(gs) < 1.2) if gs else None,
                          "angular velocity 27-29 vs the quaternion's rate: slope x,y,z "
                          + ", ".join("-" if f is None else f"{f['slope']:.2f}" for f in gfits)
                          + " (1 = body frame, as S-M-ICRL's qvel[3:6])"))

    # 34-36: up vector vs gravity in the accelerometer, when nearly still
    up = np.stack([2 * (quat[:, 1] * quat[:, 3] - quat[:, 0] * quat[:, 2]),
                   2 * (quat[:, 2] * quat[:, 3] + quat[:, 0] * quat[:, 1]),
                   1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2)], axis=1)
    norm = np.linalg.norm(accel, axis=1)
    still = (np.abs(norm - 9.81) < 0.5) & (np.linalg.norm(gyro, axis=1) < 0.2)
    if still.sum() > 50:
        cosang = np.sum(up[still] * accel[still] / norm[still, None], axis=1)
        angle = float(np.degrees(np.arccos(np.clip(np.median(cosang), -1, 1))))
        results["up_vs_accel_deg"] = r(angle, 2)
        lines.append(verdict("gravity", angle < 3.0, f"gravity direction 34-36 vs the accelerometer at rest: {angle:.1f}° apart"))
    else:
        lines.append(verdict("gravity", None, "gravity direction 34-36: the robot was never still"))

    # Leg kinematics vs Unitree's foot positions and speeds (sport mode only)
    sport = "sport_t" in w and len(w["sport_t"]) > 20
    if sport and "sport_foot_position_body" in w:
        st = w["sport_t"]
        qi = np.stack([np.interp(st, t, q[:, j]) for j in range(12)], axis=1)
        feet = np.array([se.foot_positions(row) for row in qi]).reshape(len(st), 12)
        theirs = w["sport_foot_position_body"]
        offset = np.median(theirs - feet, axis=0)
        rms = float(np.sqrt(np.mean((theirs - feet - offset) ** 2)))
        corr = [np.corrcoef(feet[:, k], theirs[:, k])[0, 1] if np.std(feet[:, k]) > 1e-3 else np.nan for k in range(12)]
        results["foot_position_vs_sport"] = {"offset_m": [r(x) for x in offset], "rms_after_offset_m": r(rms),
                                             "min_correlation": r(np.nanmin(corr)) if np.isfinite(corr).any() else None}
        lines.append(verdict("kinematics", rms < 0.01, f"leg kinematics vs Unitree's foot_position_body: {rms * 1000:.1f} mm rms "
                              f"after a constant offset of up to {np.abs(offset).max() * 1000:.0f} mm "
                              "(checks joint signs and the leg model)"))
    else:
        lines.append(verdict("kinematics", None, "leg kinematics: no rt/sportmodestate, so nothing to compare with"))

    # 30-32, 33: the estimator vs sport mode's and robot_odom's
    if rs is not None:
        est_v_world, est_h = w["state_est_velocity"], w["state_est_height"]
        yaw = np.array([_rpy(*row)[2] for row in quat])
        c, s_ = np.cos(yaw), np.sin(yaw)
        est_v_body = np.stack([c * est_v_world[:, 0] + s_ * est_v_world[:, 1],
                               -s_ * est_v_world[:, 0] + c * est_v_world[:, 1]], axis=1)
        for name, src_t, src_v in (("sport velocity", "sport_t", "sport_velocity"),
                                   ("robot_odom velocity", "odom_t", "odom_velocity")):
            if src_t not in w or len(w[src_t]) < 20:
                lines.append(verdict(f"velocity_vs_{name.split()[0]}", None, f"velocity 30-32 vs {name}: not published"))
                continue
            tt, vv = w[src_t], w[src_v]
            best = None
            for frame, ev in (("world", est_v_world[:, :2]), ("body", est_v_body)):
                e = np.stack([np.interp(tt, t, ev[:, k]) for k in range(2)], axis=1)
                f = [_fit(e[:, k], vv[:, k]) for k in range(2)]
                score = np.mean([x["r2"] for x in f if x]) if any(f) else -1
                if best is None or score > best[0]:
                    best = (score, frame, f)
            score, frame, f = best
            results[f"velocity_vs_{name.split()[0]}"] = {"frame": frame, "x": f[0], "y": f[1]}
            if f[0] is None:
                lines.append(verdict(f"velocity_vs_{name.split()[0]}", None, f"velocity 30-32 vs {name}: the robot didn't move"))
                continue
            lines.append(verdict(f"velocity_vs_{name.split()[0]}", f[0]["r2"] > 0.7 and 0.8 < f[0]["slope"] < 1.25,
                                  f"velocity 30-32 vs {name} ({frame} frame): forward slope {f[0]['slope']:.2f}, "
                                  f"r2 {f[0]['r2']:.2f}, rms {f[0]['rms_raw']:.3f} m/s"
                                  + (f"; sideways slope {f[1]['slope']:.2f}" if f[1] else "")
                                  + " (slope < 1: the estimate reads slow)"))
        for name, src_t, src in (("sport body_height", "sport_t", "sport_body_height"),
                                 ("robot_odom z", "odom_t", None)):
            if src_t not in w or len(w[src_t]) < 20:
                continue
            tt = w[src_t]
            theirs = w[src] if src else w["odom_pose"][:, 2]
            mine = np.interp(tt, t, est_h)
            off = float(np.median(theirs - mine))
            spread = float(np.sqrt(np.mean((theirs - mine - off) ** 2)))
            results[f"height_vs_{name.split()[0]}"] = {"offset_m": r(off), "rms_after_offset_m": r(spread),
                                                        "estimate_mean_m": r(mine.mean())}
            lines.append(verdict(f"height_vs_{name.split()[0]}", spread < 0.02, f"height 33 vs {name}: estimate averages {mine.mean():.3f} m, "
                                  f"{spread * 1000:.0f} mm rms apart after a {off:+.3f} m offset"))
        if "state_est_contact_from_force" in w:
            share = float(np.mean(w["state_est_contact_from_force"]))
            results["contact_from_force_share"] = r(share, 3)
            lines.append(verdict("contact_force", share > 0.95, f"standing feet picked from foot_force {share:.0%} of the time "
                                  "(otherwise from foot height, which is less reliable)"))
    return lines, results, flags


# Inputs that must mean the same as in S-M-ICRL before any policy runs.
CORE_INPUT_CHECKS = ("joint_angles", "joint_velocities", "rpy", "gyro", "gravity")


def inputs(args):
    w, _ = load(args.watch)
    if "state_q" not in w:
        sys.exit("this isn't a go2/watch.py recording (no state_q)")
    lines, results, flags = check_inputs(w)
    t = w["state_t"]
    sport = "sport_t" in w and len(w["sport_t"]) > 20
    print(f"Input check: {args.watch}  ({len(t)} states over {t[-1] - t[0]:.0f} s"
          f"{', sport mode publishing' if sport else ', no sport mode'})\n")
    for line in lines:
        print("  " + line)
    out = args.watch.with_suffix(".inputs.json")
    out.write_text(json.dumps({"results": results, "flags": flags}, indent=1, default=str))
    print(f"\nSaved: {out}")
    return 0


def _rpy(w_, x, y, z):
    return np.array([np.arctan2(2 * (w_ * x + y * z), 1 - 2 * (x * x + y * y)),
                     np.arcsin(np.clip(2 * (w_ * y - z * x), -1, 1)),
                     np.arctan2(2 * (w_ * z + x * y), 1 - 2 * (y * y + z * z))])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("reference", help="run the policy through S-M-ICRL's trial code and record it")
    p.add_argument("policy", type=Path)
    p.add_argument("--smicrl", type=Path, default=HERE.parents[1] / "S-MICRL", help="S-M-ICRL checkout (default: %(default)s)")
    p.add_argument("--config", required=True, help="task config, relative to --smicrl, e.g. func_config/go2/walk/walk.yaml")
    p.add_argument("--out", type=Path, help="default: go2/logs/reference/<policy>.npz")
    p = sub.add_parser("report", help="diagnose a run_policy.py log")
    p.add_argument("run", type=Path, help="go2/run_policy.py log (.npz)")
    p.add_argument("--watch", type=Path, help="go2/watch.py recording of the same run")
    p.add_argument("--reference", type=Path, help="diagnose.py reference output for the same policy")
    p.add_argument("--latency-warn", type=float, default=LATENCY_WARN_MS,
                   help="latency in ms past which the sim gait degrades (default: %(default)s)")
    p = sub.add_parser("inputs", help="check each robot_state input against a second source, from a watch.py recording")
    p.add_argument("watch", type=Path, help="go2/watch.py recording, ideally of the robot walking in sport mode")
    return parser.parse_args()


def main():
    args = parse_args()
    return {"reference": reference, "report": report, "inputs": inputs}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
