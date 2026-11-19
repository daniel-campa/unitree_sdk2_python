#!/usr/bin/env python
"""Measure the Go2's motors, timing and sensors, so a simulator can be matched to the robot.

The same tests run on three backends, and `compare` lines up the results:

  - the real robot (--robot), through the SDK
  - go2/sim_headless.py, Unitree's MuJoCo model, through the same DDS topics
  - --smicrl XML: S-M-ICRL's training physics, stepped directly here. It uses
    S-M-ICRL's PD law, which adds gravity compensation (qfrc_bias) to each joint
    and has no delay. The robot's motors do neither, so this backend is the one
    to match the robot against.

Tests:

  passive  READ-ONLY. Records rt/lowstate: state rate and jitter, sensor noise, IMU
           bias. Creates no publisher or SDK client, so it can't move the robot.
  hang     MOVES THE LEGS. The robot must hang in a harness with all four feet clear.
           Steps, sine sweeps and slow ramps on each joint type, about 2 minutes:
           command-to-response latency, torque scale, tracking gain and phase,
           Coulomb and viscous friction.
  stand    MOVES THE ROBOT. On the floor: stands at --kp/--kd and holds, then lies
           down. Joint sag, holding torque and foot force at the policy's gains.
  analyze  Re-analyze a saved .npz: a sysid log, or a go2/run_policy.py log, for
           torque usage, saturation, tracking and latency during a real walk.
  compare  Print two or more result .json files side by side, with suggestions.

    python go2/sysid.py passive --robot                       # real robot, read-only
    python go2/sysid.py hang --robot                          # real robot, in a harness
    python go2/sysid.py stand --robot                         # real robot, on the floor
    python go2/sysid.py hang                                  # sim_headless.py --flat --dt 0.002 --hang
    python go2/sysid.py stand                                 # sim_headless.py --flat --dt 0.002
    python go2/sysid.py hang --smicrl ../S-MICRL/Robots/go2/scene.xml    # S-M-ICRL physics
    python go2/sysid.py analyze go2/logs/2026-11-17_15-02-11.npz        # a run_policy.py walk
    python go2/sysid.py compare REAL.json SMICRL.json [SIM.json]

Each run saves go2/logs/sysid/<time>_<test>_<backend>.npz and a .json of results next to it.

Exit status: 0 = finished, 1 = stopped early, 2 = refused before commanding anything.
"""
import argparse
import json
import math
import select
import signal
import sys
import termios
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Callable, NamedTuple

import numpy as np
import run_policy as rp
from run_policy import (
    DEFAULT_INTERFACE,
    HOME,
    JOINT_HIGH,
    JOINT_LOW,
    JOINTS,
    KD,
    KP,
    LIE_DOWN,
    LIE_DOWN_S,
    MIN_BATTERY_PERCENT,
    SETTLE_S,
    STAND_UP_S,
    TORQUE_LIMIT,
    Fault,
    Refused,
    bounded,
    read_state,
    safety_problem,
)

LOG_DIR = Path(__file__).resolve().parent / "logs" / "sysid"
DEFAULT_SMICRL_XML = Path(__file__).resolve().parents[2] / "S-MICRL" / "Robots" / "go2" / "scene.xml"  # walk.yaml's xml_path

GROUPS = {"hip": [0, 3, 6, 9], "thigh": [1, 4, 7, 10], "calf": [2, 5, 8, 11]}
# Which way each leg's joint moves for a positive test offset. The hips are mirrored,
# like LIE_DOWN, so both sides swing out together and the body stays balanced.
DIRECTION = {"hip": [1, -1, 1, -1], "thigh": [1, 1, 1, 1], "calf": [1, 1, 1, 1]}

# Hang test sizes, in rad. Kept small: at kp 80 a 0.1 rad step asks for 8 N·m, well
# under the thigh's 19 N·m at the 0.8 torque fraction.
STEP_SIZE = 0.1
STEP_PATTERN = (0.0, 1.0, 0.0, -1.0)  # times STEP_SIZE, each held STEP_HOLD_S
STEP_HOLD_S = 1.0
STEP_REPEATS = 3
SINE_SIZE = 0.1
SINE_HZ = (0.5, 1.0, 2.0, 4.0)  # the policies change targets at 5-10 Hz
RAMP_SIZE = {"hip": 0.2, "thigh": 0.25, "calf": 0.25}
RAMP_SPEEDS = (0.2, 0.6)  # rad/s; two speeds separate Coulomb from viscous friction
REST_S = 0.5
WAIT_STAGE = "waiting"
WAIT_TIMEOUT_S = 60.0
PASSIVE_S = 10.0
STAND_HOLD_S = 10.0

# Analysis.
MAX_DELAY_S = 0.08  # longest delay the torque fit searches
STEP_DETECT = 0.5  # share of the expected torque jump that counts as a response
STEP_MOVE = 0.1  # share of a step that counts as the joint moving
SPEED_BINS = (0.0, 2.0, 5.0, 10.0, 20.0, np.inf)  # rad/s, for the torque-speed envelope


class Phase(NamedTuple):
    """One stretch of a test: target_at(elapsed, q_at_phase_start) -> 12 joint targets."""
    stage: str
    seconds: float
    target_at: Callable[[float, np.ndarray], np.ndarray]
    check_range: bool = True


def ramp_to(stage, goal, seconds, check_range=True):
    """From wherever the joints are, so the first command causes no torque step."""
    return Phase(stage, seconds, lambda e, q0: q0 + (goal - q0) * min(e / seconds, 1.0), check_range)


def hold(stage, goal, seconds):
    return Phase(stage, seconds, lambda e, q0: goal)


def offset(group, value):
    out = np.zeros(12)
    out[GROUPS[group]] = np.asarray(DIRECTION[group]) * value
    return out


def step_phase(group, scale):
    period = STEP_HOLD_S * len(STEP_PATTERN)

    def target_at(e, q0):
        level = STEP_PATTERN[int((e % period) // STEP_HOLD_S)]
        return HOME + offset(group, level * STEP_SIZE * scale)

    return Phase(f"step:{group}", period * STEP_REPEATS + STEP_HOLD_S, target_at)


def sine_phase(group, hz, scale):
    cycles = max(3, math.ceil(2.0 * hz))  # whole cycles, so it ends back at HOME

    def target_at(e, q0):
        return HOME + offset(group, SINE_SIZE * scale * math.sin(2.0 * math.pi * hz * e))

    return Phase(f"sine:{group}:{hz:g}", cycles / hz, target_at)


def triangle_phase(group, speed, scale):
    """0 -> +size -> -size -> 0 at a constant speed, twice."""
    size = RAMP_SIZE[group] * scale
    quarter = size / speed

    def target_at(e, q0):
        x = (e / quarter) % 4.0
        level = x if x < 1.0 else (2.0 - x if x < 3.0 else x - 4.0)
        return HOME + offset(group, level * size)

    return Phase(f"ramp:{group}:{speed:g}", 8.0 * quarter, target_at)


def hang_phases(groups, scale):
    phases = [ramp_to("standing up", HOME, STAND_UP_S, check_range=False),
              hold("settling", HOME, SETTLE_S), hold(WAIT_STAGE, HOME, WAIT_TIMEOUT_S)]
    for group in groups:
        phases.append(step_phase(group, scale))
        phases.extend(sine_phase(group, hz, scale) for hz in SINE_HZ)
        phases.append(hold(f"rest:{group}", HOME, REST_S))
        phases.extend(triangle_phase(group, speed, scale) for speed in RAMP_SPEEDS)
        phases.append(hold(f"rest:{group}", HOME, REST_S))
    phases.append(ramp_to("lying down", LIE_DOWN, LIE_DOWN_S))
    return phases


def stand_phases(seconds):
    return [ramp_to("standing up", HOME, STAND_UP_S, check_range=False),
            hold("settling", HOME, SETTLE_S), hold(WAIT_STAGE, HOME, WAIT_TIMEOUT_S),
            hold("stand:hold", HOME, seconds), ramp_to("lying down", LIE_DOWN, LIE_DOWN_S)]


def phases_seconds(phases):
    return sum(p.seconds for p in phases if p.stage != WAIT_STAGE)


# --------------------------------------------------------------------------- DDS backends


class ArrivalMonitor(rp.StateMonitor):
    """run_policy's StateMonitor, also keeping when every message arrived and its tick."""

    def __init__(self, keep_states=False):
        super().__init__()
        self.arrivals = []
        self.states = [] if keep_states else None

    def __call__(self, msg):
        stamp = time.perf_counter()
        self.msg, self.stamp = msg, stamp
        self.arrivals.append((stamp, msg.tick))
        if self.states is not None:
            self.states.append(read_state(msg))


def connect(robot, domain, keep_states=False):
    """Subscribe to rt/lowstate and wait for a valid message, as run_policy.connect does."""
    interface = robot or "lo"
    domain = domain if domain is not None else (0 if robot else 1)
    if robot and not rp.check_interface(robot):
        raise Refused(f"interface {robot} can't be used")
    rp.ChannelFactoryInitialize(domain, interface)
    monitor = ArrivalMonitor(keep_states)
    subscriber = rp.ChannelSubscriber("rt/lowstate", rp.LowState_)
    subscriber.Init(monitor)  # no queue: see run_policy.connect
    deadline = time.perf_counter() + 5.0
    while monitor.msg is None:
        if time.perf_counter() > deadline:
            where = "Is the robot connected?" if robot else "Is go2/sim_headless.py running?"
            raise Refused(f"no rt/lowstate on {interface}, domain {domain}, after 5 s. {where}")
        time.sleep(0.01)
    problems = rp.state_problems(monitor.msg)
    if problems:
        raise Refused("invalid robot state: " + "; ".join(problems))
    return monitor, subscriber


class SysidController(rp.Controller):
    """run_policy's Controller, also logging each state's arrival time, tick and the gains."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.extra = []
        self._stamp, self._tick = 0.0, 0

    def latest_state(self, check_range):
        msg, stamp = self.monitor.msg, self.monitor.stamp
        state = super().latest_state(check_range)
        self._stamp, self._tick = stamp, msg.tick
        return state

    def record(self, stage, state, target, q_cmd):
        super().record(stage, state, target, q_cmd)
        self.extra.append((self._stamp - self.clock_start, self._tick, self.kp, self.kd))

    def wait_for_enter(self, what):
        termios.tcflush(sys.stdin, termios.TCIFLUSH)  # an earlier Enter must not start it
        print(f"Standing. Press Enter to start {what} (Ctrl-C or B stops). "
              f"It lies down after {WAIT_TIMEOUT_S:.0f} s.", flush=True)

        def entered():
            return bool(select.select([sys.stdin], [], [], 0.0)[0])

        self.hold(WAIT_STAGE, HOME, WAIT_TIMEOUT_S, stop_when=entered)
        if not entered():
            return False
        return rp.read_enter()

    def run_phases(self, phases, ask, what):
        for i, phase in enumerate(phases):
            if phase.stage == WAIT_STAGE:
                if ask and not self.wait_for_enter(what):
                    self.run_phases(phases[-1:], ask, what)  # lie down
                    return
                continue
            q0 = self.latest_state(phase.check_range).q
            self.run_phase(phase.stage, phase.seconds,
                           lambda e, state, p=phase, q0=q0: p.target_at(e, q0), phase.check_range)

    def columns(self, monitor):
        fields = {name: np.array(column) for name, column in zip(rp.LOG_FIELDS, zip(*self.records))}
        stamp, tick, kp, kd = (np.array(c) for c in zip(*self.extra))
        arrivals = np.array(monitor.arrivals, dtype=float)
        fields.update(state_t=stamp, tick=tick, kp=kp, kd=kd,
                      arrival_t=arrivals[:, 0] - self.clock_start, arrival_tick=arrivals[:, 1])
        return fields


def prepare_robot(args, monitor, phases):
    if not sys.stdin.isatty():
        raise Refused("--robot needs an interactive terminal, for the confirmations")
    state = read_state(monitor.msg)
    problem = safety_problem(state, check_range=False)
    if problem:
        raise Refused(problem)
    battery = monitor.msg.bms_state.soc
    if battery < MIN_BATTERY_PERCENT:
        raise Refused(f"battery is at {battery}%, minimum {MIN_BATTERY_PERCENT}%")
    where = ("HANGING IN A HARNESS WITH ALL FOUR FEET CLEAR OF THE GROUND" if args.test == "hang"
             else "on a flat floor with clear space around it")
    print(f"\nTHIS WILL MOVE THE ROBOT on {args.robot}.\n"
          f"  Test:    {args.test}, about {phases_seconds(phases):.0f} s of motion\n"
          f"  Gains:   kp {args.kp:g}, kd {args.kd:g}; torque capped at {args.torque_fraction:.0%}\n"
          f"  Battery: {battery}%, hottest motor {state.temperature.max()} °C\n"
          f"The robot must be {where}.\n"
          f"It will lie down first, then stand up.\n")
    word = "hanging" if args.test == "hang" else "yes"
    if input(f'Type "{word}" to continue: ').strip().lower() != word:
        raise Refused("not confirmed")
    rp.confirm_stop_button(monitor)
    return rp.release_sport_mode()


def run_passive(args):
    monitor, _subscriber = connect(args.robot, args.domain, keep_states=True)
    print(f"Recording rt/lowstate for {args.seconds:g} s. Nothing is published, so this can't "
          f"move the robot. Keep it still.", flush=True)
    start = time.perf_counter()
    monitor.arrivals.clear()
    monitor.states.clear()
    time.sleep(args.seconds)
    arrivals = np.array(monitor.arrivals, dtype=float)
    states = list(monitor.states)[:len(arrivals)]
    columns = {name: np.array([getattr(s, name) for s in states])
               for name in ("q", "dq", "tau_est", "temperature", "rpy", "gyro", "accel",
                            "foot_force", "power_v", "power_a")}
    columns.update(arrival_t=arrivals[:, 0] - start, arrival_tick=arrivals[:, 1])
    return columns, {"outcome": "finished"}


def run_dds(args, phases, what):
    try:
        monitor, _subscriber = connect(args.robot, args.domain)
        original_mode = prepare_robot(args, monitor, phases) if args.robot else None
    except KeyboardInterrupt:
        raise Refused("stopped before sending any low-level commands")
    if not args.robot:
        print(f"Simulator: {'start go2/sim_headless.py with --hang' if args.test == 'hang' else 'go2/sim_headless.py'} "
              f"(and --flat --dt 0.002).", flush=True)
    publisher = rp.ChannelPublisher("rt/lowcmd", rp.LowCmd_)
    publisher.Init()
    controller = SysidController(monitor, publisher, args.kp, args.kd, args.torque_fraction)
    outcome = "finished"
    try:
        controller.run_phases(phases, ask=bool(args.robot), what=what)
    except Fault as fault:
        outcome = f"FAULT: {fault}"
    except KeyboardInterrupt:
        outcome = "stopped by the operator"
    except Exception:
        traceback.print_exc()
        outcome = "stopped by an unexpected error"
    finally:
        rp.arm_watchdog(0.0)
        rp.ignore_stop_signals()
        try:
            controller.damp()
        except Exception:
            traceback.print_exc()
            outcome += "; DAMPING FAILED"
    controller.print_summary()
    if args.robot:
        print(f"Sport mode is off (it was {original_mode!r}). Restart the robot to get it back.")
    if not controller.records:
        return None, {"outcome": outcome}
    return controller.columns(monitor), {"outcome": outcome, "original_mode": original_mode,
                                         "torque_fraction": args.torque_fraction}


# --------------------------------------------------------------------------- S-M-ICRL backend


def run_smicrl(args, phases):
    """Step S-M-ICRL's physics with its own PD law (env/env.py, Go2Env._apply_pd):

        tau = kp * (target - q) + kd * (0 - qd) + qfrc_bias, clipped to ctrlrange

    applied every physics step from the current state, so with no delay. --no-gravity-comp
    drops qfrc_bias, to show how much of the training sim's behavior comes from it.
    """
    import mujoco

    hang = args.test == "hang"
    spec = mujoco.MjSpec.from_file(str(args.smicrl))
    if hang:
        # A weld to the world, as the harness holds the robot. Only resetting the base pose
        # each step would let it free-fall during the step, and the legs would feel no gravity.
        spec.add_equality(type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                          name1=args.smicrl_base)
    model = spec.compile()
    data = mujoco.MjData(model)
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_joint") for name in JOINTS]
    if min(joint_ids) < 0:
        raise Refused(f"{args.smicrl} doesn't name its joints like {JOINTS[0]}_joint")
    qpos_idx = model.jnt_qposadr[joint_ids]
    dof_idx = model.jnt_dofadr[joint_ids]
    act_idx = [int(np.flatnonzero((model.actuator_trntype == int(mujoco.mjtTrn.mjTRN_JOINT))
                                  & (model.actuator_trnid[:, 0] == j))[0]) for j in joint_ids]
    lo, hi = model.actuator_ctrlrange[act_idx, 0], model.actuator_ctrlrange[act_idx, 1]
    dt = model.opt.timestep

    def pd(target):
        q, qd = data.qpos[qpos_idx], data.qvel[dof_idx]
        ff = data.qfrc_bias[dof_idx].copy() if not args.no_gravity_comp else np.zeros(12)
        tau = np.clip(args.kp * (target - q) + args.kd * (0.0 - qd) + ff, lo, hi)
        data.ctrl[act_idx] = tau
        return q.copy(), qd.copy(), tau, ff

    def pin():
        """Undo the weld's small give, so only the legs move."""
        if hang:
            data.qpos[:7] = model.qpos0[:7]
            data.qvel[:6] = 0.0

    # Start as Go2Env.reset does: level at the base height, joints at HOME, then settle.
    # Hanging, the base stays where the weld holds it: the model's start pose.
    mujoco.mj_resetData(model, data)
    if not hang:
        data.qpos[:7] = [0.0, 0.0, args.smicrl_height, 1.0, 0.0, 0.0, 0.0]
    data.qpos[qpos_idx] = HOME
    mujoco.mj_forward(model, data)
    for _ in range(500):  # S-M-ICRL's SETTLE_STEPS
        pd(HOME)
        mujoco.mj_step(model, data)
        pin()

    rows = []
    t0 = data.time
    for phase in phases:
        if phase.stage == WAIT_STAGE:
            continue
        q0 = data.qpos[qpos_idx].copy()
        start = data.time
        while data.time - start < phase.seconds - 1e-9:
            target = phase.target_at(data.time - start, q0)
            q, qd, tau, ff = pd(target)
            quat = data.qpos[3:7]
            rows.append((data.time - t0, phase.stage, q, qd, target, target, tau, ff,
                         rp.quat_to_rpy(*quat)))
            mujoco.mj_step(model, data)
            pin()
    t, stage, q, dq, target, q_cmd, tau, ff, rpy = (np.array(c) for c in zip(*rows))
    columns = dict(t=t, stage=stage, q=q, dq=dq, target=target, q_cmd=q_cmd, tau_est=tau, tau_ff=ff,
                   rpy=rpy, state_t=t.copy(), kp=np.full(len(t), args.kp), kd=np.full(len(t), args.kd))
    meta = {"outcome": "finished", "xml": str(args.smicrl), "timestep": dt,
            "gravity_comp": not args.no_gravity_comp,
            "joint_damping": model.dof_damping[dof_idx].tolist(),
            "joint_armature": model.dof_armature[dof_idx].tolist(),
            "joint_frictionloss": model.dof_frictionloss[dof_idx].tolist(),
            "ctrlrange": model.actuator_ctrlrange[act_idx].tolist()}
    return columns, meta


# --------------------------------------------------------------------------- analysis


def load_log(path):
    with np.load(path, allow_pickle=False) as npz:
        d = {k: npz[k] for k in npz.files}
    meta = json.loads(str(d.pop("meta"))) if "meta" in d else {}
    n = len(d.get("t", d.get("arrival_t", [])))
    if "t" in d:  # run_policy.py logs keep the gains in meta only
        d.setdefault("kp", np.full(n, meta.get("kp", KP)))
        d.setdefault("kd", np.full(n, meta.get("kd", KD)))
        d.setdefault("state_t", d["t"])  # it read the state just before logging the row
        d["stage"] = d["stage"].astype(str)
    return d, meta


def group_median(values):
    """{group: median over that group's joints} for a 12-vector that may hold NaN."""
    values = np.asarray(values, dtype=float)
    out = {}
    for group, idx in GROUPS.items():
        v = values[idx]
        v = v[np.isfinite(v)]
        out[group] = round(float(np.median(v)), 4) if v.size else None
    return out


def r(x, digits=4):
    return None if x is None or not np.isfinite(x) else round(float(x), digits)


def tau_pd(d):
    """The PD part of the joint torque: everything but S-M-ICRL's gravity compensation."""
    return d["tau_est"] - d["tau_ff"] if "tau_ff" in d else d["tau_est"]


def timing(d):
    if "arrival_t" not in d or len(d["arrival_t"]) < 3:
        return None
    gaps = np.diff(d["arrival_t"]) * 1000.0
    ticks = np.diff(d["arrival_tick"])
    tick_step = float(np.median(ticks)) if ticks.size else 0.0
    return {"state_hz": r(1000.0 / np.median(gaps), 1),
            "state_gap_ms": {"median": r(np.median(gaps), 2), "p99": r(np.percentile(gaps, 99), 2),
                             "max": r(gaps.max(), 2)},
            "tick_step": r(tick_step, 2),
            "missed_states": int(np.sum(ticks > 1.5 * tick_step)) if tick_step > 0 else None}


def passive(d):
    out = {}
    for name in ("q", "dq", "tau_est"):
        out[f"{name}_noise_std"] = group_median(np.std(d[name], axis=0))
    out["tau_est_mean"] = np.round(np.mean(d["tau_est"], axis=0), 3).tolist()
    out["gyro_bias"] = np.round(np.mean(d["gyro"], axis=0), 5).tolist()
    out["gyro_noise_std"] = np.round(np.std(d["gyro"], axis=0), 5).tolist()
    out["accel_mean"] = np.round(np.mean(d["accel"], axis=0), 4).tolist()
    out["accel_norm"] = r(np.mean(np.linalg.norm(d["accel"], axis=1)))
    out["accel_noise_std"] = np.round(np.std(d["accel"], axis=0), 4).tolist()
    out["rpy_noise_std"] = np.round(np.std(d["rpy"], axis=0), 5).tolist()
    out["foot_force_mean"] = np.round(np.mean(d["foot_force"], axis=0), 1).tolist()
    out["power_v"] = r(np.mean(d["power_v"]), 2)
    out["hottest_c"] = int(d["temperature"].max())
    return out


def stage_rows(d, *prefixes):
    return np.flatnonzero(np.logical_or.reduce([np.char.startswith(d["stage"], p) for p in prefixes]))


def contiguous(d, rows):
    """Split sorted row indices into runs of consecutive rows from the same stage."""
    if rows.size == 0:
        return []
    breaks = (np.diff(rows) > 1) | (d["stage"][rows[1:]] != d["stage"][rows[:-1]])
    return np.split(rows, np.flatnonzero(breaks) + 1)


def steps(d):
    """Latency and step response, from each step edge in the step:* stages."""
    tau = tau_pd(d)
    t, st = d["t"], d["state_t"]
    per_joint = {k: [[] for _ in range(12)] for k in ("tau_ms", "move_ms", "rise_ms", "overshoot_pct", "steady_error")}
    for run in contiguous(d, stage_rows(d, "step:")):
        group = d["stage"][run[0]].split(":")[1]
        for j in GROUPS[group]:
            target = d["target"][run, j]
            edges = np.flatnonzero(np.abs(np.diff(target)) > STEP_SIZE * 0.2) + 1
            bounds = list(edges) + [len(run)]
            for e, end in zip(edges, bounds[1:]):
                i, stop = run[e], run[end - 1]
                if e < 25:
                    continue
                size = target[e] - target[e - 1]
                sign, jump = np.sign(size), abs(d["kp"][i] * size)
                base_tau = np.median(tau[run[e - 25]:i, j])
                base_q = np.median(d["q"][run[e - 25]:i, j])
                later = np.arange(i, stop + 1)
                fresh = later[st[later] >= t[i]]  # states that arrived after the command was sent
                hit = fresh[(tau[fresh, j] - base_tau) * sign > STEP_DETECT * jump]
                if hit.size:
                    per_joint["tau_ms"][j].append((st[hit[0]] - t[i]) * 1000.0)
                moved = (d["q"][fresh, j] - base_q) * sign / abs(size)
                if moved.size == 0:
                    continue
                k = np.flatnonzero(moved > STEP_MOVE)
                if k.size:
                    per_joint["move_ms"][j].append((st[fresh[k[0]]] - t[i]) * 1000.0)
                k10, k90 = np.flatnonzero(moved > 0.1), np.flatnonzero(moved > 0.9)
                if k10.size and k90.size:
                    per_joint["rise_ms"][j].append((st[fresh[k90[0]]] - st[fresh[k10[0]]]) * 1000.0)
                per_joint["overshoot_pct"][j].append(max(0.0, moved.max() - 1.0) * 100.0)
                tail = fresh[int(len(fresh) * 0.7):]
                per_joint["steady_error"][j].append(float(np.mean(d["target"][tail, j] - d["q"][tail, j])))
    if not any(len(v) for v in per_joint["tau_ms"]):
        return None
    medians = {k: [np.median(v) if v else np.nan for v in vals] for k, vals in per_joint.items()}
    return {
        "torque_response_ms": group_median(medians["tau_ms"]),
        "joint_moves_ms": group_median(medians["move_ms"]),
        "rise_10_90_ms": group_median(medians["rise_ms"]),
        "overshoot_pct": group_median(medians["overshoot_pct"]),
        "steady_error_rad": group_median(np.abs(medians["steady_error"])),
        "per_joint_torque_response_ms": [r(v, 1) for v in medians["tau_ms"]],
        "note": "torque_response_ms is the time from publishing a step to the first state whose "
                "tau_est shows half the expected jump: command transport + motor loop + state "
                "transport. 0 in S-M-ICRL.",
    }


def torque_fit(d, rows):
    """Fit tau = a * (kp * (q_cmd(t - delay) - q) - kd * dq) + b per joint, with one delay per group.

    The motor computes PD from fresh q and dq but an older command, so the delay that fits
    best is the command's age when the motor uses it. a is tau_est's scale against the PD
    law: 1 if tau_est is calibrated and the motor isn't saturating.
    """
    _, first = np.unique(d["state_t"][rows], return_index=True)  # one row per state message
    rows = rows[np.sort(first)]
    if rows.size < 200:
        return None
    st, t = d["state_t"][rows], d["t"]
    tau = tau_pd(d)[rows]
    kp, kd = d["kp"][rows], d["kd"][rows]
    q, dq = d["q"][rows], d["dq"][rows]
    delays = np.arange(0.0, MAX_DELAY_S + 1e-9, 0.001)
    out = {"delay_ms": {}, "scale": {}, "offset_nm": {}, "r2": {}}
    for group, idx in GROUPS.items():
        best = None
        for delay in delays:
            fits = []
            for j in idx:
                cmd = np.interp(st - delay, t, d["q_cmd"][:, j])
                model = kp * (cmd - q[:, j]) - kd * dq[:, j]
                ok = np.abs(model) < 0.95 * TORQUE_LIMIT[j]
                if ok.sum() < 100 or np.std(model[ok]) < 1e-6:
                    continue
                A = np.column_stack([model[ok], np.ones(ok.sum())])
                coef, *_ = np.linalg.lstsq(A, tau[ok, j], rcond=None)
                resid = tau[ok, j] - A @ coef
                fits.append((np.sum(resid ** 2), np.var(tau[ok, j]) * ok.sum(), coef))
            if not fits:
                continue
            sse = sum(f[0] for f in fits)
            if best is None or sse < best[0]:
                best = (sse, delay, fits)
        if best is None:
            continue
        sse, delay, fits = best
        out["delay_ms"][group] = r(delay * 1000.0, 1)
        out["scale"][group] = r(np.median([f[2][0] for f in fits]), 3)
        out["offset_nm"][group] = r(np.median([f[2][1] for f in fits]), 3)
        out["r2"][group] = r(1.0 - sse / max(sum(f[1] for f in fits), 1e-12), 3)
    return out if out["delay_ms"] else None


def sines(d):
    out = {}
    for run in contiguous(d, stage_rows(d, "sine:")):
        _, group, hz = d["stage"][run[0]].split(":")
        f = float(hz)
        t = d["t"][run] - d["t"][run[0]]
        st = d["state_t"][run] - d["t"][run[0]]
        keep = t >= 1.0 / f  # skip the first cycle
        if keep.sum() < 20:
            continue
        gains, lags = [], []
        for j in GROUPS[group]:
            def fit(time, y):
                w = 2.0 * math.pi * f
                A = np.column_stack([np.sin(w * time), np.cos(w * time), np.ones_like(time)])
                (s, c, _), *_ = np.linalg.lstsq(A, y, rcond=None)
                return math.hypot(s, c), math.atan2(c, s)
            amp_t, ph_t = fit(t[keep], d["target"][run[keep], j])
            amp_q, ph_q = fit(st[keep], d["q"][run[keep], j])
            gains.append(amp_q / amp_t)
            lags.append(math.degrees((ph_t - ph_q + math.pi) % (2.0 * math.pi) - math.pi))
        lag = float(np.median(lags))
        out.setdefault(group, {})[hz] = {"gain": r(np.median(gains), 3), "phase_lag_deg": r(lag, 1),
                                         "lag_ms": r(lag / 360.0 / f * 1000.0, 1)}
    return out or None


def friction(d):
    """Coulomb and viscous friction from the ramp:* stages.

    At constant speed the motor supplies gravity + friction. Moving up and down through
    the same angle, gravity is the same and friction flips sign, so half the difference
    in torque is the friction at that speed.
    """
    tau = tau_pd(d)
    per_joint = {}  # j -> [(speed, friction, gravity torque)]
    for run in contiguous(d, stage_rows(d, "ramp:")):
        _, group, speed = d["stage"][run[0]].split(":")
        v = float(speed)
        for j in GROUPS[group]:
            dq, q = d["dq"][run, j], d["q"][run, j]
            slope = np.gradient(d["target"][run, j], d["t"][run])
            up = (slope > 0.5 * v) & (dq > 0.5 * v) & (dq < 1.5 * v)
            down = (slope < -0.5 * v) & (dq < -0.5 * v) & (dq > -1.5 * v)
            if up.sum() < 20 or down.sum() < 20:
                continue
            bins = np.linspace(q.min(), q.max(), 9)
            halves, middles = [], []
            for lo_, hi_ in zip(bins[:-1], bins[1:]):
                inside = (q >= lo_) & (q < hi_)
                a, b = tau[run[up & inside], j], tau[run[down & inside], j]
                if a.size >= 5 and b.size >= 5:
                    halves.append((a.mean() - b.mean()) / 2.0)
                    middles.append((a.mean() + b.mean()) / 2.0)
            if halves:
                per_joint.setdefault(j, []).append((v, float(np.mean(halves)), float(np.mean(middles))))
    if not per_joint:
        return None
    coulomb, viscous, at_speed = np.full(12, np.nan), np.full(12, np.nan), {}
    for j, points in per_joint.items():
        speeds = np.array([p[0] for p in points])
        values = np.array([p[1] for p in points])
        for v, h, _ in points:
            at_speed.setdefault(f"{v:g}", np.full(12, np.nan))[j] = h
        if len(np.unique(speeds)) >= 2:
            viscous[j], coulomb[j] = np.polyfit(speeds, values, 1)
        else:
            coulomb[j] = values.mean()
    return {"coulomb_nm": group_median(coulomb), "viscous_nm_s_per_rad": group_median(viscous),
            "friction_at_speed_nm": {k: group_median(v) for k, v in at_speed.items()},
            "per_joint_coulomb_nm": [r(v, 3) for v in coulomb]}


def stand(d):
    rows = stage_rows(d, "stand:hold")
    if rows.size == 0:
        return None
    rows = rows[d["t"][rows] >= d["t"][rows[0]] + 1.0]  # let it settle
    sag = np.mean(d["target"][rows] - d["q"][rows], axis=0)
    out = {"sag_rad": [r(v) for v in sag],
           "sag_abs_by_group": group_median(np.abs(sag)),
           "front_rear_thigh_sag": [r(np.mean(sag[[1, 4]])), r(np.mean(sag[[7, 10]]))],
           "tau_mean_nm": [r(v, 2) for v in np.mean(d["tau_est"][rows], axis=0)],
           "tau_abs_by_group": group_median(np.abs(np.mean(d["tau_est"][rows], axis=0)))}
    if "tau_ff" in d:
        out["gravity_comp_abs_by_group"] = group_median(np.abs(np.mean(d["tau_ff"][rows], axis=0)))
    if "foot_force" in d:
        out["foot_force_mean"] = [r(v, 1) for v in np.mean(d["foot_force"][rows], axis=0)]
    if "power_a" in d:
        out["current_a"] = r(np.mean(d["power_a"][rows]), 2)
        out["voltage_v"] = r(np.mean(d["power_v"][rows]), 2)
    return out


def torque_usage(d, meta):
    rows = np.flatnonzero(~np.isin(d["stage"], ["damping"]))
    tau = np.abs(d["tau_est"][rows])
    speed = np.abs(d["dq"][rows])
    clipped = np.abs(d["q_cmd"][rows] - d["target"][rows]) > 1e-6
    out = {}
    for group, idx in GROUPS.items():
        g_tau, g_speed = tau[:, idx].ravel(), speed[:, idx].ravel()
        limit = TORQUE_LIMIT[idx[0]]
        envelope = {}
        for lo_, hi_ in zip(SPEED_BINS[:-1], SPEED_BINS[1:]):
            inside = (g_speed >= lo_) & (g_speed < hi_)
            if inside.any():
                envelope[f"{lo_:g}-{hi_:g}"] = r(g_tau[inside].max(), 2)
        out[group] = {"peak_nm": r(g_tau.max(), 2), "p99_nm": r(np.percentile(g_tau, 99), 2),
                      "limit_nm": limit, "share_above_90pct_limit": r(np.mean(g_tau > 0.9 * limit), 4),
                      "share_clipped_by_cap": r(np.mean(clipped[:, idx]), 4),
                      "peak_speed_rad_s": r(g_speed.max(), 2),
                      "peak_torque_by_speed": envelope}
    return out


def tracking(d, prefixes=("walking", "playing")):
    rows = np.flatnonzero(np.isin(d["stage"], prefixes))
    if rows.size < 100:
        return None
    err = d["target"][rows] - d["q"][rows]
    t = d["t"][rows]
    dt = float(np.median(np.diff(t)))
    lags = np.full(12, np.nan)
    for j in range(12):
        a = d["target"][rows, j] - d["target"][rows, j].mean()
        b = d["q"][rows, j] - d["q"][rows, j].mean()
        if np.std(a) < 1e-4:
            continue
        shifts = range(0, int(0.3 / dt))
        corr = [np.dot(a[:len(a) - s], b[s:]) for s in shifts]
        lags[j] = int(np.argmax(corr)) * dt * 1000.0
    return {"rms_error_rad": group_median(np.sqrt(np.mean(err ** 2, axis=0))),
            "peak_error_rad": group_median(np.abs(err).max(axis=0)),
            "lag_ms": group_median(lags)}


def analyze(d, meta):
    if "t" not in d:  # passive
        return {"timing": timing(d), "passive": passive(d)}
    moving = stage_rows(d, "step:", "sine:", "ramp:", "walking", "playing")
    out = {"timing": timing(d), "step": steps(d), "torque_fit": torque_fit(d, moving),
           "sine": sines(d), "friction": friction(d), "stand": stand(d),
           "torque_usage": torque_usage(d, meta), "tracking": tracking(d)}
    if "tau_ff" in d and np.abs(d["tau_ff"]).max() > 0:
        out["gravity_comp"] = {"mean_abs_nm": group_median(np.mean(np.abs(d["tau_ff"]), axis=0)),
                               "note": "torque S-M-ICRL adds to every joint for free; the robot's motors don't"}
    return {k: v for k, v in out.items() if v is not None}


def print_results(results):
    print(json.dumps(results, indent=1))


# --------------------------------------------------------------------------- compare


def flatten(obj, prefix=""):
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield from flatten(v, f"{prefix}.{k}" if prefix else k)
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        yield prefix, float(obj)


def suggestions(ref, other, name):
    """Hints for moving `other` (a simulator) toward `ref` (the robot)."""
    tips = []
    get = lambda res, *keys: _dig(res, keys)  # noqa: E731
    for group in GROUPS:
        a, b = get(ref, "torque_fit", "delay_ms", group), get(other, "torque_fit", "delay_ms", group)
        if a is not None and b is not None and a - b >= 3:
            tips.append(f"{group}: the robot's command delay is {a:g} ms, {name}'s is {b:g} ms. "
                        f"Apply each target about {a - b:g} ms late in {name}.")
        a, b = get(ref, "friction", "coulomb_nm", group), get(other, "friction", "coulomb_nm", group)
        if a is not None and b is not None and abs(a - b) > 0.1:
            tips.append(f"{group}: Coulomb friction {a:.2f} N·m on the robot vs {b:.2f} in {name}. "
                        f"Change that joint's frictionloss by about {a - b:+.2f}.")
        a, b = (get(ref, "friction", "viscous_nm_s_per_rad", group),
                get(other, "friction", "viscous_nm_s_per_rad", group))
        if a is not None and b is not None and abs(a - b) > 0.05:
            tips.append(f"{group}: viscous friction {a:.2f} N·m·s/rad on the robot vs {b:.2f} in {name}. "
                        f"Change that joint's damping by about {a - b:+.2f}.")
        a, b = get(ref, "torque_fit", "scale", group), get(other, "torque_fit", "scale", group)
        if a is not None and b is not None and abs(a / b - 1.0) > 0.1:
            tips.append(f"{group}: tau_est is {a:.2f}x the PD law on the robot vs {b:.2f}x in {name}. "
                        f"Either tau_est is miscalibrated or the motor is weaker than modeled.")
        a, b = get(ref, "stand", "sag_abs_by_group", group), get(other, "stand", "sag_abs_by_group", group)
        if a is not None and b is not None and abs(a - b) > 0.03:
            tips.append(f"{group}: standing sag {a:.3f} rad on the robot vs {b:.3f} in {name}.")
    ff = get(other, "gravity_comp", "mean_abs_nm")
    if ff and not get(ref, "gravity_comp"):
        tips.append(f"{name} adds gravity compensation (mean |tau| {ff}), which the robot's motors "
                    f"don't. A policy trained with it holds poses the robot sags out of. Train without "
                    f"it, or send the same feedforward as motor_cmd.tau on the robot.")
    return tips


def _dig(obj, keys):
    for k in keys:
        if not isinstance(obj, dict) or k not in obj:
            return None
        obj = obj[k]
    return obj


def compare(paths):
    results = [json.loads(Path(p).read_text()) for p in paths]
    names = [Path(p).stem for p in paths]
    flat = [dict(flatten(res.get("results", res))) for res in results]
    keys = [k for k in flat[0] if any(k in f for f in flat[1:])]
    width = max([len(k) for k in keys] + [10])
    print(f"{'':{width}}  " + "  ".join(f"{n[-14:]:>14}" for n in names))
    for key in keys:
        cells = [f"{f[key]:>14.4g}" if key in f else f"{'-':>14}" for f in flat]
        print(f"{key:{width}}  " + "  ".join(cells))
    for res, name in zip(results[1:], names[1:]):
        tips = suggestions(results[0].get("results", results[0]), res.get("results", res), name)
        if tips:
            print(f"\nTo bring {name} closer to {names[0]}:")
            for tip in tips:
                print(f"  - {tip}")


# --------------------------------------------------------------------------- main


def save(columns, meta, args):
    backend = "smicrl" if args.smicrl else ("robot" if args.robot else "sim")
    path = args.log or LOG_DIR / f"{datetime.now():%Y-%m-%d_%H-%M-%S}_{args.test}_{backend}.npz"
    path.parent.mkdir(parents=True, exist_ok=True)
    meta = {**meta, "test": args.test, "backend": backend, "interface": args.robot or "lo",
            "kp": args.kp, "kd": args.kd, "scale": getattr(args, "scale", None)}
    np.savez_compressed(path, meta=json.dumps(meta), **columns)
    return path, meta


def write_results(npz_path, meta, results):
    json_path = npz_path.with_suffix(".json")
    json_path.write_text(json.dumps({"meta": meta, "results": results}, indent=1))
    return json_path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("test", choices=("passive", "hang", "stand", "analyze", "compare"))
    parser.add_argument("files", nargs="*", type=Path, help="for analyze: a .npz; for compare: .json files, robot first")
    parser.add_argument("--robot", metavar="INTERFACE", nargs="?", const=DEFAULT_INTERFACE,
                        help="use the robot, on this network interface (if none is given: %(const)s); "
                             "without --robot or --smicrl, use go2/sim_headless.py")
    parser.add_argument("--smicrl", metavar="XML", type=Path, nargs="?", const=DEFAULT_SMICRL_XML,
                        help="run the test in S-M-ICRL's physics on this scene (if none is given: %(const)s)")
    parser.add_argument("--no-gravity-comp", action="store_true",
                        help="with --smicrl, leave out S-M-ICRL's gravity compensation")
    parser.add_argument("--smicrl-base", default="base_link",
                        help="with --smicrl hang, the body to weld to the world (default: %(default)s)")
    parser.add_argument("--smicrl-height", type=bounded(0.1, 0.6), default=0.30,
                        help="with --smicrl stand, the base height it starts from (default: %(default)s)")
    parser.add_argument("--groups", default="hip,thigh,calf", help="hang: joint types to test (default: %(default)s)")
    parser.add_argument("--scale", type=bounded(0.25, 1.5), default=1.0,
                        help="hang: multiply every test motion's size by this (default: %(default)s)")
    parser.add_argument("--seconds", type=bounded(1.0, 120.0),
                        help=f"passive: how long to record (default {PASSIVE_S:g}); "
                             f"stand: how long to hold (default {STAND_HOLD_S:g})")
    parser.add_argument("--kp", type=bounded(1.0, 200.0), default=KP, help="default: %(default)s")
    parser.add_argument("--kd", type=bounded(0.0, 20.0), default=KD, help="default: %(default)s")
    parser.add_argument("--torque-fraction", type=bounded(0.1, 1.0), default=0.8,
                        help="share of the torque limits to aim for (default: %(default)s)")
    parser.add_argument("--domain", type=int, help="DDS domain id (default: 0 with --robot, otherwise 1)")
    parser.add_argument("--log", type=Path, help="where to save the run (default: go2/logs/sysid/<time>_<test>_<backend>.npz)")
    args = parser.parse_args()
    if args.test in ("analyze", "compare"):
        if not args.files or (args.test == "analyze" and len(args.files) != 1) or (
                args.test == "compare" and len(args.files) < 2):
            parser.error("analyze takes one .npz; compare takes two or more .json files")
    elif args.files:
        parser.error(f"{args.test} takes no files")
    if args.robot and args.smicrl:
        parser.error("choose --robot or --smicrl, not both")
    if args.test == "passive" and args.smicrl:
        parser.error("passive records DDS state, so it needs the robot or sim_headless.py")
    groups = [g.strip() for g in args.groups.split(",") if g.strip()]
    if not groups or set(groups) - set(GROUPS):
        parser.error(f"--groups takes some of: {', '.join(GROUPS)}")
    args.groups = groups
    if args.seconds is None:
        args.seconds = PASSIVE_S if args.test == "passive" else STAND_HOLD_S
    return args


def main():
    args = parse_args()
    if args.test == "compare":
        compare(args.files)
        return 0
    if args.test == "analyze":
        d, meta = load_log(args.files[0])
        results = analyze(d, meta)
        print_results(results)
        print(f"Results: {write_results(args.files[0], meta, results)}")
        return 0

    signal.signal(signal.SIGALRM, rp._on_watchdog)
    rp.stop_on_hangup()
    phases = (hang_phases(args.groups, args.scale) if args.test == "hang"
              else stand_phases(args.seconds) if args.test == "stand" else None)
    try:
        if args.test == "passive":
            columns, meta = run_passive(args)
        elif args.smicrl:
            columns, meta = run_smicrl(args, phases)
        else:
            columns, meta = run_dds(args, phases, f"the {args.test} test")
    except (Refused, Fault) as error:
        print(f"REFUSED: {error}", flush=True)
        return 2
    except KeyboardInterrupt:
        print("Stopped before sending any low-level commands.", flush=True)
        return 2
    if columns is None:
        print(f"Result: {meta['outcome']}; nothing recorded.")
        return 1
    path, meta = save(columns, meta, args)
    results = analyze(load_log(path)[0], meta)
    print_results(results)
    print(f"\nResult: {meta['outcome']}\nLog: {path}\nResults: {write_results(path, meta, results)}")
    return 0 if meta["outcome"] == "finished" else 1


if __name__ == "__main__":
    sys.exit(main())
