#!/usr/bin/env python
"""Record the Go2 walking in sport mode, and rebuild the motion later. Recording is READ-ONLY.

    python go2/record_walk.py record --robot                 # walk it with the remote; Ctrl-C saves
    python go2/record_walk.py record --robot enp3s0 --seconds 60 --note "slow trot, 2 m forward"
    python go2/record_walk.py record                          # go2/sim_headless.py
    python go2/record_walk.py info RECORDING.npz              # what's in it, and when the robot moved
    python go2/record_walk.py replay RECORDING.npz --video walk.mp4      # the motion again, in MuJoCo
    python go2/record_walk.py export RECORDING.npz --start 12 --end 20   # a joint file and a CSV

record subscribes to rt/lowstate (joint angles, speeds and torques, IMU, foot force, the
remote's sticks), rt/sportmodestate (sport mode's own body position, attitude and
velocity) and rt/lowcmd (the joint targets and gains sport mode sends its motors). It
publishes nothing and creates no SDK client, so it can't move the robot: you walk it,
with the remote or the app. Every message's time comes from the robot's own clock, so
the recording stays right when this computer falls behind.

info, replay and export rebuild the motion on an even clock: where the body was, how
it was turned, and all 12 joint angles. The start of the stretch you pick is the origin,
facing +x. They need numpy only (replay also MuJoCo and OpenCV), not the SDK, so they
also run on another computer. They read go2/watch.py recordings too.

  replay  draws the Go2 model going through the recorded poses. Nothing is simulated.
  export  writes the joint angles for `go2/run_policy.py --trajectory`, and a CSV of
          the whole rebuilt motion for any other tool.

Exit status: 0 = done, 1 = nothing recorded, or the recording can't be used that way,
2 = couldn't connect.
"""
import argparse
import json
import os
import signal
import struct
import sys
import threading
import time
from array import array
from datetime import datetime
from pathlib import Path

import numpy as np
from state_estimator import FOOT_RADIUS, BaseEstimator, foot_positions, rotation

HERE = Path(__file__).resolve().parent
LOG_DIR = HERE / "logs" / "walks"
SIM_INTERFACE = "lo"

# Joint order in every array here: the SDK's motor order.
JOINTS = [f"{leg}_{part}" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
# Column order of the exported joint file: S-M-ICRL's, as in its joint_trajectories.txt.
TRAJECTORY_ORDER = [f"{leg}_{part}" for leg in ("FL", "FR", "RL", "RR") for part in ("hip", "thigh", "calf")]

POLL_PERIOD = 0.004  # s between collections of the messages that have arrived, as go2/watch.py
DEPTH = 4000  # messages DDS keeps for a topic between collections: 8 s of rt/lowstate
STATUS_PERIOD = 1.0  # s
CONNECT_TIMEOUT = 5.0  # s
MAX_SECONDS = 600.0  # the recording is held in memory until it's saved, about 40 MB a minute
BEHIND_WARN = 0.25  # s between the robot sending a state and this program collecting it
STATE_PERIOD = 0.002  # s between rt/lowstate messages on the robot

CLOCK_WINDOW = 10.0  # s; see robot_times
GAP = 0.05  # s without a message that counts as a hole in the recording
MOVING_SPEED, MOVING_TURN, MOVING_JOINTS = 0.05, 0.15, 0.3  # m/s, rad/s, rad/s
MOVING_MERGE, MOVING_MIN = 1.0, 0.5  # s
ODOMETRY_HZ = 250.0  # how often the leg odometry runs when there's no rt/sportmodestate
ODOMETRY_LEAD = 1.0  # s of recording before the stretch that it settles on
VIDEO_FPS = 30
VIDEO_SIZE = (640, 480)  # MuJoCo's default offscreen buffer


class Problem(Exception):
    """A reason this recording can't be used the way that was asked."""


# --------------------------------------------------------------------------- recording

# (name, values per message, type in the saved file). Names match go2/watch.py's.
STATE_FIELDS = (("tick", 1, np.int64), ("q", 12, np.float32), ("dq", 12, np.float32),
                ("tau_est", 12, np.float32), ("temperature", 12, np.int32), ("quaternion", 4, np.float32),
                ("gyro", 3, np.float32), ("accel", 3, np.float32), ("imu_rpy", 3, np.float32),
                ("foot_force", 4, np.int32), ("foot_force_est", 4, np.int32), ("power_v", 1, np.float32),
                ("power_a", 1, np.float32), ("battery", 1, np.int32),
                ("remote", 4, np.float32),  # the sticks: lx, ly, rx, ry, each -1 to 1
                ("remote_keys", 1, np.int32))
CMD_FIELDS = (("q", 12, np.float32), ("dq", 12, np.float32), ("kp", 12, np.float32), ("kd", 12, np.float32),
              ("tau", 12, np.float32))
SPORT_FIELDS = (("stamp", 1, np.float64), ("position", 3, np.float32), ("velocity", 3, np.float32),
                ("yaw_speed", 1, np.float32), ("body_height", 1, np.float32), ("mode", 1, np.int32),
                ("gait_type", 1, np.int32), ("foot_raise_height", 1, np.float32), ("progress", 1, np.float32),
                ("error_code", 1, np.int32), ("quaternion", 4, np.float32), ("rpy", 3, np.float32),
                ("foot_position_body", 12, np.float32), ("foot_speed_body", 12, np.float32),
                ("foot_force", 4, np.int32), ("range_obstacle", 4, np.float32))


def lowstate_row(msg):
    motors, imu = msg.motor_state[:12], msg.imu_state
    remote = bytes(msg.wireless_remote)
    lx, rx, ry, _, ly = struct.unpack_from("<5f", remote, 4)  # as example/wireless_controller
    return [msg.tick, *[m.q for m in motors], *[m.dq for m in motors], *[m.tau_est for m in motors],
            *[m.temperature for m in motors], *imu.quaternion, *imu.gyroscope, *imu.accelerometer, *imu.rpy,
            *msg.foot_force, *msg.foot_force_est, msg.power_v, msg.power_a, msg.bms_state.soc,
            lx, ly, rx, ry, remote[2] | remote[3] << 8]


def lowcmd_row(msg):
    motors = msg.motor_cmd[:12]
    return [*[m.q for m in motors], *[m.dq for m in motors], *[m.kp for m in motors],
            *[m.kd for m in motors], *[m.tau for m in motors]]


def sport_row(msg):
    return [msg.stamp.sec + 1e-9 * msg.stamp.nanosec, *msg.position, *msg.velocity, msg.yaw_speed,
            msg.body_height, msg.mode, msg.gait_type, msg.foot_raise_height, msg.progress, msg.error_code,
            *msg.imu_state.quaternion, *msg.imu_state.rpy, *msg.foot_position_body, *msg.foot_speed_body,
            *msg.foot_force, *msg.range_obstacle]


def sent_time(msg):
    """When DDS's writer sent this message, in s on the sender's clock; 0 if unknown."""
    stamp = getattr(getattr(msg, "sample_info", None), "source_timestamp", 0)
    return stamp * 1e-9 if stamp and stamp > 0 else 0.0


class Stream:
    """One topic's messages as rows of numbers.

    poll() collects the messages that arrived since the last call; the main loop calls it
    every POLL_PERIOD. DDS isn't given a handler to call for each message, because handler
    calls wait for the Python interpreter and fall behind when it's busy (see
    run_policy.StateMonitor). DDS keeps up to DEPTH messages between calls.

    A row starts with when it was collected (time.perf_counter()) and when DDS's writer
    sent it. The first only says how far behind this program is. The message's real time
    comes from the robot's clock: see robot_times. Each topic reads the clock just before
    its own collection: with one reading for all three, a slow collection of the first
    topic made the others' messages look read before they had arrived.
    """

    def __init__(self, subscriber, fields, extract):
        self.fields = (("sent", 1, np.float64),) + tuple(fields)
        self.width = 1 + sum(width for _, width, _ in self.fields)
        self.count = 0
        self.first = self.last = None  # the first row and the newest
        self.bad = 0
        self.error = None
        self._subscriber = subscriber
        self._extract = extract
        self._data = array("d")
        self._at = {}
        start = 1
        for name, width, _ in self.fields:
            self._at[name] = slice(start, start + width)
            start += width

    def poll(self):
        now = time.perf_counter()
        for msg in self._subscriber.ReadAll():
            try:
                row = [now, sent_time(msg), *self._extract(msg)]
                if len(row) != self.width:
                    raise ValueError(f"{len(row)} values in a message, expected {self.width}")
                self._data.extend(row)
            except Exception as error:  # a message this script doesn't understand: count it, keep recording
                self.bad += 1
                self.error = self.error or repr(error)
                continue
            if self.first is None:
                self.first = row
            self.last = row
            self.count += 1

    def value(self, name, row=None):
        """A field of the newest message, or of row, as a list."""
        return (row or self.last)[self._at[name]]

    def columns(self, prefix):
        rows = np.frombuffer(self._data, dtype=np.float64).reshape(-1, self.width)
        if not len(rows):
            return {}
        out = {f"{prefix}_t": rows[:, 0].copy()}
        for name, width, kind in self.fields:
            column = rows[:, self._at[name]].astype(kind)
            out[f"{prefix}_{name}"] = column[:, 0] if width == 1 else column
        return out


def status_line(elapsed, rates, state, sport, origin, behind, lost):
    parts = [f"t={elapsed:6.1f}", "  ".join(f"{name} {rate:3.0f} Hz" for name, rate in rates.items())]
    if origin is not None:
        position, velocity = sport.value("position"), sport.value("velocity")
        parts.append(f"{np.hypot(position[0] - origin[0], position[1] - origin[1]):5.2f} m from the start, "
                     f"speed {np.hypot(velocity[0], velocity[1]):4.2f} m/s, "
                     f"turning {sport.value('yaw_speed')[0]:+5.2f} rad/s, height {position[2]:.2f} m")
    lx, ly, rx, _ = state.value("remote")
    if max(abs(lx), abs(ly), abs(rx)) > 0.02:
        parts.append(f"sticks fwd {ly:+.2f} side {lx:+.2f} turn {rx:+.2f}")
    if behind > 0.02:
        parts.append(f"{behind * 1000:.0f} ms behind")
    if lost > 0.001:
        parts.append(f"lost {lost:.1%}")
    return "  |  ".join(parts)


def record(args):
    try:
        from check_connection import DEFAULT_INTERFACE, check_interface
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
    except ImportError as error:
        print(f"record needs the Unitree SDK, which go2/setup_env.sh installs: {error}")
        return 2
    on_robot = args.robot is not None
    interface = (args.robot or DEFAULT_INTERFACE) if on_robot else SIM_INTERFACE
    domain = args.domain if args.domain is not None else (0 if on_robot else 1)
    if on_robot and not check_interface(interface):
        return 2
    try:
        ChannelFactoryInitialize(domain, interface)
    except Exception as error:
        print(f"Couldn't start DDS on {interface}, domain {domain}: {error}")
        return 2
    streams = {}
    for name, topic, kind, fields, extract in (("state", "rt/lowstate", LowState_, STATE_FIELDS, lowstate_row),
                                               ("sport", "rt/sportmodestate", SportModeState_, SPORT_FIELDS, sport_row),
                                               ("cmd", "rt/lowcmd", LowCmd_, CMD_FIELDS, lowcmd_row)):
        subscriber = ChannelSubscriber(topic, kind)
        subscriber.InitPolled(DEPTH)
        streams[name] = Stream(subscriber, fields, extract)
    state, sport, cmd = streams["state"], streams["sport"], streams["cmd"]

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    def on_hangup(*_):  # the terminal closed: save quietly
        try:
            fd = os.open(os.devnull, os.O_WRONLY)
            os.dup2(fd, 1)
            os.dup2(fd, 2)
        except OSError:
            pass
        stop.set()

    signal.signal(signal.SIGHUP, on_hangup)

    def collect():
        for stream in streams.values():
            stream.poll()
        return time.perf_counter()

    deadline = time.perf_counter() + CONNECT_TIMEOUT
    while state.last is None:
        if stop.wait(POLL_PERIOD):
            return 1
        if collect() > deadline:
            where = "Is the robot connected?" if on_robot else "Is go2/sim_headless.py running?"
            print(f"No rt/lowstate on {interface}, domain {domain}, after {CONNECT_TIMEOUT:g} s. {where}")
            return 2

    print(f"Recording {interface}, domain {domain}. Read-only: this publishes nothing.\n"
          f"Walk the robot in sport mode now. Ctrl-C stops and saves, or it stops after {args.seconds:g} s.",
          flush=True)
    start = last_status = time.perf_counter()
    started_wall = datetime.now()
    counts = {name: stream.count for name, stream in streams.items()}
    smallest_gap = None
    warned = set()

    def warn(key, text):
        if key not in warned:
            warned.add(key)
            print(f"  WARNING: {text}", flush=True)

    while not stop.wait(POLL_PERIOD):
        now = collect()
        if now - last_status < STATUS_PERIOD:
            continue
        rates = {name: (stream.count - counts[name]) / (now - last_status) for name, stream in streams.items()}
        counts = {name: stream.count for name, stream in streams.items()}
        last_status = now
        behind = lost = 0.0
        read, tick, ticks = state.last[0], state.last[2], state.last[2] - state.first[2]
        if ticks > 0:  # the simulator's tick doesn't advance
            gap = read - tick / 1000.0
            smallest_gap = gap if smallest_gap is None else min(smallest_gap, gap)
            behind = gap - smallest_gap  # how old the newest state is, past the freshest one seen
            lost = max(0.0, 1.0 - (state.count - 1) / (ticks / 1000.0 / STATE_PERIOD))
        origin = sport.value("position", sport.first) if sport.first is not None else None
        if not args.quiet:
            print(status_line(now - start, rates, state, sport, origin, behind, lost), flush=True)
        if rates["state"] == 0:
            warn("state", "rt/lowstate stopped. Check the cable.")
        if now - start > 2.0 and sport.count == 0:
            warn("sport", "no rt/sportmodestate, so the body's position isn't being recorded, only the joints "
                          "and IMU. Is sport mode on?")
        if behind > BEHIND_WARN:
            warn("behind", f"this program is {behind * 1000:.0f} ms behind the robot. Messages' times come from "
                           f"the robot's clock, so the recording is still right, but past "
                           f"{DEPTH * STATE_PERIOD:g} s behind, messages are lost. Close other programs.")
        for name, stream in streams.items():
            if stream.error:
                warn(f"bad {name}", f"some {name} messages couldn't be read: {stream.error}")
        if now - start >= args.seconds:
            break
    collect()  # whatever arrived since the last collection

    data = {}
    for name, stream in streams.items():
        data.update(stream.columns(name))
    if "state_t" not in data or len(data["state_t"]) < 2:
        print("Nothing recorded.")
        return 1
    path = args.log or LOG_DIR / f"{started_wall:%Y-%m-%d_%H-%M-%S}.npz"
    path.parent.mkdir(parents=True, exist_ok=True)
    meta = {"kind": "record_walk", "version": 1, "interface": interface, "domain": domain,
            "started": start, "started_wall": started_wall.isoformat(timespec="seconds"), "note": args.note,
            "joints": JOINTS, "remote": ["lx", "ly", "rx", "ry"],
            "topics": {"lowstate": state.count, "sportmodestate": sport.count, "lowcmd": cmd.count},
            "unreadable": {name: stream.bad for name, stream in streams.items() if stream.bad}}
    np.savez_compressed(path, meta=json.dumps(meta), **data)
    print(f"\nSaved: {path}")
    try:
        print_info(load(path))
    except Problem as problem:
        print(f"  {problem}")
    print(f"Next:\n  python go2/record_walk.py replay {path} --video {path.with_suffix('.mp4')}\n"
          f"  python go2/record_walk.py export {path}")
    return 0


# --------------------------------------------------------------------------- reading a recording


def robot_times(read, stamp):
    """Each message's robot-clock stamp, moved onto this computer's clock, and how late it was read.

    A message can wait before this program reads it, for seconds when the computer is
    busy, so the time it was read doesn't say when it was measured. Its stamp does, on
    the robot's clock. The smallest gap between the two clocks belongs to a message that
    was read as soon as it was sent, and a message's lateness is how much longer than
    that it took.

    The two clocks drift apart by a few ms a minute, so the smallest gap is followed as a
    line: its slope is the median slope between the smallest gaps of each CLOCK_WINDOW,
    which stretches where every message was late can't tilt.

    Returns (None, None) when the stamps don't advance with time (the simulator's tick is 0).
    """
    span = read[-1] - read[0]
    if len(read) < 2 or span <= 0 or not 0.5 < (stamp.max() - stamp.min()) / span < 2.0:
        return None, None
    gap = read - stamp
    edges = np.searchsorted(read, np.arange(read[0], read[-1] + CLOCK_WINDOW, CLOCK_WINDOW))
    windows = [(read[a:b].mean(), gap[a:b].min()) for a, b in zip(edges[:-1], edges[1:]) if b - a >= 10]
    slope = 0.0
    if len(windows) >= 4:
        x, y = np.array(windows).T
        i, j = np.triu_indices(len(x), 1)
        slope = float(np.median((y[j] - y[i]) / (x[j] - x[i])))
    line = slope * (read - read[0])
    offset = line + np.min(gap - line)
    return stamp + offset, gap - offset


def spread(read):
    """Messages collected together share a read time. Give each its own, evenly spaced since
    the collection before: that is when they arrived, if they come at a steady rate."""
    first = np.flatnonzero(np.diff(read, prepend=-np.inf) > 0)  # where each collection starts
    if len(first) == len(read):
        return read
    counts = np.diff(np.append(first, len(read)))
    times = read[first]
    before = np.concatenate([[times[0] - (np.median(np.diff(times)) if len(times) > 1 else 0.0)], times[:-1]])
    group = np.repeat(np.arange(len(first)), counts)
    rank = np.arange(len(read)) - first[group] + 1
    return before[group] + (times[group] - before[group]) * rank / counts[group]


def _ordered(stream, t):
    """The stream's rows in time order, keeping only rows later than every row before them."""
    order = np.argsort(t, kind="stable")
    t = t[order]
    keep = np.concatenate([[True], t[1:] > np.maximum.accumulate(t)[:-1]])
    out = {name: values[order][keep] for name, values in stream.items()}
    out["t"] = t[keep]
    return out


def _stream(d, prefix):
    if f"{prefix}_t" not in d or len(d[f"{prefix}_t"]) < 2:
        return None
    n = len(d[f"{prefix}_t"])
    return {key[len(prefix) + 1:]: d[key] for key in d if key.startswith(prefix + "_") and len(d[key]) == n}


def load(path):
    """A recording as {"meta", "state", "sport", "cmd", "timing"}. sport and cmd are None if absent.

    Each stream is a dict of arrays, with "t" in seconds counted from the first
    rt/lowstate. Each message's time comes from the best clock it carries: the robot's
    tick for rt/lowstate, sport mode's stamp for rt/sportmodestate, then the time DDS's
    writer sent it. Reads this script's recordings and go2/watch.py's.
    """
    try:
        with np.load(path, allow_pickle=False) as file:
            d = {key: file[key] for key in file.files}
    except (OSError, ValueError) as error:
        raise Problem(f"can't read {path} as a recording: {error}") from error
    meta = json.loads(str(d.pop("meta"))) if "meta" in d else {}
    state = _stream(d, "state")
    if state is None or "tick" not in state or "q" not in state:
        raise Problem(f"{path} has no rt/lowstate messages. It needs a recording made by "
                      f"`go2/record_walk.py record` or go2/watch.py.")
    if "remote" not in state and "wireless_remote" in state:  # a go2/watch.py recording
        raw = np.ascontiguousarray(state["wireless_remote"].astype(np.uint8))
        lx, rx, ry, _, ly = np.frombuffer(raw[:, 4:24].tobytes(), dtype="<f4").reshape(-1, 5).T
        state["remote"] = np.column_stack([lx, ly, rx, ry])

    read = state["t"].astype(float)
    clock = t = None
    for name, stamp in (("the robot's tick", state["tick"] / 1000.0), ("DDS's send times", state.get("sent"))):
        if stamp is not None:
            t, late = robot_times(read, stamp.astype(float))
            if t is not None:
                clock = name
                break
    if t is None:  # nothing from the sender's side to go by
        t, late = spread(read), np.zeros(len(read))
    start = t[0]
    collections = np.flatnonzero(np.diff(read, prepend=-np.inf) > 0)
    behind = np.minimum.reduceat(late, collections)  # the freshest state in each collection

    def times(stream, *stamps):
        """When each of the stream's messages was sent, counted from the first rt/lowstate."""
        read_at = stream["t"].astype(float)
        for name in stamps:
            if name in stream:
                sent = robot_times(read_at, stream[name].astype(float))[0]
                if sent is not None:
                    return sent - start
        # No clock of the sender's on these messages. Go by when they got here, less how
        # far behind the program was then, going by the states read with them. That
        # explained the motors' torque to R2 0.99 on recordings read up to 2.4 s late.
        at = spread(read_at)
        return at - np.interp(at, read[collections], behind) - start

    sport, cmd = _stream(d, "sport"), _stream(d, "cmd")
    if sport is not None:
        sport = _ordered(sport, times(sport, "stamp", "sent"))
    if cmd is not None:
        cmd = _ordered(cmd, times(cmd, "sent"))
    state["late"] = late
    state = _ordered(state, t - start)

    seconds = float(state["t"][-1])
    timing = {"clock": clock, "seconds": seconds, "late_median": float(np.median(late)),
              "late_max": float(late.max()), "longest_gap": float(np.diff(state["t"]).max()), "lost": None}
    if clock:
        period = max(float(np.median(np.diff(state["t"]))), STATE_PERIOD)
        timing["lost"] = float(max(0.0, 1.0 - len(state["t"]) / (seconds / period + 1)))
    return {"path": Path(path), "meta": meta, "state": state, "sport": sport, "cmd": cmd, "timing": timing}


# --------------------------------------------------------------------------- rebuilding the motion


def yaw_of(quat):
    w, x, y, z = np.asarray(quat, dtype=float).T
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def turned(quat, angle):
    """quat (N, 4 as w, x, y, z) turned by angle about the world's vertical."""
    c, s = np.cos(angle / 2.0), np.sin(angle / 2.0)
    w, x, y, z = quat.T
    return np.column_stack([c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w])


def resample(t, source_t, values):
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        return np.interp(t, source_t, values)
    return np.column_stack([np.interp(t, source_t, column) for column in values.T])


def resample_quat(t, source_t, quat):
    """Quaternions at times t. The samples are a few ms apart, so interpolating each part is enough."""
    quat = np.asarray(quat, dtype=float)
    opposite = np.sum(quat[1:] * quat[:-1], axis=1) < 0  # q and -q are the same attitude
    flip = np.cumprod(np.concatenate([[1.0], np.where(opposite, -1.0, 1.0)]))
    out = resample(t, source_t, quat * flip[:, None])
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def holes(t, source_t):
    """For each time, the gap between the recorded messages either side of it, in s."""
    i = np.clip(np.searchsorted(source_t, t), 1, len(source_t) - 1)
    gap = source_t[i] - source_t[i - 1]
    outside = np.maximum(source_t[0] - t, t - source_t[-1])
    return np.maximum(gap, outside)


def lowest_foot(q, quat):
    """How far the lowest foot's sole is below the base, for each row, in m."""
    return np.array([-(foot_positions(qi) @ rotation(*wi).T)[:, 2].min() + FOOT_RADIUS for qi, wi in zip(q, quat)])


def leg_odometry(state, t):
    """Body position and velocity at times t from the legs and IMU alone: go2/state_estimator.py.

    Returns (position, velocity along the body's own axes). The estimator's world frame
    has x along the heading at t[0], so the position is already in the rebuilt frame.
    """
    lead = max(t[0] - ODOMETRY_LEAD, state["t"][0])
    fine = np.arange(lead, t[-1] + 1.0 / ODOMETRY_HZ, 1.0 / ODOMETRY_HZ)
    q, dq = resample(fine, state["t"], state["q"]), resample(fine, state["t"], state["dq"])
    quat = resample_quat(fine, state["t"], state["quaternion"])
    gyro, accel = resample(fine, state["t"], state["gyro"]), resample(fine, state["t"], state["accel"])
    force = resample(fine, state["t"], state["foot_force"])
    start_yaw = float(yaw_of(quat[np.searchsorted(fine, t[0])]))
    estimator = BaseEstimator(start_yaw=start_yaw)
    velocity, height = np.zeros((len(fine), 3)), np.zeros(len(fine))
    for i in range(len(fine)):
        velocity[i], height[i] = estimator.update(fine[i], q[i], dq[i], quat[i], gyro[i], accel[i], force[i])
    walked = np.cumsum(velocity[:, :2], axis=0) / ODOMETRY_HZ
    walked -= resample(t[:1], fine, walked)
    position = np.column_stack([resample(t, fine, walked), resample(t, fine, height)])
    heading = yaw_of(resample_quat(t, fine, quat)) - start_yaw
    v = resample(t, fine, velocity)
    c, s = np.cos(heading), np.sin(heading)
    return position, np.column_stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1], v[:, 2]])


def reconstruct(rec, rate, start=0.0, end=None):
    """The recorded motion every 1/rate s, from start to end (s into the recording).

    Returns a dict of arrays with one row per time: t (from 0), position (m) and quat
    (w, x, y, z) of the base in a frame that starts at the origin facing +x, q and dq in
    the SDK's joint order, velocity along the body's own axes and yaw_rate, and
    foot_force. cmd_q, kp, kd and remote are there if they were recorded. "notes" says
    how the body's position was found and what was interpolated.
    """
    state, sport, cmd = rec["state"], rec["sport"], rec["cmd"]
    last = state["t"][-1]
    end = last if end is None else min(end, last)
    if not 0.0 <= start < end:
        raise Problem(f"there's nothing between {start:g} s and {end:g} s: the recording is {last:.1f} s long")
    use_sport = sport is not None and sport["t"][0] < end and sport["t"][-1] > start
    if use_sport:  # the topics start and stop a few ms apart: keep to where both have data
        if 0.0 < sport["t"][0] - start < GAP:
            start = float(sport["t"][0])
        if 0.0 < end - sport["t"][-1] < GAP:
            end = float(sport["t"][-1])
    t = np.arange(start, end + 0.5 / rate, 1.0 / rate)  # the end too, when it falls on a sample
    t = t[t <= end + 1e-9]
    if len(t) < 2:
        raise Problem(f"{end - start:g} s at {rate:g} Hz is less than two samples")
    notes = []
    out = {"t": t - start, "start": start, "rate": rate, "notes": notes,
           "q": resample(t, state["t"], state["q"]), "dq": resample(t, state["t"], state["dq"]),
           "foot_force": resample(t, state["t"], state["foot_force"])}
    gap = holes(t, state["t"])
    if gap.max() > GAP:
        notes.append(f"{np.mean(gap > GAP):.1%} of it is interpolated across holes in the joint data of up to "
                     f"{gap.max() * 1000:.0f} ms")

    if use_sport:
        position = resample(t, sport["t"], sport["position"])
        quat = sport["quaternion"] if np.abs(sport["quaternion"]).sum() > 0 else None
        if quat is not None:  # sport mode's own attitude: its yaw is the one its position goes with
            quat = resample_quat(t, sport["t"], quat)
        else:
            quat = resample_quat(t, state["t"], state["quaternion"])
        velocity = resample(t, sport["t"], sport["velocity"])
        yaw_rate = resample(t, sport["t"], sport["yaw_speed"])
        gap = holes(t, sport["t"])
        if gap.max() > GAP:
            notes.append(f"{np.mean(gap > GAP):.1%} of it is interpolated across holes in the body position of "
                         f"up to {gap.max() * 1000:.0f} ms")
        # Unitree doesn't say which point of the body its height is for, so set it from the
        # feet: one shift for the whole stretch that puts the lowest foot on the floor.
        some = np.linspace(0, len(t) - 1, min(len(t), 2000)).astype(int)
        needed = lowest_foot(out["q"][some], quat[some]) - position[some, 2]
        shift = float(np.median(needed))
        position[:, 2] += shift
        notes.append(f"body position from sport mode; height shifted {shift * 1000:+.0f} mm to stand the feet on "
                     f"the floor (they stay within {np.sqrt(np.mean((needed - shift) ** 2)) * 1000:.0f} mm rms of it)")
        yaw0 = float(yaw_of(quat[0]))
        c, s = np.cos(yaw0), np.sin(yaw0)
        xy = position[:, :2] - position[0, :2]
        position[:, :2] = np.column_stack([c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]])
    else:
        quat = resample_quat(t, state["t"], state["quaternion"])
        yaw0 = float(yaw_of(quat[0]))
        position, velocity = leg_odometry(state, t)
        yaw_rate = resample(t, state["t"], state["gyro"])[:, 2]
        notes.append("no rt/sportmodestate here, so the body's position is worked out from the legs and IMU "
                     "(go2/state_estimator.py). On the one walk that was compared with sport mode's position, "
                     "2026-11-19, it read 9-13% long")
    out.update(position=position, quat=turned(quat, -yaw0), velocity=velocity, yaw_rate=yaw_rate)

    if cmd is not None and cmd["t"][0] < end and cmd["t"][-1] > start:
        inside = (cmd["t"] >= start) & (cmd["t"] <= end)
        out.update(cmd_q=resample(t, cmd["t"], cmd["q"]), kp=float(np.median(cmd["kp"][inside])) if inside.any() else 0.0,
                   kd=float(np.median(cmd["kd"][inside])) if inside.any() else 0.0)
        position_targets = (cmd["kp"][inside] > 0).all(axis=1) & (np.abs(cmd["q"][inside]) < 10.0).all(axis=1)
        out["cmd_share"] = float(position_targets.mean()) if inside.any() else 0.0
        out["cmd_hole"] = float(holes(t, cmd["t"]).max())
    if "remote" in state:
        out["remote"] = resample(t, state["t"], state["remote"])
    return out


def moving_stretches(rec, rate=20.0):
    """[(start, end)] in s: when the robot was walking or turning, going by sport mode's own
    speed, or by the joints if there's none."""
    state, sport = rec["state"], rec["sport"]
    t = np.arange(0.0, state["t"][-1], 1.0 / rate)
    if len(t) < 2:
        return []
    half = np.ones(int(rate * 0.5)) / int(rate * 0.5)
    if sport is not None:
        speed = np.hypot(*resample(t, sport["t"], sport["velocity"])[:, :2].T)
        turn = np.abs(resample(t, sport["t"], sport["yaw_speed"]))
        moving = (np.convolve(speed, half, "same") > MOVING_SPEED) | (np.convolve(turn, half, "same") > MOVING_TURN)
        moving &= holes(t, sport["t"]) < 1.0
    else:
        joints = np.sqrt(np.mean(resample(t, state["t"], state["dq"]) ** 2, axis=1))
        moving = np.convolve(joints, half, "same") > MOVING_JOINTS
    edges = np.flatnonzero(np.diff(np.concatenate([[False], moving, [False]]).astype(int)))
    stretches = []
    for a, b in zip(edges[::2], edges[1::2]):
        a, b = float(t[a]), float(t[min(b, len(t) - 1)])
        if stretches and a - stretches[-1][1] < MOVING_MERGE:
            stretches[-1][1] = b
        else:
            stretches.append([a, b])
    return [(a, b) for a, b in stretches if b - a >= MOVING_MIN]


def path_summary(motion):
    """Distance along the path, displacement forward and left of the start heading, and the turn."""
    position = motion["position"]
    step = max(1, int(round(motion["rate"] / 10.0)))  # 10 Hz, so noise doesn't add up as distance
    xy = position[::step, :2]
    yaw = np.unwrap(yaw_of(motion["quat"]))
    return (float(np.sum(np.hypot(*np.diff(xy, axis=0).T))), float(position[-1, 0]), float(position[-1, 1]),
            float(yaw[-1] - yaw[0]))


def print_info(rec):
    meta, timing, state, sport, cmd = rec["meta"], rec["timing"], rec["state"], rec["sport"], rec["cmd"]
    seconds = timing["seconds"]
    made_by = "go2/record_walk.py" if meta.get("kind") == "record_walk" else "go2/watch.py"
    when = f", {meta['started_wall']}" if meta.get("started_wall") else ""
    print(f"Recording: {rec['path']}  ({made_by}{when})")
    if meta.get("note"):
        print(f"  Note: {meta['note']}")
    lost = "" if timing["lost"] is None else f", {timing['lost']:.1%} lost"
    print(f"  Length {seconds:.1f} s. rt/lowstate {len(state['t'])} messages ({len(state['t']) / seconds:.0f} Hz{lost}, "
          f"longest hole {timing['longest_gap'] * 1000:.0f} ms)")
    for name, stream in (("rt/sportmodestate", sport), ("rt/lowcmd", cmd)):
        if stream is None:
            print(f"  {name}: none")
        else:
            span = max(stream["t"][-1] - stream["t"][0], 1e-9)
            print(f"  {name} {len(stream['t'])} messages ({len(stream['t']) / span:.0f} Hz, "
                  f"from {stream['t'][0]:.1f} to {stream['t'][-1]:.1f} s)")
    if timing["clock"] and timing["late_max"] > 1e-4:
        print(f"  Timing: from {timing['clock']}. Messages were read {timing['late_median'] * 1000:.0f} ms after "
              f"they were sent at the median, {timing['late_max'] * 1000:.0f} ms at worst.")
    elif timing["clock"]:
        print(f"  Timing: from {timing['clock']}.")
    else:
        print("  Timing: from when messages were read, because they carry no clock that advances.")
    if cmd is not None:
        targets = (cmd["kp"] > 0).all(axis=1)
        if targets.any():
            print(f"  Joint commands: position targets with kp {np.median(cmd['kp'][targets]):g}, "
                  f"kd {np.median(cmd['kd'][targets]):g}"
                  + ("" if targets.all() else f" in {targets.mean():.0%} of the messages"))
    if "remote" in state and np.abs(state["remote"]).max() > 0.05:
        used = [name for name, column in zip(("sideways", "forward", "turn"), state["remote"].T) if np.abs(column).max() > 0.05]
        print(f"  Remote: sticks used for {', '.join(used)}")
    if sport is None:
        print("  No rt/sportmodestate, so there's no recorded body position: sport mode was off. replay and "
              "export work it out from the legs and IMU instead.")
    stretches = moving_stretches(rec)
    whole = reconstruct(rec, 20.0)
    path, forward, left, turn = path_summary(whole)
    print(f"  Whole recording: {path:.2f} m walked, ending {np.hypot(forward, left):.2f} m from the start; "
          f"top speed {np.hypot(*whole['velocity'][:, :2].T).max():.2f} m/s; "
          f"base height {np.median(whole['position'][:, 2]):.3f} m")
    for note in whole["notes"]:
        print(f"    ({note})")
    if not stretches:
        print("  The robot didn't move.")
        return
    print("  Moving (times for --start and --end; forward and left are along the heading at each start):")
    print("       start       end    path  forward    left    turn    speed")
    for a, b in stretches:
        try:
            path, forward, left, turn = path_summary(reconstruct(rec, 20.0, a, b))
        except Problem:
            continue
        print(f"    {a:6.1f} s  {b:6.1f} s  {path:5.2f} m  {forward:+6.2f} m  {left:+5.2f} m  {np.degrees(turn):+5.0f}°  "
              f"{path / (b - a):4.2f} m/s")


def info(args):
    print_info(load(args.recording))
    return 0


# --------------------------------------------------------------------------- replay


def find_scene(given):
    if given:
        if not given.exists():
            raise Problem(f"no scene at {given}")
        return given
    places = [HERE.parents[1] / "unitree_mujoco" / "unitree_robots" / "go2" / "scene.xml"]
    places += [HERE.parents[1] / name / "Robots" / "go2" / "scene.xml" for name in ("S-MICRL", "S-M-ICRL")]
    for place in places:
        if place.exists():
            return place
    raise Problem("no Go2 MuJoCo model found next to this repo. Pass --scene with unitree_mujoco's "
                  "unitree_robots/go2/scene.xml or S-M-ICRL's Robots/go2/scene.xml.")


def replay(args):
    rec = load(args.recording)
    motion = reconstruct(rec, VIDEO_FPS / args.speed, args.start, args.end)
    scene = find_scene(args.scene)
    if sys.platform.startswith("linux"):
        os.environ.setdefault("MUJOCO_GL", "egl")  # offscreen rendering without a display
    try:
        import cv2
        import mujoco
    except ImportError as error:
        raise Problem(f"replay needs MuJoCo and OpenCV, which `go2/setup_env.sh --with-sim` installs: {error}") from error
    model = mujoco.MjModel.from_xml_path(str(scene))
    try:
        base = int(model.jnt_qposadr[model.body("base_link").jntadr[0]])
        joints = [int(model.joint(f"{name}_joint").qposadr[0]) for name in JOINTS]
        floor = model.geom("floor").id
    except KeyError as error:
        raise Problem(f"{scene} isn't a Go2 scene this script knows: {error}") from error
    for geom in range(model.ngeom):  # leave only the floor: the robot didn't walk among the scene's boxes
        if model.geom_bodyid[geom] == 0 and geom != floor:
            model.geom_rgba[geom, 3] = 0.0
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=VIDEO_SIZE[1], width=VIDEO_SIZE[0])
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = model.body("base_link").id
    camera.distance, camera.azimuth, camera.elevation = 1.6, 135.0, -15.0
    video = args.video or rec["path"].with_suffix(".mp4")
    video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, VIDEO_SIZE)
    if not writer.isOpened():
        raise Problem(f"can't write a video to {video}")
    for i, t in enumerate(motion["t"]):
        data.qpos[base:base + 3] = motion["position"][i]
        data.qpos[base + 3:base + 7] = motion["quat"][i]
        data.qpos[joints] = motion["q"][i]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera)
        frame = np.ascontiguousarray(renderer.render()[:, :, ::-1])  # RGB to BGR
        x, y = motion["position"][i, :2]
        cv2.putText(frame, f"t {motion['start'] + t:6.2f} s   x {x:+.2f} m   y {y:+.2f} m", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        writer.write(frame)
    writer.release()
    path, forward, left, turn = path_summary(motion)
    slow = "" if args.speed == 1.0 else f", played at {args.speed:g}x"
    print(f"Video: {video}  ({len(motion['t'])} frames, {motion['t'][-1]:.1f} s of the recording{slow})")
    print(f"  {path:.2f} m walked: {forward:+.2f} m forward, {left:+.2f} m left, turned {np.degrees(turn):+.0f}°")
    for note in motion["notes"]:
        print(f"  ({note})")
    print(f"  Model: {scene}. The model is posed as recorded; nothing is simulated.")
    return 0


# --------------------------------------------------------------------------- export


def export(args):
    rec = load(args.recording)
    motion = reconstruct(rec, args.rate, args.start, args.end)
    stem = str(rec["path"].with_suffix(""))
    if args.source == "commanded":
        if "cmd_q" not in motion:
            raise Problem("this stretch has no rt/lowcmd, so there are no commanded targets: use --source measured")
        if motion["cmd_share"] < 1.0:
            raise Problem(f"{1.0 - motion['cmd_share']:.0%} of the commands in this stretch aren't position targets "
                          f"(their kp is 0), so they can't be played back: pick another stretch with --start and "
                          f"--end, or use --source measured")
        angles = motion["cmd_q"]
        if motion["cmd_hole"] > GAP:
            motion["notes"].append(f"the commands have holes of up to {motion['cmd_hole'] * 1000:.0f} ms")
    else:
        angles = motion["q"]

    trajectory = args.trajectory or Path(f"{stem}_trajectory.txt")
    trajectory.parent.mkdir(parents=True, exist_ok=True)
    order = [JOINTS.index(name) for name in TRAJECTORY_ORDER]
    np.savetxt(trajectory, angles[:, order], fmt="%.5f", comments="#",
               header=" ".join(f"{name}_joint" for name in TRAJECTORY_ORDER))

    names = (["t", "x", "y", "z", "qw", "qx", "qy", "qz", "yaw"] + [f"{j}_q" for j in JOINTS]
             + [f"{j}_dq" for j in JOINTS] + ["vx_body", "vy_body", "vz_body", "yaw_rate"]
             + [f"{leg}_foot_force" for leg in ("FR", "FL", "RR", "RL")])
    table = [motion["t"], motion["position"], motion["quat"], np.unwrap(yaw_of(motion["quat"])), motion["q"],
             motion["dq"], motion["velocity"], motion["yaw_rate"], motion["foot_force"]]
    if "cmd_q" in motion:
        names += [f"{j}_cmd" for j in JOINTS]
        table.append(motion["cmd_q"])
    if "remote" in motion:
        names += ["stick_lx", "stick_ly", "stick_rx", "stick_ry"]
        table.append(motion["remote"])
    csv = args.csv or Path(f"{stem}_motion.csv")
    csv.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(csv, np.column_stack(table), fmt="%.6g", delimiter=",", header=",".join(names), comments="")

    path, forward, left, turn = path_summary(motion)
    gains = f" --kp {motion['kp']:g} --kd {motion['kd']:g}" if args.source == "commanded" else ""
    command = f"python go2/run_policy.py --trajectory {trajectory} --rate {args.rate:g}{gains}"
    print(f"From {motion['start']:g} s, {motion['t'][-1]:.2f} s long: {path:.2f} m walked, {forward:+.2f} m forward, "
          f"{left:+.2f} m left, turned {np.degrees(turn):+.0f}°")
    for note in motion["notes"]:
        print(f"  ({note})")
    print(f"Joint file: {trajectory}\n  {len(angles)} samples at {args.rate:g} Hz, the {args.source} joint angles, "
          f"columns as its header names them")
    try:  # run_policy.py's own check, when the SDK it imports is installed here
        from run_policy import describe_trajectory, trajectory_problem

        problem = trajectory_problem(angles, args.rate)
        print(f"  {describe_trajectory(angles, args.rate)}")
    except Exception:
        problem = None
        print(f"  Check it against run_policy.py's limits: {command} --check")
    if problem:
        print(f"  run_policy.py would refuse to play it: {problem}. Another stretch or a lower --rate may pass"
              + (", and so may --source measured." if args.source == "commanded" else "."))
    else:
        print(f"  Play it, in the simulator first: {command}")
        if args.source == "commanded":
            print(f"  (kp {motion['kp']:g} and kd {motion['kd']:g} are the gains sport mode sent with these targets.)")
        print("  A playback is open loop: sport mode balanced with feedback, and this doesn't.")
    print(f"CSV: {csv}\n  {len(names)} columns: time, base pose, joint angles and speeds, body velocity, foot force"
          + (", commanded angles" if "cmd_q" in motion else "") + (", sticks" if "remote" in motion else ""))
    return 0


# --------------------------------------------------------------------------- main


def positive(text):
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be above 0")
    return value


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    p = commands.add_parser("record", help="record the robot while you walk it in sport mode (read-only)")
    p.add_argument("--robot", metavar="INTERFACE", nargs="?", const="",
                   help="record the robot, on this network interface (if none is given: check_connection.py's "
                        "default); without --robot, record go2/sim_headless.py")
    p.add_argument("--domain", type=int, help="DDS domain id (default: 0 with --robot, otherwise 1)")
    p.add_argument("--seconds", type=positive, default=MAX_SECONDS,
                   help="stop and save after this long (default: %(default)s); the recording is held in memory "
                        "until then, about 40 MB a minute")
    p.add_argument("--note", help="a line about this walk, kept in the recording")
    p.add_argument("--quiet", action="store_true", help="print warnings only, not the status line")
    p.add_argument("--log", type=Path, help="where to save (default: go2/logs/walks/<time>.npz)")
    p.set_defaults(run=record)

    p = commands.add_parser("info", help="what a recording holds, and when the robot was moving")
    p.add_argument("recording", type=Path)
    p.set_defaults(run=info)

    stretch = argparse.ArgumentParser(add_help=False)
    stretch.add_argument("recording", type=Path)
    stretch.add_argument("--start", type=float, default=0.0, help="s into the recording (default: its start)")
    stretch.add_argument("--end", type=float, help="s into the recording (default: its end)")

    p = commands.add_parser("replay", parents=[stretch], help="a video of the Go2 model going through the motion")
    p.add_argument("--video", type=Path, help="the .mp4 to write (default: next to the recording)")
    p.add_argument("--speed", type=positive, default=1.0, help="playback speed; 0.25 is slow motion (default: 1)")
    p.add_argument("--scene", type=Path, help="the Go2 MuJoCo scene (default: unitree_mujoco's or S-M-ICRL's, "
                                              "next to this repo)")
    p.set_defaults(run=replay)

    p = commands.add_parser("export", parents=[stretch],
                            help="a joint file for run_policy.py --trajectory, and a CSV of the motion")
    p.add_argument("--rate", type=positive, default=50.0, help="samples per second (default: %(default)s)")
    p.add_argument("--source", choices=("measured", "commanded"), default="measured",
                   help="joint angles for the joint file: where the joints were, or the targets sport mode "
                        "sent them (default: %(default)s)")
    p.add_argument("--trajectory", type=Path, help="the joint file (default: <recording>_trajectory.txt)")
    p.add_argument("--csv", type=Path, help="the CSV (default: <recording>_motion.csv)")
    p.set_defaults(run=export)
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        return args.run(args)
    except Problem as problem:
        print(f"Can't: {problem}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
