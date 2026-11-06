#!/usr/bin/env python
"""Test whether the Go2's odometry can score S-M-ICRL's walk reward. READ-ONLY.

This script only subscribes to topics. It creates no publishers and no SDK
clients, so it can't command the robot or change its mode. When a step needs
the robot to move, you run go2/run_policy.py in another terminal or use the remote.

    python go2/check_odom.py record [interface]           # guided test; saves go2/logs/odom_<time>.npz
    python go2/check_odom.py record [interface] --free    # no steps: record until Ctrl-C
    python go2/check_odom.py analyze LOG.npz              # report on a saved recording again
    python go2/check_odom.py record lo --domain 1 --free  # go2/sim_headless.py --lidar

For each topic that might carry the robot's position, the report answers:
  1. Does it keep publishing after run_policy.py releases sport mode?
  2. Does it follow the robot when the walk policy moves it, and when it's moved by hand?
  3. How much walk reward does its noise add while the robot is still?
  4. Can its z and yaw stand in for the simulator's base height and yaw?

It also saves a plot next to the recording. The guided test has five steps, which
go2/README.md describes.

Exit status: 0 = report written, 1 = no robot state or no position topic heard,
2 = bad network interface.
"""
import argparse
import json
import math
import sys
import threading
import time
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import NamedTuple

import numpy as np

from check_connection import DEFAULT_INTERFACE, check_interface
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

LOG_DIR = Path(__file__).resolve().parent / "logs"

# Topics that might carry the robot's position. robot_odom and sportmodestate are
# known to publish with sport mode on. The other two are guesses. A guess costs
# nothing, but no messages can mean either that the topic doesn't exist or that it
# has a different message type.
POSE_TOPICS = {
    "robot_odom": ("rt/utlidar/robot_odom", "Odometry_"),
    "robot_pose": ("rt/utlidar/robot_pose", "PoseStamped_"),
    "sportmodestate": ("rt/sportmodestate", "SportModeState_"),
    "odommodestate": ("rt/odommodestate", "SportModeState_"),
}
POSE_COLUMNS = ("stamp", "x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "body_height")
X, Y, Z, QW, QZ, BODY_HEIGHT = 1, 2, 3, 4, 7, 11

# rt/lowstate arrives at 500 Hz. Every fifth message is plenty for height and yaw.
LOW_STATE_KEEP_EVERY = 5
Q, DQ, QUAT, GYRO, FOOT_FORCE = slice(0, 12), slice(12, 24), slice(24, 28), slice(28, 31), slice(31, 35)

# Leg geometry from S-M-ICRL's Robots/go2/go2.xml, in SDK order FR, FL, RR, RL.
HIP_POSITIONS = np.array([[0.1934, -0.0465, 0.0], [0.1934, 0.0465, 0.0],
                          [-0.1934, -0.0465, 0.0], [-0.1934, 0.0465, 0.0]])
LEG_SIDE = np.array([-1.0, 1.0, -1.0, 1.0])
THIGH_OFFSET = 0.0955  # m, sideways from hip to thigh joint
THIGH_LENGTH = CALF_LENGTH = 0.213  # m
FOOT_RADIUS = 0.022  # m

# The walk reward's distance and velocity terms, from env/reward.py
# compute_dense_walk_reward with the weights in func_config/go2/walk/walk.yaml.
# The simulator samples every loop step at 500 Hz, and an episode is 2000
# rollout steps plus a 500-step hold, so 5 s.
W_FORWARD, W_VELOCITY, W_BACKWARD, W_SIDEWAYS = 5.0, 4.0, 2.0, 2.0
SIM_LOOP_HZ = 500.0
EPISODE_S = 5.0
FILTER_WIDTHS = (0.0, 0.1, 0.25, 0.5)  # s, centred moving averages to try on the noise

STILL_JOINT_SPEED = 0.3  # rad/s
STILL_GYRO = 0.05  # rad/s
MIN_STILL_S = 5.0  # shortest still window found from the joints and gyro
STILL_TRIM_S = 0.5  # dropped at each end of a still window
STILL_DEFAULT_S = 20.0
STANDING_HEIGHT = 0.2  # m; above this the robot is on its legs, so leg kinematics give its height
ENDPOINT_S = 0.5  # positions are averaged over this long at each end of a step
JUMP_M = 0.05  # a step this large between consecutive messages is reported
SPORT_GONE_S = 2.0  # sportmodestate ending this long before the recording counts as sport mode off
FROZEN_M = 0.01  # a source moving less than this while the joints move didn't follow the robot


def quat_yaw(w, x, y, z):
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# Recording ---------------------------------------------------------------------

def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_odometry(msg):
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
    row = [stamp_seconds(msg.header.stamp), p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, math.nan]
    return row, f"{msg.header.frame_id} -> {msg.child_frame_id}"


def read_pose_stamped(msg):
    p, q = msg.pose.position, msg.pose.orientation
    nan = math.nan
    return [stamp_seconds(msg.header.stamp), p.x, p.y, p.z, q.w, q.x, q.y, q.z, nan, nan, nan, nan], msg.header.frame_id


def read_sport_state(msg):
    w, x, y, z = msg.imu_state.quaternion
    return [stamp_seconds(msg.stamp), *msg.position, w, x, y, z, *msg.velocity, msg.body_height], None


def read_low_state(msg):
    motors = msg.motor_state[:12]
    imu = msg.imu_state
    return ([m.q for m in motors] + [m.dq for m in motors] + list(imu.quaternion)
            + list(imu.gyroscope) + list(msg.foot_force)), None


MESSAGE_TYPES = {
    "Odometry_": (Odometry_, read_odometry),
    "PoseStamped_": (PoseStamped_, read_pose_stamped),
    "SportModeState_": (SportModeState_, read_sport_state),
}


class TopicRecorder:
    """Keep a row from each message on one topic, stamped with this computer's clock."""

    def __init__(self, read, keep_every=1):
        self._read = read
        self._keep_every = keep_every
        self._lock = threading.Lock()
        self._times, self._rows = [], []
        self.count = 0
        self.frame = None
        self.errors = Counter()

    def __call__(self, msg):
        now = time.monotonic()
        with self._lock:
            self.count += 1
            if (self.count - 1) % self._keep_every:
                return
        try:
            row, frame = self._read(msg)
        except Exception as error:  # a bad message mustn't kill the DDS callback thread
            with self._lock:
                self.errors[f"{type(error).__name__}: {error}"] += 1
            return
        with self._lock:
            self._times.append(now)
            self._rows.append(np.asarray(row, dtype=float))
            if self.frame is None and frame:
                self.frame = frame

    def snapshot(self, columns):
        with self._lock:
            times, rows, count = list(self._times), list(self._rows), self.count
        rows = np.array(rows) if rows else np.empty((0, columns))
        return np.array(times), rows, count


class Step(NamedTuple):
    label: str
    kind: str  # "still", "move", or "hand" (moved without stepping, so the joints look still)
    before: str
    during: str = ""
    optional: bool = False


STEPS = (
    Step("still, sport mode on", "still",
         "Stand the robot in sport mode, as it is after start-up, with a clear flat lane ahead\n"
         "(about 8 x 4 m, as for run_policy.py). Then stand back. Everything near the robot\n"
         "must stay still, you included, because the lidar sees people."),
    Step("remote walk, sport mode on", "move",
         "Lay a tape line on the floor along the robot's heading, and mark the floor under\n"
         "the middle of its body.",
         "Walk it straight forward about 1 m with the remote, then let it stand still.",
         optional=True),
    Step("policy walk, sport mode off", "move",
         "Lay a tape line along the robot's heading and mark the floor under the middle of\n"
         "its body.",
         "In another terminal, run run_policy.py with --robot and the options that worked\n"
         "before, for example:\n"
         "    python go2/run_policy.py go2/policies/walk_tmnxt_iter17.py --robot --seconds 3\n"
         "It releases sport mode, which this step needs. Wait until it has damped the motors."),
    Step("moved by hand, sport mode off", "hand",
         "Lay a tape line along the robot's heading and mark the floor under the middle of\n"
         "its body.",
         "Slide or carry it about 1 m forward along the line, keeping its heading. Hold the\n"
         "body, not the legs. This shows whether the odometry follows the robot with sport\n"
         "mode off and no legs stepping.",
         optional=True),
    Step("still, sport mode off", "still",
         "Leave the robot lying still, and stand back."),
)


def indent(text):
    return "\n".join("  " + line for line in text.splitlines())


def ask_distance(prompt):
    while True:
        text = input(prompt).strip()
        if not text:
            return None
        try:
            return float(text)
        except ValueError:
            print("  Type a number in metres, or leave it blank.")


def hold_still(seconds):
    end = time.monotonic() + seconds
    next_note = seconds
    while (left := end - time.monotonic()) > 0:
        if left <= next_note:
            print(f"  {math.ceil(left):.0f} s left", flush=True)
            next_note -= 5.0
        time.sleep(0.1)


def run_steps(clock, segments, save, still_seconds):
    print("\nCtrl-C here only stops the recording. It doesn't stop the robot: for that, use B on\n"
          "the remote, or Ctrl-C in run_policy.py's terminal.")
    for number, step in enumerate(STEPS, 1):
        print(f"\nStep {number} of {len(STEPS)}: {step.label}\n{indent(step.before)}")
        segment = {"label": step.label, "kind": step.kind, "forward": None, "left": None}
        if step.kind == "still":
            input(f"Press Enter to start {still_seconds:g} s of stillness. ")
            segment["start"] = clock()
            hold_still(still_seconds)
            segment["end"] = clock()
        else:
            skip = ", or type s and Enter to skip" if step.optional else ""
            if input(f"Press Enter to start{skip}. ").strip().lower() == "s" and step.optional:
                print("  Skipped.")
                continue
            segment["start"] = clock()
            print(indent(step.during))
            input("Press Enter once it's finished and still. ")
            segment["end"] = clock()
            segment["forward"] = ask_distance("  Measured distance forward along the line (m, blank if not measured): ")
            segment["left"] = ask_distance("  Measured distance left of the line (m, right is negative, blank if not measured): ")
        segments.append(segment)
        save()
        print("  Saved.")
    print("\nAll steps done.")


def save_recording(path, recorders, start, segments, meta):
    arrays = {}
    meta["topics"] = {}
    for name, (topic, recorder, columns) in recorders.items():
        times, rows, count = recorder.snapshot(columns)
        arrays[f"{name}_t"] = times - start
        arrays[f"{name}_rows"] = rows
        meta["topics"][name] = {"topic": topic, "count": count, "frame": recorder.frame,
                                "errors": dict(recorder.errors)}
    meta["segments"] = segments
    meta["duration"] = time.monotonic() - start
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, meta=json.dumps(meta), **arrays)


def record(args):
    if not check_interface(args.interface):
        return 2
    ChannelFactoryInitialize(args.domain, args.interface)

    low_state = TopicRecorder(read_low_state, LOW_STATE_KEEP_EVERY)
    recorders = {"lowstate": ("rt/lowstate", low_state, 35)}
    subscribers = [ChannelSubscriber("rt/lowstate", LowState_)]
    for name, (topic, type_name) in POSE_TOPICS.items():
        message_type, read = MESSAGE_TYPES[type_name]
        recorders[name] = (topic, TopicRecorder(read), len(POSE_COLUMNS))
        subscribers.append(ChannelSubscriber(topic, message_type))
    start = time.monotonic()  # before Init, so no message is stamped before the recording starts
    started = datetime.now()
    for subscriber, (_, recorder, _) in zip(subscribers, recorders.values()):
        subscriber.Init(recorder, 10)

    path = args.out or LOG_DIR / f"odom_{started:%Y-%m-%d_%H-%M-%S}.npz"
    segments = []
    meta = {"interface": args.interface, "domain": args.domain, "started": started.isoformat(),
            "low_state_keep_every": LOW_STATE_KEEP_EVERY, "free": args.free}

    def clock():
        return time.monotonic() - start

    def save():
        save_recording(path, recorders, start, segments, meta)

    print(f"Listening on {args.interface}, domain {args.domain} (read-only)...")
    time.sleep(3.0)
    if low_state.count == 0:
        print("ERROR: no rt/lowstate after 3 s. Run check_connection.py first.")
        return 1
    for topic, recorder, _ in recorders.values():
        print(f"  {topic:24s} {recorder.count / clock():6.1f} Hz")

    try:
        if args.free:
            print(f"\nRecording until Ctrl-C (at most {args.seconds:g} s)...")
            while clock() < args.seconds:
                time.sleep(0.2)
        else:
            run_steps(clock, segments, save, args.still_seconds)
    except (KeyboardInterrupt, EOFError):
        print("\nStopped recording.")
    save()
    print(f"Saved {path}\n")
    return analyze(path, make_plot=not args.no_plot)


# Analysis ----------------------------------------------------------------------

class Track(NamedTuple):
    """A topic's positions, in the frame it reports them in."""
    name: str
    topic: str
    t: np.ndarray
    xyz: np.ndarray
    yaw: np.ndarray
    body_height: np.ndarray


class LowState(NamedTuple):
    t: np.ndarray
    joint_speed: np.ndarray  # fastest joint, rad/s
    turn_rate: np.ndarray  # gyro magnitude, rad/s
    yaw: np.ndarray  # unwrapped, rad
    height: np.ndarray  # base height from leg kinematics, m


def feet_in_base(q):
    """Foot centres in the base frame, as (N, 4, 3), from (N, 12) joint angles."""
    legs = q.reshape(-1, 4, 3)
    hip, thigh, calf = legs[..., 0], legs[..., 1], legs[..., 2]
    x = -(THIGH_LENGTH * np.sin(thigh) + CALF_LENGTH * np.sin(thigh + calf))
    z = -(THIGH_LENGTH * np.cos(thigh) + CALF_LENGTH * np.cos(thigh + calf))
    y = LEG_SIDE * THIGH_OFFSET
    return HIP_POSITIONS + np.stack([x, y * np.cos(hip) - z * np.sin(hip), y * np.sin(hip) + z * np.cos(hip)], -1)


def kinematic_height(q, quaternion):
    """Height of the base origin above the two lowest feet, which is the simulator's z on flat ground."""
    w, x, y, z = quaternion.T
    up = np.stack([2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)], -1)
    heights = -np.einsum("nfk,nk->nf", feet_in_base(q), up)
    return np.sort(heights, axis=1)[:, -2:].mean(axis=1) + FOOT_RADIUS


def load(path):
    with np.load(path) as data:
        meta = json.loads(str(data["meta"]))
        arrays = {key: data[key] for key in data.files if key != "meta"}
    rows = arrays["lowstate_rows"]
    if len(rows):
        quaternion = rows[:, QUAT]
        low = LowState(
            t=arrays["lowstate_t"],
            joint_speed=np.abs(rows[:, DQ]).max(axis=1),
            turn_rate=np.linalg.norm(rows[:, GYRO], axis=1),
            yaw=np.unwrap(quat_yaw(*quaternion.T)),
            height=kinematic_height(rows[:, Q], quaternion),
        )
    else:
        low = None
    tracks = {}
    for name, (topic, _) in POSE_TOPICS.items():
        rows = arrays.get(f"{name}_rows")
        if rows is None or not len(rows):
            continue
        tracks[name] = Track(name, topic, arrays[f"{name}_t"], rows[:, X:Z + 1],
                             quat_yaw(*rows[:, QW:QZ + 1].T), rows[:, BODY_HEIGHT])
    return meta, low, tracks


def to_heading_frame(xy, origin, yaw):
    """Forward and left of origin, along heading yaw."""
    d = xy - origin
    c, s = math.cos(yaw), math.sin(yaw)
    return d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c


def runs(mask, t, min_length):
    """(start, end) times of stretches where mask is true for at least min_length seconds."""
    found, begin = [], None
    for i, value in enumerate(mask):
        if value and begin is None:
            begin = i
        if begin is not None and (not value or i == len(mask) - 1):
            end = i if value else i - 1
            if t[end] - t[begin] >= min_length:
                found.append((t[begin], t[end]))
            begin = None
    return found


def still_windows(meta, low, sport_off):
    """Still steps from the guided test, then stretches the joints and gyro show as still."""
    windows = [(s["label"], s["start"] + STILL_TRIM_S, s["end"] - STILL_TRIM_S)
               for s in meta["segments"] if s["kind"] == "still"]
    # Still steps are already in, and a robot moved by hand can look still to its joints.
    excluded = [(s["start"], s["end"]) for s in meta["segments"] if s["kind"] in ("still", "hand")]
    if low is not None:
        still = (low.joint_speed < STILL_JOINT_SPEED) & (low.turn_rate < STILL_GYRO)
        for begin, end in runs(still, low.t, MIN_STILL_S + 2 * STILL_TRIM_S):
            if any(begin < b and a < end for a, b in excluded):
                continue
            inside = (low.t >= begin) & (low.t <= end)
            pose = "standing" if np.median(low.height[inside]) > STANDING_HEIGHT else "lying"
            if sport_off is None:
                sport = ""
            else:
                sport = ", sport on" if end < sport_off else ", sport off" if begin > sport_off else ""
            windows.append((f"found {begin:.0f}-{end:.0f} s, {pose}{sport}",
                            begin + STILL_TRIM_S, end - STILL_TRIM_S))
    return windows


def noise_reward(t, forward, left, width):
    """The walk reward's distance terms per episode for a still robot, as if fully stable."""
    grid = np.arange(t[0], t[-1], 1.0 / SIM_LOOP_HZ)
    f, l = np.interp(grid, t, forward), np.interp(grid, t, left)
    if width > 0:
        kernel = np.ones(max(1, round(width * SIM_LOOP_HZ)))
        kernel /= kernel.size
        if kernel.size >= len(grid):
            return math.nan
        f, l = np.convolve(f, kernel, "valid"), np.convolve(l, kernel, "valid")
    dx, dy, dt = np.diff(f), np.diff(l), 1.0 / SIM_LOOP_HZ
    gained = (W_FORWARD * np.maximum(dx, 0.0)
              + W_VELOCITY * np.minimum(np.maximum(dx, 0.0) / dt, 1.0) * dt
              - W_BACKWARD * np.maximum(-dx, 0.0)
              - W_SIDEWAYS * np.abs(dy)).sum()
    return gained / (len(dx) * dt) * EPISODE_S


def pose_at(track, t):
    """Median position over ENDPOINT_S before t, and the yaw nearest t; None without data."""
    near = (track.t >= t - ENDPOINT_S) & (track.t <= t)
    if not near.any():
        nearest = np.argmin(np.abs(track.t - t))
        if abs(track.t[nearest] - t) > 1.0:
            return None
        near = np.zeros(len(track.t), bool)
        near[nearest] = True
    last = np.flatnonzero(near)[-1]
    return np.median(track.xyz[near], axis=0), track.yaw[last]


def sport_off_time(meta, tracks):
    sport = tracks.get("sportmodestate")
    if sport is not None and sport.t[-1] < meta["duration"] - SPORT_GONE_S:
        return sport.t[-1]
    return None


def rate(times, begin, end):
    inside = times[(times >= begin) & (times <= end)]
    return len(inside) / (end - begin) if end > begin else math.nan


def report_topics(meta, tracks, sport_off):
    duration = meta["duration"]
    print(f"Topics (recorded {duration:.0f} s on {meta['interface']})")
    for name, info in meta["topics"].items():
        line = f"  {info['topic']:24s}"
        if info["count"] == 0:
            kind = POSE_TOPICS.get(name, ("", "LowState_"))[1]
            print(f"{line} no messages: absent, or its type isn't {kind}")
            continue
        line += f" {info['count']:7d} msgs"
        track = tracks.get(name)
        if track is not None:
            gaps = np.diff(track.t)
            line += f"  {track.t[0]:5.1f}-{track.t[-1]:5.1f} s  longest gap {gaps.max() if len(gaps) else 0:.2f} s"
            if track.t[-1] < duration - SPORT_GONE_S:
                line += "  STOPPED"
        if info["frame"]:
            line += f"  frame {info['frame']}"
        print(line)
        for error, count in info["errors"].items():
            print(f"    {count} unreadable: {error}")
    if sport_off is not None:
        print(f"\nrt/sportmodestate stopped at {sport_off:.1f} s, which is taken as when sport mode was released.")
    elif "sportmodestate" in tracks:
        print("\nrt/sportmodestate published throughout, so sport mode was never seen going off. "
              "(The simulator always publishes it.)")
    else:
        print("\nrt/sportmodestate never published, so when sport mode went off is unknown.")


def report_publishing(meta, tracks, sport_off):
    print("\n1. Publishing after sport mode was released")
    if sport_off is None:
        print("  Unknown, because sport mode wasn't seen going off.")
        return
    for track in tracks.values():
        if track.name == "sportmodestate":
            continue
        before = rate(track.t, max(track.t[0], sport_off - 30.0), sport_off)
        after = rate(track.t, sport_off, meta["duration"])
        if after == 0:
            verdict = f"STOPPED at {track.t[-1]:.1f} s"
        else:
            verdict = f"kept publishing, until {track.t[-1]:.1f} s"
        print(f"  {track.name:15s} {before:6.1f} Hz before, {after:6.1f} Hz after: {verdict}")
        steps = np.linalg.norm(np.diff(track.xyz, axis=0), axis=1)
        for i in np.argsort(steps)[::-1][:3]:
            if steps[i] < JUMP_M:
                break
            print(f"    jumped {steps[i]:.2f} m at {track.t[i + 1]:.1f} s, "
                  f"over a {track.t[i + 1] - track.t[i]:.2f} s gap")


def report_following(meta, low, tracks):
    print("\n2. Following the robot (forward and left of each step's starting heading)")
    moves = [s for s in meta["segments"] if s["kind"] != "still"]
    if not moves:
        print("  No move steps were recorded, so this compares the start and end of the recording.")
        moves = [{"label": "whole recording", "start": ENDPOINT_S, "end": meta["duration"],
                  "forward": None, "left": None}]
    for segment in moves:
        start, end = segment["start"], segment["end"]
        measured = ", ".join(f"{segment[key]:+.2f} m {key}" for key in ("forward", "left") if segment[key] is not None)
        print(f"  {segment['label']} ({start:.0f}-{end:.0f} s): measured {measured or 'nothing'}")
        if low is not None:
            during = (low.t >= start) & (low.t <= end)
            joints_moved = during.any() and low.joint_speed[during].max() > 1.0
            imu_turn = math.degrees(np.interp(end, low.t, low.yaw) - np.interp(start, low.t, low.yaw))
            print(f"    {'IMU':15s} {'':28s} yaw {imu_turn:+6.1f} deg")
        else:
            joints_moved = False
        for track in tracks.values():
            first, last = pose_at(track, start), pose_at(track, end)
            if first is None or last is None:
                print(f"    {track.name:15s} no data at the start or end")
                continue
            forward, left = to_heading_frame(last[0][None, :2], first[0][:2], first[1])
            forward, left = float(forward[0]), float(left[0])
            turn = math.degrees(wrap(last[1] - first[1]))
            line = f"    {track.name:15s} {forward:+6.2f} m forward {left:+6.2f} m left  yaw {turn:+6.1f} deg"
            errors = [abs(value - segment[key]) for key, value in (("forward", forward), ("left", left))
                      if segment[key] is not None]
            if errors:
                line += f"  off by {math.hypot(*errors) if len(errors) == 2 else errors[0]:.2f} m"
            inside = (track.t >= start) & (track.t <= end)
            span = np.ptp(track.xyz[inside, :2], axis=0).max() if inside.sum() > 1 else 0.0
            if joints_moved and span < FROZEN_M:
                line += "  DIDN'T MOVE"
            print(line)


def report_noise(meta, low, tracks, sport_off):
    widths = "  ".join(f"{'raw' if w == 0 else f'{w:g} s':>6s}" for w in FILTER_WIDTHS)
    print(f"\n3. Walk reward added by noise while still, per {EPISODE_S:g} s episode "
          f"(walking 1 m earns {W_FORWARD + W_VELOCITY:g})")
    print(f"  {'window':34s} {'source':15s} {'spread':>8s} {'drift':>9s}  {widths}")
    windows = still_windows(meta, low, sport_off)
    if not windows:
        print("  No still windows were recorded.")
    for label, begin, end in windows:
        for track in tracks.values():
            inside = (track.t >= begin) & (track.t <= end)
            if inside.sum() < 10:
                continue
            t, xy = track.t[inside], track.xyz[inside, :2]
            forward, left = to_heading_frame(xy, xy[0], track.yaw[inside][0])
            fit = np.polyfit(t - t[0], np.column_stack([forward, left]), 1)
            residual = np.column_stack([forward, left]) - (np.outer(t - t[0], fit[0]) + fit[1])
            spread = residual.std(axis=0).max()
            drift = np.hypot(*fit[0])
            rewards = [noise_reward(t, forward, left, w) for w in FILTER_WIDTHS]
            values = "  ".join(f"{r:+6.2f}" for r in rewards)
            print(f"  {label[:34]:34s} {track.name:15s} {spread * 1000:6.1f}mm {drift * 1000:5.1f}mm/s  {values}")
    print("  Spread is the noise around a straight-line fit, and drift is that line's slope. The\n"
          "  reward assumes the stability gate is fully open, as for a robot standing well.")


def report_height_and_yaw(low, tracks):
    print("\n4. Height and yaw")
    if low is None:
        print("  No rt/lowstate, so nothing to compare with.")
        return
    standing = low.height > STANDING_HEIGHT
    if standing.any():
        print(f"  Leg kinematics, while on its legs: base height median {np.median(low.height[standing]):.3f} m, "
              f"range {low.height[standing].min():.3f}-{low.height[standing].max():.3f} m")
    for track in tracks.values():
        kinematic = np.interp(track.t, low.t, low.height)
        on_legs = kinematic > STANDING_HEIGHT
        for label, values in (("z", track.xyz[:, 2]), ("body_height", track.body_height)):
            ok = on_legs & np.isfinite(values)
            if ok.sum() < 10:
                continue
            difference = values[ok] - kinematic[ok]
            print(f"  {track.name:15s} {label:11s} minus leg kinematics: offset {difference.mean():+.3f} m, "
                  f"std {difference.std() * 100:.1f} cm, worst {np.abs(difference - difference.mean()).max() * 100:.1f} cm")
        imu = np.interp(track.t, low.t, low.yaw)
        own = np.unwrap(track.yaw)
        difference = np.degrees((own - own[0]) - (imu - imu[0]))
        imu_turn, own_turn = math.degrees(imu[-1] - imu[0]), math.degrees(own[-1] - own[0])
        line = (f"  {track.name:15s} yaw minus IMU yaw: worst {np.abs(difference).max():.1f} deg, "
                f"at the end {difference[-1]:+.1f} deg (turned {own_turn:+.0f} vs IMU {imu_turn:+.0f} deg)")
        if abs(imu_turn) > 20 and own_turn * imu_turn < 0:
            line += "  OPPOSITE SIGN"
        print(line)


def save_plot(path, meta, low, tracks, sport_off):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nNo plot: matplotlib isn't installed.")
        return
    figure, panels = plt.subplots(4, 1, figsize=(13, 11), sharex=True, constrained_layout=True)
    for track in tracks.values():
        forward, left = to_heading_frame(track.xyz[:, :2], track.xyz[0, :2], track.yaw[0])
        panels[0].plot(track.t, forward, label=track.name)
        panels[1].plot(track.t, left, label=track.name)
        own = np.unwrap(track.yaw)
        panels[2].plot(track.t, np.degrees(own - own[0]), label=track.name)
        if low is not None:
            on_legs = np.interp(track.t, low.t, low.height) > STANDING_HEIGHT
            if on_legs.any():
                offset = np.median(track.xyz[on_legs, 2] - np.interp(track.t[on_legs], low.t, low.height))
                panels[3].plot(track.t, track.xyz[:, 2] - offset, label=f"{track.name} z, shifted {-offset:+.2f} m")
        if np.isfinite(track.body_height).any():
            panels[3].plot(track.t, track.body_height, label=f"{track.name} body_height")
    if low is not None:
        panels[2].plot(low.t, np.degrees(low.yaw - low.yaw[0]), "k--", label="IMU")
        panels[3].plot(low.t, low.height, "k--", label="leg kinematics")
    for segment in meta["segments"]:
        colour = "0.85" if segment["kind"] == "still" else "#cfe3f7"
        for panel in panels:
            panel.axvspan(segment["start"], segment["end"], color=colour, zorder=0)
        panels[0].annotate(segment["label"], (segment["start"], 1.0), xycoords=("data", "axes fraction"),
                           fontsize=8, va="top", rotation=90)
    if sport_off is not None:
        for panel in panels:
            panel.axvline(sport_off, color="red", linestyle=":")
    titles = ("forward of the first heading (m)", "left of the first heading (m)",
              "yaw change (deg)", "base height (m)")
    for panel, title in zip(panels, titles):
        panel.set_ylabel(title)
        panel.legend(fontsize=8, loc="best")
        panel.grid(alpha=0.3)
    panels[-1].set_xlabel("time since the recording started (s); grey = still step, blue = move step, "
                          "red = sport mode off")
    png = path.with_suffix(".png")
    figure.savefig(png, dpi=110)
    print(f"\nPlot: {png}")


def analyze(path, make_plot=True):
    meta, low, tracks = load(path)
    sport_off = sport_off_time(meta, tracks)
    report_topics(meta, tracks, sport_off)
    if low is None:
        print("\nNo rt/lowstate was recorded.")
    if not tracks:
        print("\nNo position topic published, so there's nothing to test.")
        return 1
    report_publishing(meta, tracks, sport_off)
    report_following(meta, low, tracks)
    report_noise(meta, low, tracks, sport_off)
    report_height_and_yaw(low, tracks)
    if make_plot:
        save_plot(path, meta, low, tracks, sport_off)
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    rec = commands.add_parser("record", help="record the robot's topics, then report")
    rec.add_argument("interface", nargs="?", default=DEFAULT_INTERFACE,
                     help="network interface wired to the robot (default: %(default)s)")
    rec.add_argument("--domain", type=int, default=0, help="DDS domain id (the robot uses 0, the simulator 1)")
    rec.add_argument("--free", action="store_true", help="skip the guided steps and record until Ctrl-C")
    rec.add_argument("--seconds", type=float, default=900.0, help="longest --free recording (default: %(default)s)")
    rec.add_argument("--still-seconds", type=float, default=STILL_DEFAULT_S,
                     help="length of each still step (default: %(default)s)")
    rec.add_argument("--out", type=Path, help="where to save (default: go2/logs/odom_<time>.npz)")
    rec.add_argument("--no-plot", action="store_true", help="don't save a plot")
    ana = commands.add_parser("analyze", help="report on a saved recording")
    ana.add_argument("log", type=Path)
    ana.add_argument("--no-plot", action="store_true", help="don't save a plot")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.command == "record":
        return record(args)
    return analyze(args.log, make_plot=not args.no_plot)


if __name__ == "__main__":
    sys.exit(main())
