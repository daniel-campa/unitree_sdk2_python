#!/usr/bin/env python
"""Run an S-M-ICRL policy on the Go2 through the Unitree SDK.

This uses the simulator unless you pass --robot. With --robot it MOVES THE ROBOT,
so follow the test order in go2/README.md.

    python go2/run_policy.py POLICY.py                         # simulator
    python go2/run_policy.py POLICY.py --robot --dry-run        # robot, read-only
    python go2/run_policy.py POLICY.py --robot                  # robot
    python go2/run_policy.py POLICY.py --robot enp3s0           # robot, on another interface

The sequence is: stand up, hold, walk for --seconds, return to the home pose, lie
down, then damp the motors. A fault, the remote's B button or Ctrl-C goes straight
to damping. Start the simulator with `python go2/sim_headless.py --flat --dt 0.002`.

POLICY.py is S-M-ICRL code that defines policy(robot_state) and ends with
robot.simulate_rollout(policy). The defaults for --policy-hz, --kp and --kd come
from the run that produced go2/policies/walk_tmnxt_iter17.py. Other policies need
the values from their own run's config.json.

Exit status: 0 = finished, 1 = stopped early, 2 = refused before commanding anything.
"""
import argparse
import builtins
import hashlib
import json
import select
import signal
import sys
import termios
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import NamedTuple

import numpy as np
from check_connection import DEFAULT_INTERFACE, JOINTS, check_interface, state_problems
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# Joint order throughout: FR, FL, RR, RL, each hip / thigh / calf. S-M-ICRL and the
# SDK use the same order.
HOME = np.array([0.0, 0.9, -1.8] * 4)
LIE_DOWN = np.array([0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
                     0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375])
HIP_RANGE, CALF_RANGE = (-1.0472, 1.0472), (-2.7227, -0.83776)
FRONT_THIGH_RANGE, REAR_THIGH_RANGE = (-1.5708, 3.4907), (-0.5236, 4.5379)
JOINT_RANGES = np.array(
    [HIP_RANGE, FRONT_THIGH_RANGE, CALF_RANGE] * 2 + [HIP_RANGE, REAR_THIGH_RANGE, CALF_RANGE] * 2
)
JOINT_LOW, JOINT_HIGH = JOINT_RANGES[:, 0], JOINT_RANGES[:, 1]
TORQUE_LIMIT = np.array([23.7, 23.7, 45.43] * 4)  # N·m, as in the MuJoCo model

# Defaults taken from walk_2026-10-27_17-37-32_TMNXT/config.json.
POLICY_HZ = 5.0
KP, KD = 80.0, 4.0
# Mean base height while walking in S-M-ICRL. On a flat floor the terrain probes
# read the negative of this, and the policy's calf targets depend on it.
PROBE_HEIGHT = 0.262
# What S-M-ICRL reports with nothing overhead or ahead.
OPEN_CLEARANCE, OPEN_OBSTACLE_DISTANCE = 2.0, 10.0

DT = 0.002  # rt/lowcmd period, as in Unitree's examples
STAND_UP_S = 3.0
SETTLE_S = 2.0
RETURN_HOME_S = 1.0
LIE_DOWN_S = 3.0
DAMP_S = 2.0
DAMP_KD = 4.0
GO_TIMEOUT_S = 60.0
MAX_WALK_S = 60.0

# Limits that stop a run. Walking in simulation peaks at about 0.15 rad of tilt and
# 11 rad/s (thigh) and 8 rad/s (calf).
MAX_TILT = 0.5  # rad
MAX_SPEED = np.array([25.0, 25.0, 14.0] * 4)  # rad/s, below Unitree's 30.1 and 15.7
MAX_TEMPERATURE = 75  # °C
JOINT_LIMIT_MARGIN = 0.05  # rad
MAX_STATE_AGE = 0.1  # s
# The policy holds each target for 200 ms anyway, so a shorter stall is harmless.
MAX_LOOP_STALL = 0.2  # s

# Limits for starting to walk.
MAX_STAND_ERROR = 0.2  # rad; kp = 80 sags the rear hips by about 0.12 rad
MAX_STAND_TILT = 0.2  # rad
MAX_STAND_SPEED = 0.5  # rad/s
MIN_BATTERY_PERCENT = 15

STOP_BUTTON_MASK = 1 << 1  # B, in wireless_remote[3]; see example/wireless_controller
POS_STOP, VEL_STOP = 2.146e9, 16000.0  # values that disable a motor's control loops
STAND_DOWN_S = 4.0
RELEASE_ATTEMPTS = 5

LOG_FIELDS = ("t", "stage", "q", "dq", "target", "q_cmd", "tau_est", "temperature", "rpy",
              "gyro", "accel", "foot_force", "power_v", "power_a")

# The same limits S-M-ICRL applies when it runs generated policy code.
POLICY_IMPORTS = ("numpy", "math", "random", "itertools", "functools", "collections", "copy")
POLICY_BUILTINS = {
    name: getattr(builtins, name)
    for name in ("abs", "round", "min", "max", "len", "range", "enumerate", "zip", "sum", "list",
                 "tuple", "dict", "float", "int", "bool", "str", "print", "hasattr", "getattr",
                 "isinstance", "sorted", "any", "all")
}


class Refused(Exception):
    """A reason not to start. Nothing has been commanded."""


class Fault(Exception):
    """A reason to stop a run and damp the motors."""


class StateMonitor:
    """Keep the latest rt/lowstate message and when it arrived."""

    def __init__(self):
        self.msg = None
        self.stamp = 0.0

    def __call__(self, msg):
        self.msg = msg
        self.stamp = time.perf_counter()


class State(NamedTuple):
    q: np.ndarray
    dq: np.ndarray
    tau_est: np.ndarray
    temperature: np.ndarray
    quaternion: np.ndarray
    rpy: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    foot_force: np.ndarray
    power_v: float
    power_a: float
    stop_pressed: bool


def quat_to_rpy(w, x, y, z):
    """Roll, pitch and yaw, using the same ZYX conversion as S-M-ICRL."""
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def read_state(msg):
    motors = msg.motor_state[:12]
    imu = msg.imu_state
    return State(
        q=np.array([motor.q for motor in motors]),
        dq=np.array([motor.dq for motor in motors]),
        tau_est=np.array([motor.tau_est for motor in motors]),
        temperature=np.array([motor.temperature for motor in motors]),
        quaternion=np.array(imu.quaternion),
        rpy=quat_to_rpy(*imu.quaternion),
        gyro=np.array(imu.gyroscope),
        accel=np.array(imu.accelerometer),
        foot_force=np.array(msg.foot_force),
        power_v=msg.power_v,
        power_a=msg.power_a,
        stop_pressed=bool(msg.wireless_remote[3] & STOP_BUTTON_MASK),
    )


def build_robot_state(state, start_yaw):
    """Build S-M-ICRL's 37-element robot_state from what the robot can measure.

    Linear velocity and height need sport mode or an estimator, so they are NaN.
    check_policy refuses policies that use them.
    """
    w, x, y, z = state.quaternion
    roll, pitch, yaw = state.rpy
    yaw_deviation = np.arctan2(np.sin(yaw - start_yaw), np.cos(yaw - start_yaw))
    # Row 3 of the body rotation matrix, which is what S-M-ICRL reads from xmat[6:9].
    up = [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]
    return np.concatenate([
        state.q, state.dq, [roll, pitch, yaw_deviation], state.gyro,
        [np.nan, np.nan, np.nan], [np.nan], up,
    ])


def flat_floor_env(probe_height):
    """S-M-ICRL's env vector on a clear, flat floor: three probes, clearance, obstacle, bias."""
    probe = -probe_height
    return np.array([probe, probe, probe, OPEN_CLEARANCE, OPEN_OBSTACLE_DISTANCE, 1.0])


def policy_target(policy, robot_state, env):
    try:
        gains = np.asarray(policy(robot_state), dtype=float)
    except Exception as error:
        raise Fault(f"policy failed: {error}") from error
    if gains.shape != (12, env.size):
        raise Fault(f"policy returned shape {gains.shape}, expected (12, {env.size})")
    if not np.isfinite(gains).all():
        raise Fault("policy returned non-finite values")
    return np.clip(gains @ env, JOINT_LOW, JOINT_HIGH)


def limit_for_torque(target, q, dq, kp, kd, torque_cap):
    """Move target just enough that kp * (q_cmd - q) - kd * dq stays within torque_cap.

    The cap is approximate, because the motor applies this to a newer q and dq. In
    simulation, with state about 2 ms old, a 14.2 N·m thigh cap peaked at 20.8 N·m.
    """
    return np.clip(target, q + (kd * dq - torque_cap) / kp, q + (kd * dq + torque_cap) / kp)


def safety_problem(state, check_range):
    """Return why this state should stop the robot, or None."""
    if state.stop_pressed:
        return "stop button pressed"
    if not (np.isfinite(state.q).all() and np.isfinite(state.dq).all() and np.isfinite(state.rpy).all()):
        return "non-finite joint or IMU reading"
    tilt = np.abs(state.rpy[:2]).max()
    if tilt > MAX_TILT:
        return f"tilt {tilt:.2f} rad is above {MAX_TILT} rad"
    if state.temperature.max() >= MAX_TEMPERATURE:
        joint = JOINTS[int(state.temperature.argmax())]
        return f"{joint} is at {state.temperature.max()} °C, limit {MAX_TEMPERATURE} °C"
    fast = np.abs(state.dq) > MAX_SPEED
    if fast.any():
        i = int(fast.argmax())
        return f"{JOINTS[i]} speed {state.dq[i]:+.1f} rad/s is above {MAX_SPEED[i]} rad/s"
    if check_range:
        outside = (state.q < JOINT_LOW + JOINT_LIMIT_MARGIN) | (state.q > JOINT_HIGH - JOINT_LIMIT_MARGIN)
        if outside.any():
            i = int(outside.argmax())
            return f"{JOINTS[i]} at {state.q[i]:+.2f} rad is at its joint limit"
    return None


def _policy_import(name, globals=None, locals=None, fromlist=(), level=0):
    if level != 0 or not any(name == m or name.startswith(m + ".") for m in POLICY_IMPORTS):
        raise ImportError(f"module {name!r} is not available to policy code")
    return builtins.__import__(name, globals, locals, fromlist, level)


class _PolicyCatcher:
    """Stands in for S-M-ICRL's `robot`, which the policy file hands its policy to."""

    def __init__(self):
        self.policy = None

    def simulate_rollout(self, policy):
        self.policy = policy


def load_policy(path):
    """Run the policy file with S-M-ICRL's restrictions and return its policy function."""
    catcher = _PolicyCatcher()
    namespace = {
        "__builtins__": {**POLICY_BUILTINS, "__import__": _policy_import},
        "robot": catcher,
        "np": np,
    }
    exec(compile(Path(path).read_text(), str(path), "exec"), namespace)
    if not callable(catcher.policy):
        raise ValueError("the file did not call robot.simulate_rollout(policy)")
    return catcher.policy


def check_policy(policy, env):
    """Refuse a policy that fails, or that depends on state the robot can't provide."""
    for thigh in (0.7, 0.9, 1.1):
        q = HOME.copy()
        q[1::3] = thigh
        robot_state = np.concatenate([q, np.zeros(18), [np.nan] * 4, [0.0, 0.0, 1.0]])
        reference = policy_target(policy, robot_state, env)
        for height, speed in ((0.1, -1.0), (0.4, 1.0)):
            probe = robot_state.copy()
            probe[30:33], probe[33] = speed, height
            if not np.array_equal(policy_target(policy, probe, env), reference):
                raise Fault(
                    "its output changes with body height or linear velocity (robot_state[30:34]), "
                    "or between identical calls. The robot can't provide those values, and "
                    "this script doesn't support a policy that keeps its own state."
                )


def prepare_policy(path, env):
    """Return a checked policy and the file's hash. A stalling policy trips the watchdog."""
    arm_watchdog(5.0)
    try:
        check_policy(load_policy(path), env)
        policy = load_policy(path)  # a fresh copy, unaffected by the checks above
    except Exception as error:
        raise Refused(f"can't use policy {path}: {error}") from error
    finally:
        arm_watchdog(0.0)
    return policy, hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arm_watchdog(seconds):
    """Raise Fault in the main thread after this long unless re-armed. 0 disarms it."""
    signal.setitimer(signal.ITIMER_REAL, seconds)


def _on_watchdog(signum, frame):
    raise Fault("control loop stalled")


def _on_terminate(signum, frame):
    raise KeyboardInterrupt


def format_legs(values):
    legs = [" ".join(f"{v:+.2f}" for v in values[i:i + 3]) for i in range(0, 12, 3)]
    return " | ".join(f"{name} {leg}" for name, leg in zip(("FR", "FL", "RR", "RL"), legs))


def dry_run(monitor, policy, env, policy_hz, seconds):
    """Print what the policy would command. This creates no publisher or SDK client."""
    print("Dry run: nothing is published, so this can't move the robot.", flush=True)
    start = time.perf_counter()
    start_yaw = read_state(monitor.msg).rpy[2]
    calls = 0
    while True:
        elapsed = time.perf_counter() - start
        if elapsed >= seconds:
            return
        age = time.perf_counter() - monitor.stamp
        if age > MAX_STATE_AGE:
            raise Fault(f"no robot state for {age * 1000:.0f} ms")
        state = read_state(monitor.msg)
        target = policy_target(policy, build_robot_state(state, start_yaw), env)
        if calls % max(1, round(policy_hz)) == 0:
            roll, pitch, _ = state.rpy
            print(f"t={elapsed:5.1f} s  roll={roll:+.2f}  pitch={pitch:+.2f}  "
                  f"B button={'pressed' if state.stop_pressed else 'not pressed'}\n"
                  f"  joints {format_legs(state.q)}\n"
                  f"  target {format_legs(target)}", flush=True)
        calls = int(elapsed * policy_hz) + 1
        time.sleep(max(0.0, start + calls / policy_hz - time.perf_counter()))


class Controller:
    """Stream rt/lowcmd at 500 Hz and check safety on every cycle."""

    def __init__(self, monitor, publisher, kp, kd, torque_fraction):
        self.monitor = monitor
        self.publisher = publisher
        self.kp, self.kd = kp, kd
        self.torque_cap = TORQUE_LIMIT * torque_fraction
        self.crc = CRC()
        self.records = []
        self.clock_start = time.perf_counter()
        self.worst_overrun = 0.0
        self.worst_state_age = 0.0
        self.policy_calls = self.target_changes = 0
        self.worst_policy_time = 0.0

        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0], self.cmd.head[1] = 0xFE, 0xEF
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        for motor in self.cmd.motor_cmd:
            motor.mode = 0x01
            motor.q, motor.dq = POS_STOP, VEL_STOP
            motor.kp = motor.kd = motor.tau = 0.0
        for motor in self.cmd.motor_cmd[:12]:
            motor.dq = 0.0  # VEL_STOP with kd > 0 would ask for a huge torque in the simulator

    def latest_state(self, check_range):
        age = time.perf_counter() - self.monitor.stamp
        if age > MAX_STATE_AGE:
            raise Fault(f"no robot state for {age * 1000:.0f} ms")
        self.worst_state_age = max(self.worst_state_age, age)
        state = read_state(self.monitor.msg)
        problem = safety_problem(state, check_range)
        if problem:
            raise Fault(problem)
        return state

    def send(self, q, kp, kd):
        for motor, value in zip(self.cmd.motor_cmd[:12], q):
            motor.q = float(value)
            motor.kp = kp
            motor.kd = kd
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.publisher.Write(self.cmd)

    def record(self, stage, state, target, q_cmd):
        self.records.append((
            time.perf_counter() - self.clock_start, stage, state.q, state.dq, target, q_cmd,
            state.tau_est, state.temperature, state.rpy, state.gyro, state.accel,
            state.foot_force, state.power_v, state.power_a,
        ))

    def run_phase(self, stage, seconds, target_at, check_range=True, stop_when=None):
        """Send target_at(elapsed, state) every cycle until seconds pass or stop_when() is true."""
        print(f"{stage}...", flush=True)
        start = next_tick = time.perf_counter()
        next_status = 1.0
        while True:
            arm_watchdog(MAX_LOOP_STALL)
            elapsed = time.perf_counter() - start
            if elapsed >= seconds or (stop_when is not None and stop_when()):
                return
            state = self.latest_state(check_range)
            target = target_at(elapsed, state)
            q_cmd = limit_for_torque(target, state.q, state.dq, self.kp, self.kd, self.torque_cap)
            self.send(q_cmd, self.kp, self.kd)
            self.record(stage, state, target, q_cmd)
            if elapsed >= next_status:
                next_status += 1.0
                print(f"  {stage} t={elapsed:4.1f} s  roll={state.rpy[0]:+.2f}  pitch={state.rpy[1]:+.2f}  "
                      f"hottest={state.temperature.max()} °C  current={state.power_a:.1f} A", flush=True)
            next_tick += DT
            remaining = next_tick - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                self.worst_overrun = max(self.worst_overrun, -remaining)
                next_tick = time.perf_counter()  # don't send a burst to catch up

    def move_to(self, stage, goal, seconds, check_range=True):
        """Ramp from the measured pose, so the first command causes no torque step."""
        start_q = self.latest_state(check_range).q

        def target_at(elapsed, state):
            return start_q + (goal - start_q) * min(elapsed / seconds, 1.0)

        self.run_phase(stage, seconds, target_at, check_range)

    def hold(self, stage, goal, seconds, stop_when=None):
        self.run_phase(stage, seconds, lambda elapsed, state: goal, stop_when=stop_when)

    def check_ready_to_walk(self):
        state = self.latest_state(check_range=True)
        error = np.abs(state.q - HOME).max()
        tilt = np.abs(state.rpy[:2]).max()
        speed = np.abs(state.dq).max()
        if error > MAX_STAND_ERROR or tilt > MAX_STAND_TILT or speed > MAX_STAND_SPEED:
            raise Fault(f"not standing still at home: joint error {error:.2f} rad, "
                        f"tilt {tilt:.2f} rad, joint speed {speed:.2f} rad/s")

    def wait_for_go(self):
        termios.tcflush(sys.stdin, termios.TCIFLUSH)  # an earlier Enter must not start the walk
        print(f"Standing. Press Enter to walk (Ctrl-C or B stops). "
              f"It lies down after {GO_TIMEOUT_S:.0f} s.", flush=True)

        def entered():
            return bool(select.select([sys.stdin], [], [], 0.0)[0])

        self.hold("waiting", HOME, GO_TIMEOUT_S, stop_when=entered)
        if not entered():
            return False
        sys.stdin.readline()
        return True

    def walk(self, policy, env, policy_hz, seconds):
        start_yaw = self.latest_state(check_range=True).rpy[2]
        target = None
        next_call = 0

        def target_at(elapsed, state):
            nonlocal target, next_call
            if target is None or elapsed >= next_call / policy_hz:
                robot_state = build_robot_state(state, start_yaw)
                call_start = time.perf_counter()
                new_target = policy_target(policy, robot_state, env)
                self.worst_policy_time = max(self.worst_policy_time, time.perf_counter() - call_start)
                if target is not None and np.abs(new_target - target).max() > 0.1:
                    self.target_changes += 1
                target = new_target
                self.policy_calls += 1
                next_call = int(elapsed * policy_hz) + 1  # skip calls missed in a stall
            return target

        self.run_phase("walking", seconds, target_at)

    def run_sequence(self, policy, env, policy_hz, seconds, ask_before_walking):
        self.move_to("standing up", HOME, STAND_UP_S, check_range=False)
        self.hold("settling", HOME, SETTLE_S)
        if seconds > 0:
            self.check_ready_to_walk()
            if not ask_before_walking or self.wait_for_go():
                self.walk(policy, env, policy_hz, seconds)
                self.move_to("returning home", HOME, RETURN_HOME_S)
                self.hold("settling after walking", HOME, 1.0)
        self.move_to("lying down", LIE_DOWN, LIE_DOWN_S)

    def damp(self):
        """Stream damping. Motor state doesn't matter here, so this can't fault."""
        print("Damping the motors.", flush=True)
        zero = np.zeros(12)
        next_tick = time.perf_counter()
        end = next_tick + DAMP_S
        while time.perf_counter() < end:
            self.send(zero, 0.0, DAMP_KD)
            next_tick += DT
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    def save_log(self, path, meta):
        if not self.records:
            return None
        path.parent.mkdir(parents=True, exist_ok=True)
        columns = {name: np.array(column) for name, column in zip(LOG_FIELDS, zip(*self.records))}
        np.savez_compressed(path, meta=json.dumps(meta), **columns)
        return path

    def print_summary(self):
        if self.policy_calls:
            print(f"Policy: {self.policy_calls} calls, target changed on {self.target_changes}, "
                  f"slowest {self.worst_policy_time * 1000:.1f} ms")
        if self.records:
            tau = np.abs(np.array([record[LOG_FIELDS.index("tau_est")] for record in self.records]))
            print(f"Peak estimated torque: hip {tau[:, 0::3].max():.1f}, "
                  f"thigh {tau[:, 1::3].max():.1f}, calf {tau[:, 2::3].max():.1f} N·m")
        print(f"Timing: worst loop overrun {self.worst_overrun * 1000:.1f} ms, "
              f"oldest state {self.worst_state_age * 1000:.1f} ms")


def connect(args):
    """Subscribe to the robot's state and wait for a valid message."""
    interface = args.robot or "lo"
    domain = args.domain if args.domain is not None else (0 if args.robot else 1)
    if args.robot and not check_interface(args.robot):
        raise Refused(f"interface {args.robot} can't be used")
    ChannelFactoryInitialize(domain, interface)
    monitor = StateMonitor()
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(monitor, 10)
    deadline = time.perf_counter() + 5.0
    while monitor.msg is None:
        if time.perf_counter() > deadline:
            where = "Is the robot connected?" if args.robot else "Is go2/sim_headless.py running?"
            raise Refused(f"no rt/lowstate on {interface}, domain {domain}, after 5 s. {where}")
        time.sleep(0.01)
    problems = state_problems(monitor.msg)
    if problems:
        raise Refused("invalid robot state: " + "; ".join(problems))
    return monitor, subscriber


def confirm_stop_button(monitor):
    """Prove the remote reaches this script, because low-level mode relies on it to stop."""
    print("Press and release B on the remote to show that the stop button works.", flush=True)
    deadline = time.perf_counter() + 30.0
    for wanted in (True, False):
        while read_state(monitor.msg).stop_pressed is not wanted:
            if time.perf_counter() > deadline:
                raise Refused("didn't see B pressed and released. Is the remote on?")
            time.sleep(0.01)


def release_sport_mode():
    """Lie the robot down and switch sport mode off. Return the mode that was active."""
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(5.0)
    switcher.Init()
    sport = SportClient()
    sport.SetTimeout(5.0)
    sport.Init()
    code, mode = switcher.CheckMode()
    if code != 0:
        raise Refused(f"couldn't read the robot's mode (code {code})")
    original = mode["name"]
    if not original:
        return original
    print(f"Lying down and releasing sport mode ({original})...", flush=True)
    for _ in range(RELEASE_ATTEMPTS):
        sport.StandDown()
        time.sleep(STAND_DOWN_S)  # let it finish, so releasing doesn't drop the robot
        switcher.ReleaseMode()
        time.sleep(1.0)
        code, mode = switcher.CheckMode()
        if code == 0 and not mode["name"]:
            return original
    raise Refused("sport mode didn't release")


def prepare_robot(args, monitor, digest):
    """Run the checks and confirmations for a real robot. Return the original mode."""
    if not sys.stdin.isatty():
        raise Refused("--robot needs an interactive terminal, for the confirmations")
    state = read_state(monitor.msg)
    problem = safety_problem(state, check_range=False)
    if problem:
        raise Refused(problem)
    battery = monitor.msg.bms_state.soc
    if battery < MIN_BATTERY_PERCENT:
        raise Refused(f"battery is at {battery}%, minimum {MIN_BATTERY_PERCENT}%")
    print(f"\nTHIS WILL MOVE THE ROBOT on {args.robot}.\n"
          f"  Policy:  {args.policy} (sha256 {digest[:12]})\n"
          f"  Rate:    {args.policy_hz:g} Hz, kp {args.kp:g}, kd {args.kd:g}\n"
          f"  Torque:  capped at {args.torque_fraction:.0%} of the limits\n"
          f"  Walking: {args.seconds:g} s; it will lie down first, then stand up\n"
          f"  Battery: {battery}%, hottest motor {state.temperature.max()} °C\n")
    if input('Type "yes" to continue: ').strip().lower() != "yes":
        raise Refused("not confirmed")
    confirm_stop_button(monitor)
    return release_sport_mode()


def bounded(low, high):
    def parse(text):
        value = float(text)
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(f"must be between {low:g} and {high:g}")
        return value
    return parse


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("policy", type=Path, help="policy file from S-M-ICRL")
    parser.add_argument("--robot", metavar="INTERFACE", nargs="?", const=DEFAULT_INTERFACE,
                        help="use the robot, on this network interface (if none is given: %(const)s); "
                             "without --robot, use the simulator")
    parser.add_argument("--dry-run", action="store_true", help="print commands without publishing them")
    parser.add_argument("--seconds", type=bounded(0.0, MAX_WALK_S), default=3.0,
                        help="how long to walk; 0 only stands up and lies down (default: %(default)s)")
    parser.add_argument("--policy-hz", type=bounded(0.5, 500.0), default=POLICY_HZ,
                        help="policy rate, which sets this policy's gait (default: %(default)s)")
    parser.add_argument("--kp", type=bounded(1.0, 200.0), default=KP, help="default: %(default)s")
    parser.add_argument("--kd", type=bounded(0.0, 20.0), default=KD, help="default: %(default)s")
    parser.add_argument("--torque-fraction", type=bounded(0.1, 1.0), default=0.6,
                        help="share of the torque limits to aim for; peaks can go above it "
                             "(default: %(default)s)")
    parser.add_argument("--probe-height", type=bounded(0.0, 1.0), default=PROBE_HEIGHT,
                        help="flat-floor terrain probes read minus this value (default: %(default)s)")
    parser.add_argument("--domain", type=int, help="DDS domain id (default: 0 with --robot, otherwise 1)")
    parser.add_argument("--log", type=Path, help="where to save the run (default: go2/logs/<time>.npz)")
    return parser.parse_args()


def main():
    args = parse_args()
    signal.signal(signal.SIGALRM, _on_watchdog)
    signal.signal(signal.SIGTERM, _on_terminate)
    env = flat_floor_env(args.probe_height)
    original_mode = None
    try:
        policy, digest = prepare_policy(args.policy, env)
        monitor, subscriber = connect(args)
        if args.dry_run:
            dry_run(monitor, policy, env, args.policy_hz, args.seconds)
            return 0
        if args.robot:
            original_mode = prepare_robot(args, monitor, digest)
    except (Refused, Fault) as error:
        print(f"REFUSED: {error}", flush=True)
        return 2
    except KeyboardInterrupt:
        print("Stopped before sending any low-level commands.", flush=True)
        return 2

    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    controller = Controller(monitor, publisher, args.kp, args.kd, args.torque_fraction)
    outcome = "finished"
    status = 0
    try:
        controller.run_sequence(policy, env, args.policy_hz, args.seconds, ask_before_walking=bool(args.robot))
    except Fault as fault:
        outcome, status = f"FAULT: {fault}", 1
    except KeyboardInterrupt:
        outcome, status = "stopped by the operator", 1
    except Exception:
        traceback.print_exc()
        outcome, status = "stopped by an unexpected error", 1
    finally:
        # Nothing may interrupt damping now.
        arm_watchdog(0.0)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            controller.damp()
        except Exception:
            traceback.print_exc()
            outcome, status = f"{outcome}; DAMPING FAILED", 1

    log_path = args.log or Path(__file__).resolve().parent / "logs" / f"{datetime.now():%Y-%m-%d_%H-%M-%S}.npz"
    saved = controller.save_log(log_path, {
        "policy": str(args.policy), "sha256": digest, "outcome": outcome, "interface": args.robot or "lo",
        "policy_hz": args.policy_hz, "kp": args.kp, "kd": args.kd, "seconds": args.seconds,
        "torque_fraction": args.torque_fraction, "probe_height": args.probe_height,
        "original_mode": original_mode,
    })
    print(f"\nResult: {outcome}")
    controller.print_summary()
    if saved:
        print(f"Log: {saved}")
    if args.robot:
        print(f"Sport mode is off (it was {original_mode!r}). Restart the robot to get it back.")
    return status


if __name__ == "__main__":
    sys.exit(main())
