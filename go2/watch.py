#!/usr/bin/env python
"""Watch a policy run from a second terminal and record what the robot did. READ-ONLY.

Run it next to go2/run_policy.py. It subscribes to rt/lowstate, rt/lowcmd (the commands
run_policy.py sends), rt/sportmodestate and rt/utlidar/robot_odom, and publishes
nothing, so it can't move the robot. Every second it prints the state and command
rates, the command-to-torque latency measured during the walk, tilt, torque saturation,
temperature and current, and a warning for anything out of bounds. When run_policy.py
stops sending commands, it saves everything for go2/diagnose.py report.

It saves every field of every message, not only the ones the scripts use, and it
runs go2/state_estimator.py on each rt/lowstate, so the log has the full 37-element
robot_state the policy would see at 500 Hz. With the robot walking in sport mode
(remote control, no run_policy.py), Ctrl-C saves a recording that
`go2/diagnose.py inputs` uses to check the estimate against Unitree's own.
Once a second it also samples this computer's CPU, memory and temperature and
run_policy.py's CPU share, and at the end it prints what the run used.

    nice -n 10 python go2/watch.py --robot                    # real robot, on the default interface
    nice -n 10 python go2/watch.py --robot enp3s0             # real robot, on another interface
    python go2/watch.py                                       # go2/sim_headless.py

Start it before run_policy.py. `nice` keeps it from taking CPU time from the control loop.

Exit status: 0 = saved a recording, 1 = nothing recorded, 2 = couldn't connect.
"""
import argparse
import bisect
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime
from types import SimpleNamespace
from pathlib import Path

import numpy as np
from check_connection import DEFAULT_INTERFACE, check_interface
from run_policy import (JOINTS, MAX_TEMPERATURE, MAX_TILT, ODOM_TOPIC, STOP_BUTTON_MASK, TORQUE_LIMIT,
                        build_robot_state, quat_to_rpy)
from state_estimator import BaseEstimator
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_

LOG_DIR = Path(__file__).resolve().parent / "logs" / "watch"
STATUS_PERIOD = 1.0  # s
POLL_PERIOD = 0.004  # s between collections of the messages that have arrived
DEPTH = 4000  # messages a topic keeps between collections: 8 s of rt/lowstate
WINDOW = 2.0  # s of data behind each status line
BEHIND_WARN_MS = 250.0  # warn when the newest state collected is older than this
IDLE_STOP = 3.0  # s without commands, after some arrived, before saving
SAT_SHARE = 0.95
LIVE_FIT_R2 = 0.9  # show a live latency only when the fit explains this much of the torque

# Warning levels. LATENCY_WARN is where the S-M-ICRL sweep of the walk policy degraded
# (fine at 10 ms, half the distance at 20 ms); set it with --latency-warn.
LATENCY_WARN_MS = 15.0
STATE_GAP_WARN_MS = 10.0  # 5 states in a row missing
CMD_GAP_WARN_MS = 10.0
SAT_WARN_PCT = 10.0
TILT_WARN = 0.3  # rad; run_policy.py stops at MAX_TILT
HOT_WARN = MAX_TEMPERATURE - 10

GROUPS = {"hip": [0, 3, 6, 9], "thigh": [1, 4, 7, 10], "calf": [2, 5, 8, 11]}


def source_time(msg):
    """When DDS's writer sent this message, in seconds on the sender's clock; None if unknown."""
    stamp = getattr(getattr(msg, "sample_info", None), "source_timestamp", 0)
    return stamp * 1e-9 if stamp and stamp > 0 else None


def tick_time(msg):
    """rt/lowstate carries the robot's own clock, in ms."""
    return msg.tick / 1000.0 if msg.tick > 0 else source_time(msg)


class Stream:
    """Rows from one topic, each with the time it was sent.

    poll() collects the messages that arrived since the last call; the main loop calls it
    every POLL_PERIOD. DDS isn't given a handler to call for each message: its handler
    calls wait for the Python interpreter, and while the status line was being worked
    out they fell hundreds of ms behind, which showed up as state gaps the robot never
    had (see run_policy.StateMonitor). DDS keeps up to DEPTH messages between calls, so
    none are lost while this program is busy.

    A row starts with the sender's clock, in seconds: the robot's tick for rt/lowstate,
    DDS's send time for the others. `offset` turns that into this computer's
    time.perf_counter(): it is the smallest gap seen between the two clocks, so the
    fastest delivery counts as no delay. perf_counter is CLOCK_MONOTONIC on Linux, shared
    by every process, so the times line up with run_policy.py's log (its meta
    "clock_start" is on the same clock). A topic that gives no send time is stamped with
    the time of collection instead.
    """

    def __init__(self, topic, kind, extract, sender_time=source_time):
        self.rows = []
        self.sent = []  # each row's first element, for searching by time
        self.offset = 0.0
        self._gap = None
        self._extract = extract
        self._sender_time = sender_time
        self._subscriber = ChannelSubscriber(topic, kind)
        self._subscriber.InitPolled(DEPTH)

    def poll(self):
        now = time.perf_counter()
        for msg in self._subscriber.ReadAll():
            sent = self._sender_time(msg) if self._gap != 0.0 else None
            if sent is None:  # no send time on this topic: use the time of collection
                sent, self._gap = now, 0.0
            elif self._gap is None or now - sent < self._gap:
                self._gap = now - sent
            self.offset = self._gap
            self.sent.append(sent)
            self.rows.append((sent,) + self._extract(msg))

    def since_index(self, i):
        return self.rows[i:]

    def since(self, t0):
        """Rows sent at or after t0, a time.perf_counter() reading."""
        return self.rows[bisect.bisect_left(self.sent, t0 - self.offset):]

    def times(self, rows):
        """These rows' send times as time.perf_counter() readings."""
        return np.array([r[0] for r in rows]) + self.offset

    def newest(self):
        """The newest row's send time as a time.perf_counter() reading, or None."""
        return self.sent[-1] + self.offset if self.sent else None

    def __len__(self):
        return len(self.rows)


def lowstate_row(msg):
    """Every rt/lowstate field."""
    m = msg.motor_state[:12]
    imu, bms = msg.imu_state, msg.bms_state
    return (msg.tick, [x.q for x in m], [x.dq for x in m], [x.tau_est for x in m],
            [x.temperature for x in m], list(imu.quaternion), list(imu.gyroscope),
            list(imu.accelerometer), list(msg.foot_force), msg.power_v, msg.power_a,
            bms.soc, bool(msg.wireless_remote[3] & STOP_BUTTON_MASK),
            list(imu.rpy), imu.temperature, [x.ddq for x in m], [x.q_raw for x in m],
            [x.dq_raw for x in m], [x.ddq_raw for x in m], [x.mode for x in m], [x.lost for x in m],
            list(msg.foot_force_est), bms.current, bms.status, list(bms.cell_vol), list(bms.bq_ntc),
            list(bms.mcu_ntc), list(msg.wireless_remote), msg.bit_flag, msg.adc_reel,
            getattr(msg, "temperature_ntc1", 0), getattr(msg, "temperature_ntc2", 0),
            list(msg.fan_frequency))


class Derived:
    """The estimator's output and the policy's 37-element robot_state for every rt/lowstate.

    It runs in batches from the status loop, with the same results as running it on each
    message as it arrives. Heading is measured from the first message's yaw;
    run_policy.py measures it from the start of the walk, which go2/diagnose.py lines
    up afterwards.
    """

    FIELDS = ("robot_state", "est_velocity", "est_height", "est_contact_weight", "est_contact_from_force",
              "est_accel_used")

    def __init__(self):
        self.estimator = BaseEstimator()  # IMU world frame; build_robot_state turns it to the heading
        self.start_yaw = None
        self.rows = []

    def catch_up(self, stream):
        for row in stream.since_index(len(self.rows)):
            q, dq, quat, gyro, accel, force = (np.asarray(row[i], dtype=float) for i in (2, 3, 6, 7, 8, 9))
            rpy = quat_to_rpy(*quat)
            if self.start_yaw is None:
                self.start_yaw = float(rpy[2])
            velocity, height = self.estimator.update(row[0], q, dq, quat, gyro, accel, force)
            view = SimpleNamespace(q=q, dq=dq, quaternion=quat, rpy=rpy, gyro=gyro)
            robot_state = build_robot_state(view, self.start_yaw, estimate=(velocity, height))
            self.rows.append((robot_state, velocity, height, self.estimator.weight.copy(),
                              self.estimator.contact_source == "force", self.estimator.accel_ok))

    def columns(self):
        if not self.rows:
            return {}
        return {f"state_{name}": np.array(column) for name, column in zip(self.FIELDS, zip(*self.rows))}


LOWSTATE_FIELDS = ("tick", "q", "dq", "tau_est", "temperature", "quaternion", "gyro", "accel",
                   "foot_force", "power_v", "power_a", "battery", "stop_pressed",
                   "imu_rpy", "imu_temperature", "ddq", "q_raw", "dq_raw", "ddq_raw", "motor_mode",
                   "motor_lost", "foot_force_est", "bms_current", "bms_status", "bms_cell_vol", "bms_bq_ntc",
                   "bms_mcu_ntc", "wireless_remote", "bit_flag", "adc_reel", "temperature_ntc1",
                   "temperature_ntc2", "fan_frequency")


def lowcmd_row(msg):
    m = msg.motor_cmd[:12]
    return ([x.q for x in m], [x.dq for x in m], [x.kp for x in m], [x.kd for x in m], [x.tau for x in m])


LOWCMD_FIELDS = ("q", "dq", "kp", "kd", "tau")


def sport_row(msg):
    imu = msg.imu_state
    return (list(msg.position), list(msg.velocity), msg.body_height, msg.yaw_speed, msg.mode,
            msg.gait_type, msg.foot_raise_height, list(msg.range_obstacle), list(msg.foot_force),
            list(msg.foot_position_body), list(msg.foot_speed_body), list(imu.quaternion), list(imu.rpy),
            list(imu.gyroscope), list(imu.accelerometer), msg.error_code, msg.progress,
            msg.stamp.sec + 1e-9 * msg.stamp.nanosec)


SPORT_FIELDS = ("position", "velocity", "body_height", "yaw_speed", "mode", "gait_type", "foot_raise_height",
                "range_obstacle", "foot_force", "foot_position_body", "foot_speed_body", "quaternion", "rpy",
                "gyro", "accel", "error_code", "progress", "stamp")


def odom_row(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    return ([p.x, p.y, p.z, q.w, q.x, q.y, q.z], [v.x, v.y, v.z], [w.x, w.y, w.z],
            msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)


ODOM_FIELDS = ("pose", "velocity", "angular_velocity", "stamp")


class SystemSampler:
    """This computer's load once a second: CPU, memory, temperature, and run_policy.py's share.

    Reads Linux's /proc and /sys; elsewhere only the load average and this process's CPU.
    """

    FIELDS = ("t", "load1", "cpu_pct", "mem_used_pct", "cpu_temp_c", "policy_cpu_pct", "policy_rss_mb",
              "watch_cpu_pct")

    def __init__(self):
        self.rows = []
        self._last_cpu = self._read_cpu()
        self._last_proc = {}
        self._policy_pid = None
        self._pid_checked = 0.0
        self._last_t = time.perf_counter()
        self._own_last = time.process_time()

    @staticmethod
    def _read_cpu():
        try:
            fields = [float(x) for x in Path("/proc/stat").read_text().split("\n")[0].split()[1:]]
            return sum(fields), fields[3] + fields[4]  # total, idle + iowait
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _proc_times(pid):
        try:
            parts = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
            ticks = float(parts[11]) + float(parts[12])  # utime + stime
            rss_mb = float(parts[21]) * os.sysconf("SC_PAGE_SIZE") / 1e6
            return ticks / os.sysconf("SC_CLK_TCK"), rss_mb
        except (OSError, ValueError, IndexError):
            return None

    def _find_policy(self, now):
        if (self._policy_pid and Path(f"/proc/{self._policy_pid}").exists()) or now - self._pid_checked < 2.0:
            return self._policy_pid
        self._pid_checked, self._policy_pid = now, None
        for proc in Path("/proc").glob("[0-9]*"):
            try:
                if b"run_policy.py" in (proc / "cmdline").read_bytes():
                    self._policy_pid = int(proc.name)
                    break
            except OSError:
                continue
        return self._policy_pid

    def sample(self):
        now = time.perf_counter()
        dt = max(now - self._last_t, 1e-3)
        self._last_t = now
        load1 = os.getloadavg()[0] if hasattr(os, "getloadavg") else np.nan
        cpu_pct = mem_pct = temp = pol_cpu = pol_rss = np.nan
        cpu = self._read_cpu()
        if cpu and self._last_cpu:
            total, idle = cpu[0] - self._last_cpu[0], cpu[1] - self._last_cpu[1]
            cpu_pct = 100.0 * (1.0 - idle / total) if total > 0 else np.nan
        self._last_cpu = cpu
        try:
            info = dict(line.split(":", 1) for line in Path("/proc/meminfo").read_text().splitlines() if ":" in line)
            total, avail = float(info["MemTotal"].split()[0]), float(info["MemAvailable"].split()[0])
            mem_pct = 100.0 * (1.0 - avail / total)
        except (OSError, KeyError, ValueError):
            pass
        temps = []
        for zone in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
            try:
                temps.append(float(zone.read_text()) / 1000.0)
            except (OSError, ValueError):
                pass
        temp = max(temps) if temps else np.nan
        pid = self._find_policy(now) if Path("/proc").exists() else None
        if pid:
            times = self._proc_times(pid)
            if times:
                last = self._last_proc.get(pid)
                if last:
                    pol_cpu = 100.0 * (times[0] - last) / dt
                self._last_proc = {pid: times[0]}
                pol_rss = times[1]
        own = time.process_time()
        watch_cpu = 100.0 * (own - self._own_last) / dt
        self._own_last = own
        self.rows.append((now, load1, cpu_pct, mem_pct, temp, pol_cpu, pol_rss, watch_cpu))

    def columns(self):
        if not self.rows:
            return {}
        cols = list(zip(*self.rows))
        return {f"sys_{name}": np.array(col, dtype=float) for name, col in zip(self.FIELDS, cols)}


def usage_summary(data, system):
    """What the run used: energy, battery, temperatures, torque and speed, packets, computer load."""
    out = {}
    t = data.get("state_t")
    if t is None or len(t) < 2:
        return out
    sel = np.ones(len(t), dtype=bool)
    if "cmd_t" in data and len(data["cmd_t"]):
        sel = (t >= data["cmd_t"][0]) & (t <= data["cmd_t"][-1])  # while commands came
        out["commanded_s"] = round(float(data["cmd_t"][-1] - data["cmd_t"][0]), 2)
    ts = t[sel]
    if ts.size < 2:
        return out
    power = data["state_power_v"][sel] * data["state_power_a"][sel]
    dt = np.diff(ts, prepend=ts[0])
    out["energy_j"] = round(float(np.sum(power * dt)), 1)
    out["power_w"] = {"mean": round(float(power.mean()), 1), "peak": round(float(power.max()), 1)}
    battery = data["state_battery"][sel]
    out["battery_pct"] = {"start": int(battery[0]), "end": int(battery[-1])}
    temp = data["state_temperature"][sel].astype(float)
    out["motor_temp_c"] = {g: {"start": float(temp[0, i].max()), "peak": float(temp[:, i].max())}
                           for g, i in GROUPS.items()}
    tau, dq = np.abs(data["state_tau_est"][sel]), np.abs(data["state_dq"][sel])
    out["torque_share_of_limit"] = {g: {"mean": round(float((tau[:, i] / TORQUE_LIMIT[i]).mean()), 3),
                                        "p99": round(float(np.percentile(tau[:, i] / TORQUE_LIMIT[i], 99)), 3),
                                        "saturated_pct": round(100.0 * float(np.mean(tau[:, i] >= SAT_SHARE * TORQUE_LIMIT[i])), 2)}
                                    for g, i in GROUPS.items()}
    out["joint_speed_peak_rad_s"] = {g: round(float(dq[:, i].max()), 2) for g, i in GROUPS.items()}
    if "state_motor_lost" in data:
        lost = data["state_motor_lost"][sel].astype(float)
        out["motor_packets_lost"] = int((lost[-1] - lost[0]).sum())
    gaps = np.diff(ts) * 1000.0
    out["state_hz"] = round(float(len(ts) / (ts[-1] - ts[0])), 1)
    out["state_gap_ms_max"] = round(float(gaps.max()), 1)
    if "state_robot_state" in data:
        rs = data["state_robot_state"][sel]
        out["estimated_distance_m"] = round(float(np.sum(np.hypot(rs[:, 30], rs[:, 31]) * dt)), 2)
        out["estimated_height_m"] = {"mean": round(float(rs[:, 33].mean()), 3), "min": round(float(rs[:, 33].min()), 3)}
    if system:
        st = system["sys_t"]
        ssel = (st >= ts[0]) & (st <= ts[-1] + 1.0)
        def stat(name):
            v = system[name][ssel]
            v = v[np.isfinite(v)]
            return {"mean": round(float(v.mean()), 1), "max": round(float(v.max()), 1)} if v.size else None
        out["computer"] = {"cpu_pct": stat("sys_cpu_pct"), "load1": stat("sys_load1"),
                           "cpu_temp_c": stat("sys_cpu_temp_c"), "mem_used_pct": stat("sys_mem_used_pct"),
                           "run_policy_cpu_pct": stat("sys_policy_cpu_pct"),
                           "run_policy_rss_mb": stat("sys_policy_rss_mb"), "watch_cpu_pct": stat("sys_watch_cpu_pct")}
    return out


def print_usage(usage):
    if not usage:
        return
    print("Usage" + (f" over {usage['commanded_s']} s of commands:" if "commanded_s" in usage else ":"))
    print(f"  energy {usage['energy_j']} J, power {usage['power_w']['mean']} W mean / {usage['power_w']['peak']} W peak, "
          f"battery {usage['battery_pct']['start']}% -> {usage['battery_pct']['end']}%")
    print("  torque (share of limit, mean / p99 / saturated %): " + ", ".join(
        f"{g} {v['mean']:.2f}/{v['p99']:.2f}/{v['saturated_pct']:.1f}" for g, v in usage["torque_share_of_limit"].items()))
    print("  motor temperature: " + ", ".join(f"{g} {v['start']:.0f}->{v['peak']:.0f} °C"
                                              for g, v in usage["motor_temp_c"].items()))
    print(f"  state {usage['state_hz']} Hz (gap max {usage['state_gap_ms_max']} ms)"
          + (f", motor packets lost {usage['motor_packets_lost']}" if "motor_packets_lost" in usage else ""))
    if "estimated_distance_m" in usage:
        print(f"  estimated distance {usage['estimated_distance_m']} m, height {usage['estimated_height_m']['mean']} m "
              f"(min {usage['estimated_height_m']['min']})")
    c = usage.get("computer") or {}
    if c.get("run_policy_cpu_pct") or c.get("cpu_pct"):
        print(f"  computer: CPU {c.get('cpu_pct')}, run_policy.py CPU {c.get('run_policy_cpu_pct')}, "
              f"CPU temp {c.get('cpu_temp_c')} °C")


def columns(stream, fields, prefix):
    rows = stream.rows
    if not rows:
        return {}
    out = {f"{prefix}_t": stream.times(rows)}
    for i, name in enumerate(fields, start=1):
        out[f"{prefix}_{name}"] = np.array([r[i] for r in rows])
    return out


def fit_delay(cmd_t, cmd_q, cmd_kp, cmd_kd, st_t, q, dq, tau, max_delay=0.06):
    """Delay and scale that best explain tau_est as the PD law on an older command.

    The motor uses fresh q and dq with the command it received some time ago, so
    tau(s) = a * (kp * (q_cmd(s - delay) - q(s)) - kd * dq(s)). Returns
    {group: (delay_ms, scale, r2)} for groups with enough motion to tell.
    """
    out = {}
    if len(cmd_t) < 50 or len(st_t) < 50:
        return out
    delays = np.arange(0.0, max_delay + 1e-9, 0.001)
    order = np.argsort(cmd_t)
    cmd_t = cmd_t[order]
    for group, idx in GROUPS.items():
        best = None
        for delay in delays:
            sse = var = 0.0
            scales = []
            for j in idx:
                kp = np.interp(st_t - delay, cmd_t, cmd_kp[order, j])
                kd = np.interp(st_t - delay, cmd_t, cmd_kd[order, j])
                qc = np.interp(st_t - delay, cmd_t, cmd_q[order, j])
                model = kp * (qc - q[:, j]) - kd * dq[:, j]
                ok = (kp > 1.0) & (np.abs(model) < 0.95 * TORQUE_LIMIT[j]) & (np.abs(qc) < 100.0)
                if ok.sum() < 50 or np.std(model[ok]) < 0.5:
                    continue
                A = np.column_stack([model[ok], np.ones(ok.sum())])
                coef, *_ = np.linalg.lstsq(A, tau[ok, j], rcond=None)
                sse += float(np.sum((tau[ok, j] - A @ coef) ** 2))
                var += float(np.var(tau[ok, j]) * ok.sum())
                scales.append(coef[0])
            if scales and (best is None or sse < best[0]):
                best = (sse, delay, 1.0 - sse / max(var, 1e-12))
        if best:
            out[group] = (best[1] * 1000.0, _scale(best[1], idx, cmd_t, cmd_q[order], cmd_kp[order],
                                                   cmd_kd[order], st_t, q, dq, tau), best[2])
    return out


def _scale(delay, idx, cmd_t, cmd_q, cmd_kp, cmd_kd, st_t, q, dq, tau, edge=0.004):
    """tau_est / PD law, away from command steps, where a 1 ms timing error would dominate."""
    scales = []
    for j in idx:
        at = st_t - delay
        kp, kd = np.interp(at, cmd_t, cmd_kp[:, j]), np.interp(at, cmd_t, cmd_kd[:, j])
        qc = np.interp(at, cmd_t, cmd_q[:, j])
        jump = np.abs(np.interp(at + edge, cmd_t, cmd_q[:, j]) - np.interp(at - edge, cmd_t, cmd_q[:, j]))
        model = kp * (qc - q[:, j]) - kd * dq[:, j]
        ok = (kp > 1.0) & (jump < 0.01) & (np.abs(model) < 0.95 * TORQUE_LIMIT[j]) & (np.abs(qc) < 100.0)
        if ok.sum() >= 50 and np.std(model[ok]) >= 0.5:
            A = np.column_stack([model[ok], np.ones(ok.sum())])
            scales.append(np.linalg.lstsq(A, tau[ok, j], rcond=None)[0][0])
    return float(np.median(scales)) if scales else float("nan")


def status(state, cmd, now, latency_warn, derived):
    """One status line, and the warnings that go with it."""
    warnings = []
    s = state.since(now - WINDOW)
    c = cmd.since(now - WINDOW)
    parts = []
    if len(s) < 2:
        return "no rt/lowstate", ["NO STATE: is the robot connected?"]
    st_t = state.times(s)
    gaps = np.diff(st_t) * 1000.0
    parts.append(f"state {len(s) / WINDOW:4.0f} Hz (gap max {gaps.max():5.1f} ms)")
    if gaps.max() > STATE_GAP_WARN_MS:
        warnings.append(f"state gap {gaps.max():.0f} ms: that much of the robot's state never arrived "
                        f"(run_policy.py faults at 100 ms)")
    behind = (now - st_t[-1]) * 1000.0
    if behind > BEHIND_WARN_MS:
        warnings.append(f"watch.py is {behind:.0f} ms behind the robot: this computer is too busy, and "
                        f"the lines above describe that long ago")
    q = np.array([r[2] for r in s])
    dq = np.array([r[3] for r in s])
    tau = np.array([r[4] for r in s])
    temp = np.array([r[5] for r in s])
    rpy = np.array([quat_to_rpy(*r[6]) for r in s[:: max(1, len(s) // 50)]])
    tilt = np.abs(rpy[:, :2]).max()
    sat = 100.0 * np.mean(np.abs(tau) >= SAT_SHARE * TORQUE_LIMIT)
    current = np.mean([r[11] for r in s])
    if c:
        cmd_t = cmd.times(c)
        cgaps = np.diff(cmd_t) * 1000.0 if len(c) > 1 else np.array([0.0])
        parts.append(f"cmd {len(c) / WINDOW:4.0f} Hz (gap max {cgaps.max():5.1f} ms)")
        if cgaps.max() > CMD_GAP_WARN_MS:
            warnings.append(f"command gap {cgaps.max():.0f} ms: run_policy.py's loop stalled")
        fit = fit_delay(cmd_t, np.array([r[1] for r in c]), np.array([r[3] for r in c]),
                        np.array([r[4] for r in c]), st_t, q, dq, tau)
        fit = {g: v for g, v in fit.items() if v[2] >= LIVE_FIT_R2}  # too little motion otherwise
        if fit:
            parts.append("latency " + " ".join(f"{g[0]}{v[0]:.0f}" for g, v in fit.items()) + " ms")
            worst = max(v[0] for v in fit.values())
            if worst > latency_warn:
                warnings.append(f"latency {worst:.0f} ms is above {latency_warn:g} ms, where the "
                                f"sim gait starts to break")
    else:
        parts.append("cmd    0 Hz")
    if derived.rows:
        rs, _, _, _, from_force, accel_used = derived.rows[-1]
        notes = [n for n, ok in (("no force", from_force), ("no accel", accel_used)) if not ok]
        parts.append(f"v {rs[30]:+.2f},{rs[31]:+.2f} h {rs[33]:.3f}" + (f" ({', '.join(notes)})" if notes else ""))
    parts.append(f"tilt {tilt:.2f}")
    parts.append(f"sat {sat:4.1f}%")
    parts.append(f"hot {temp.max():.0f}°C")
    parts.append(f"{current:.1f} A")
    if tilt > TILT_WARN:
        warnings.append(f"tilt {tilt:.2f} rad (run_policy.py stops at {MAX_TILT})")
    if c and sat > SAT_WARN_PCT:
        by = {g: 100.0 * np.mean(np.abs(tau[:, i]) >= SAT_SHARE * TORQUE_LIMIT[i]) for g, i in GROUPS.items()}
        worst = max(by, key=by.get)
        warnings.append(f"torque saturated {sat:.0f}% of the time, most on the {worst}s ({by[worst]:.0f}%)")
    if temp.max() >= HOT_WARN:
        warnings.append(f"{JOINTS[int(temp.max(axis=0).argmax())]} at {temp.max():.0f} °C")
    if s[-1][13]:
        warnings.append("B pressed")
    return "  ".join(parts), warnings


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", metavar="INTERFACE", nargs="?", const=DEFAULT_INTERFACE,
                        help="watch the robot, on this network interface (if none is given: %(const)s); "
                             "without --robot, watch go2/sim_headless.py")
    parser.add_argument("--domain", type=int, help="DDS domain id (default: 0 with --robot, otherwise 1)")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after this long; 0 = stop once commands have come and then stopped")
    parser.add_argument("--latency-warn", type=float, default=LATENCY_WARN_MS,
                        help="warn when the measured latency is above this, in ms (default: %(default)s)")
    parser.add_argument("--quiet", action="store_true", help="print warnings only, not the status line")
    parser.add_argument("--log", type=Path, help="where to save (default: go2/logs/watch/<time>.npz)")
    return parser.parse_args()


def main():
    args = parse_args()
    interface = args.robot or "lo"
    domain = args.domain if args.domain is not None else (0 if args.robot else 1)
    if args.robot and not check_interface(args.robot):
        return 2
    ChannelFactoryInitialize(domain, interface)
    state = Stream("rt/lowstate", LowState_, lowstate_row, sender_time=tick_time)
    derived = Derived()
    system = SystemSampler()
    cmd = Stream("rt/lowcmd", LowCmd_, lowcmd_row)
    sport = Stream("rt/sportmodestate", SportModeState_, sport_row)
    odom = Stream(ODOM_TOPIC, Odometry_, odom_row)
    streams = (state, cmd, sport, odom)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    def on_hangup(*_):  # terminal closed: save quietly
        try:
            fd = os.open(os.devnull, os.O_WRONLY)
            os.dup2(fd, 1)
            os.dup2(fd, 2)
        except OSError:
            pass
        stop.set()

    signal.signal(signal.SIGHUP, on_hangup)

    print(f"Watching {interface}, domain {domain}. Read-only: this publishes nothing. "
          f"Ctrl-C to stop and save.", flush=True)
    start = time.perf_counter()
    seen_cmd = None
    warned = set()
    next_status = start + STATUS_PERIOD
    while not stop.wait(POLL_PERIOD):
        for stream in streams:
            stream.poll()
        now = time.perf_counter()
        if now < next_status:
            continue
        next_status = max(next_status + STATUS_PERIOD, now)
        derived.catch_up(state)
        system.sample()
        line, warnings = status(state, cmd, now, args.latency_warn, derived)
        if not args.quiet:
            print(f"t={now - start:6.1f}  {line}", flush=True)
        for w in warnings:
            key = w.split(" ")[0]
            print(f"  WARNING: {w}", flush=True)
            warned.add(key)
        if len(cmd):
            seen_cmd = seen_cmd or now
            if not args.seconds and now - cmd.newest() > IDLE_STOP:
                print("Commands stopped; saving.", flush=True)
                break
        if args.seconds and now - start >= args.seconds:
            break

    for stream in streams:  # whatever arrived during the last status line
        stream.poll()
    data = {}
    data.update(columns(state, LOWSTATE_FIELDS, "state"))
    derived.catch_up(state)
    n = len(data.get("state_t", []))
    data.update({k: v[:n] for k, v in derived.columns().items()})
    sys_columns = system.columns()
    data.update(sys_columns)
    data.update(columns(cmd, LOWCMD_FIELDS, "cmd"))
    data.update(columns(sport, SPORT_FIELDS, "sport"))
    data.update(columns(odom, ODOM_FIELDS, "odom"))
    if not data:
        print("Nothing recorded.")
        return 1
    path = args.log or LOG_DIR / f"{datetime.now():%Y-%m-%d_%H-%M-%S}.npz"
    path.parent.mkdir(parents=True, exist_ok=True)
    meta = {"interface": interface, "domain": domain, "started": start, "start_yaw": derived.start_yaw,
            "topics": {"lowstate": len(state), "lowcmd": len(cmd), "sportmodestate": len(sport),
                       "robot_odom": len(odom)}}
    meta["usage"] = usage_summary(data, sys_columns)
    np.savez_compressed(path, meta=json.dumps(meta), **data)
    print_usage(meta["usage"])
    print(f"Saved: {path}  ({', '.join(f'{k} {v}' for k, v in meta['topics'].items())} messages)")
    print(f"Next: python go2/diagnose.py report <run_policy log> --watch {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
