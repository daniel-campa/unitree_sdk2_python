#!/usr/bin/env python
"""Measure the Go2's lidar for go2/lidar_env.py: what it sends, and where it sits. READ-ONLY.

This script only subscribes to topics. It creates no publishers and no SDK clients,
so it can't move the robot or switch the lidar on or off.

    python go2/lidar_calibrate.py --robot "$IFACE"                         # flat floor only
    python go2/lidar_calibrate.py --robot "$IFACE" --wall-distance 1.775   # with the crate (or a board) ahead
    python go2/lidar_calibrate.py                                          # go2/sim_headless.py --lidar

Set-up: the robot standing still in sport mode on a flat, clear floor, nobody within
2 m. For --wall-distance, stand a flat board (at least 0.6 m wide and 0.5 m tall), or
push_box_turn's crate, square to the robot, straight ahead, with its face that many
metres from the robot's centre (on the floor, midway between the front and rear feet).
The crate's centre 2 m away puts its face at 1.775 m. Without the board, the lidar's
forward position and which way it faces come from Unitree's Go2 model (URDF_MOUNT).

It records every lidar topic for --seconds and works out:

  topics      each topic's rate, frame, fields and points per cloud; rt/utlidar/lidar_state,
              rt/utlidar/height_map_array and rt/utlidar/robot_odom
  view        the spread of the points in the lidar's frame, and the nearest floor it sees
              straight ahead (inside that, the terrain probes have no points)
  mount       of --topic, rt/utlidar/cloud by default: the one in the lidar's own frame,
              which go2/run_policy.py reads with sport mode off. From the floor: the
              lidar's roll, pitch and height on the body; from the board: how far forward
              it is, and which way it faces
  check       the env vector from each CLOUD_WINDOW_S of clouds, with that mount, as
              go2/run_policy.py builds it: on flat floor the terrain probes read minus the
              body height and object_dist reads 10, or the board's distance along
              S-M-ICRL's ray with --wall-distance. Also push_box_turn's seven crate beams

Sport mode is on during this, so it can't say whether the lidar keeps publishing once
run_policy.py switches sport mode off; run_policy.py's log records that on every run.

Results go to go2/logs/lidar/<time>.json, with the clouds in a .npz beside it
(--log moves both). A measured mount also goes to go2/logs/lidar_mount.json, where
go2/run_policy.py reads it (--mount-out moves it).

Exit status: 0 = mount measured, 1 = not (the summary says why), 2 = no connection or no lidar.
"""
import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from check_connection import DEFAULT_INTERFACE, check_interface
from lidar_env import (BODY_FRAMES, CLOUD_TOPICS, CLOUD_WINDOW_S, HEIGHT_MAP_TOPIC, LIDAR_STATE_TOPIC, MOUNT_FILE,
                       OPEN_OBSTACLE_DISTANCE, RAY_START_HEIGHT, WORLD_FRAMES, EnvLayout, Mount, NoReading, build_env,
                       cloud_frame, cloud_stamp, decode_points, rotation_rpy, to_level_base)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LidarState_, LowState_, SportModeState_

LOG_DIR = Path(__file__).resolve().parent / "logs" / "lidar"
ODOM_TOPIC = "rt/utlidar/robot_odom"
SECONDS = 10.0
POLL_PERIOD = 0.01  # s
DEPTH = 64  # messages a topic keeps between collections
KEEP_CLOUDS = 60  # clouds kept per topic for the fits and the log
FIT_POINTS = 60000  # points the floor fit uses at most
FLOOR_TOL = 0.02  # m from the plane that counts as on the floor
LIDAR_HEIGHT_RANGE = (0.1, 1.0)  # m: the floor must be this far from the lidar
WALL_MIN_POINTS = 30
WALL_HALF_WIDTH = 0.30  # m either side of straight ahead that the board is looked for in
EXPECTED_TOL = {"probe": 0.03, "object_dist": 0.05}  # m: how close the check must come
# Unitree's Go2 model (go2_description's radar_joint): on the head, facing forward and 15 deg
# down from upside-down. The floor fit can't tell forward from back, nor the forward offset.
URDF_MOUNT = Mount("utlidar_lidar", (0.28945, 0.0, -0.046825), (0.0, 2.8782, 0.0))
PUSH_BOX_TURN = EnvLayout(mode="obstacle_bearing", probe_bearings_deg=(30, 20, 10, 0, -10, -20, -30),
                          probe_range=4.0, probe_sector_deg=10.0, probe_rays_per_sector=5)


# --------------------------------------------------------------------------- recording


class Recorder:
    """Collects every message on one topic, without a handler (see run_policy.StateMonitor)."""

    def __init__(self, topic, kind, depth=DEPTH):
        self.topic = topic
        self.times = []
        self.messages = []
        self._subscriber = ChannelSubscriber(topic, kind)
        self._subscriber.InitPolled(depth)

    def poll(self, now, keep=None):
        for msg in self._subscriber.ReadAll():
            self.times.append(now)
            if keep is None or len(self.messages) < keep:
                self.messages.append(msg)
            else:  # keep the newest `keep`
                self.messages = self.messages[1:] + [msg]

    def rate(self):
        if len(self.times) < 2:
            return None
        return (len(self.times) - 1) / (self.times[-1] - self.times[0])


def quat_rpy(w, x, y, z):
    return (math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))),
            math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


# --------------------------------------------------------------------------- fits


def fit_plane(points, rng):
    """The largest plane LIDAR_HEIGHT_RANGE from the origin: (unit normal pointing at the origin, distance, inliers)."""
    best = None
    for _ in range(400):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        d = -n @ a
        if d < 0:
            n, d = -n, -d
        if not LIDAR_HEIGHT_RANGE[0] <= d <= LIDAR_HEIGHT_RANGE[1]:
            continue
        inliers = np.abs(points @ n + d) < FLOOR_TOL
        if best is None or inliers.sum() > best[2].sum():
            best = (n, d, inliers)
    if best is None:
        return None
    # Least squares on the inliers.
    on = points[best[2]]
    centre = on.mean(axis=0)
    n = np.linalg.svd(on - centre, full_matrices=False)[2][-1]
    d = -n @ centre
    if d < 0:
        n, d = -n, -d
    return n, d, np.abs(points @ n + d) < FLOOR_TOL


def mount_rotation(floor_normal, up_in_body, yaw):
    """Roll and pitch so that Rz(yaw) Ry(pitch) Rx(roll) turns the floor normal (lidar frame) into body up."""
    target = rotation_rpy(0.0, 0.0, -yaw) @ up_in_body  # what Ry(pitch) Rx(roll) must give
    best = None
    for step, span in ((math.radians(0.5), None), (math.radians(0.01), math.radians(1.0))):
        if span is None:
            rolls, pitches = np.arange(-math.pi, math.pi, step), np.arange(-math.pi / 2, math.pi / 2 + step, step)
        else:
            rolls = np.arange(best[0] - span, best[0] + span, step)
            pitches = np.arange(best[1] - span, best[1] + span, step)
        r, p = np.meshgrid(rolls, pitches)
        cr, sr, cp, sp = np.cos(r), np.sin(r), np.cos(p), np.sin(p)
        nx, ny, nz = floor_normal
        v = np.stack((cp * nx + sp * sr * ny + sp * cr * nz, cr * ny - sr * nz, -sp * nx + cp * sr * ny + cp * cr * nz))
        error = np.linalg.norm(v - target[:, None, None], axis=0)
        i = np.unravel_index(np.argmin(error), error.shape)
        best = (float(r[i]), float(p[i]), float(error[i]))
    return best


def find_wall(points, height):
    """The board's face in level-base-frame points: (distance ahead, its yaw in rad, points), or None."""
    face = points[(points[:, 0] > 0.2) & (points[:, 0] < 4.0) & (np.abs(points[:, 1]) < WALL_HALF_WIDTH)
                  & (points[:, 2] > 0.08 - height) & (points[:, 2] < 0.35)]
    if len(face) < WALL_MIN_POINTS:
        return None
    counts, edges = np.histogram(face[:, 0], bins=np.arange(0.2, 4.02, 0.02))
    dense = np.flatnonzero(counts >= max(10, WALL_MIN_POINTS // 3))
    if dense.size == 0:
        return None
    x0 = edges[dense[0]]
    on = face[np.abs(face[:, 0] - np.median(face[(face[:, 0] >= x0) & (face[:, 0] < x0 + 0.06), 0])) < 0.04]
    if len(on) < WALL_MIN_POINTS:
        return None
    slope = np.polyfit(on[:, 1], on[:, 0], 1)[0] if np.ptp(on[:, 1]) > 0.1 else 0.0  # x = a*y + b
    return float(np.median(on[:, 0])), float(math.atan(slope)), on


# --------------------------------------------------------------------------- analysis


def describe_topics(clouds, state_rec, map_rec, odom_rec):
    out = {}
    for rec in clouds:
        info = {"messages": len(rec.times), "rate_hz": rec.rate()}
        if rec.messages:
            m = rec.messages[-1]
            info.update(frames=sorted({cloud_frame(x) for x in rec.messages}),
                        fields=[f"{f.name}:{f.datatype}@{f.offset}" for f in m.fields], point_step=m.point_step,
                        points_median=int(np.median([x.width * x.height for x in rec.messages])),
                        dense=bool(m.is_dense))
        out[rec.topic] = info
    out[LIDAR_STATE_TOPIC] = {"messages": len(state_rec.times), "rate_hz": state_rec.rate()}
    if state_rec.messages:
        s = state_rec.messages[-1]
        out[LIDAR_STATE_TOPIC].update({k: (v if isinstance(v, (int, float, str)) else str(v))
                                       for k, v in vars(s).items() if not k.startswith("_")})
    out[HEIGHT_MAP_TOPIC] = {"messages": len(map_rec.times), "rate_hz": map_rec.rate()}
    if map_rec.messages:
        m = map_rec.messages[-1]
        out[HEIGHT_MAP_TOPIC].update(frame=m.frame_id, resolution=m.resolution, width=m.width, height=m.height)
    out[ODOM_TOPIC] = {"messages": len(odom_rec.times), "rate_hz": odom_rec.rate()}
    return out


def view_in_frame(points):
    """The spread of points in the cloud's own frame, in degrees and metres."""
    r = np.linalg.norm(points, axis=1)
    az = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    el = np.degrees(np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1])))
    pct = lambda a: [round(float(v), 1) for v in np.percentile(a, [1, 99])]
    return {"azimuth_deg_p1_p99": pct(az), "elevation_deg_p1_p99": pct(el), "range_m_p1_p99": pct(r)}


def rotation_difference(a, b):
    """The angle between two rotation matrices, in rad."""
    return math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1.0) / 2.0)))


def windows(points, times, window):
    """For each cloud, it and the clouds taken in up to window seconds before it, together."""
    return [np.concatenate([p for p, t in zip(points, times) if now - window <= t <= now]) for now in times]


def analyse(args, clouds, imu_rpy, body_height, odom_poses):
    results = {"body_height_m": body_height, "imu_roll_pitch": [round(v, 4) for v in imu_rpy[:2]]}
    roll, pitch = imu_rpy[0], imu_rpy[1]
    rec = next(r for r in clouds if r.topic == args.topic)
    if not rec.messages:
        return results, None, f"no clouds arrived on {args.topic}"
    frame = cloud_frame(rec.messages[-1])
    raw = [decode_points(m) for m in rec.messages]
    results["fit_topic"], results["fit_frame"] = rec.topic, frame
    results["view_in_frame"] = view_in_frame(np.concatenate(raw))
    if body_height is None:
        return results, None, "no body height: sport mode isn't publishing rt/sportmodestate; pass --body-height"
    h = body_height

    if frame in WORLD_FRAMES:
        # The cloud is already placed in the world, so no mount is needed while odometry publishes.
        if odom_poses is None:
            return results, None, f"clouds are in frame {frame!r}, but rt/utlidar/robot_odom isn't arriving"
        points = [to_level_base(p, frame, roll, pitch, None, odom_poses) for p in raw]
        mount, problem = None, None
        results["mount_note"] = f"frame {frame!r} is a world frame: run_policy.py needs odometry, not a mount"
    elif frame in BODY_FRAMES:
        points = [to_level_base(p, frame, roll, pitch) for p in raw]
        mount, problem = None, None
        results["mount_note"] = f"frame {frame!r} is the body frame: no mount needed"
    else:
        rng = np.random.default_rng(0)
        allp = np.concatenate(raw)
        sample = allp[rng.choice(len(allp), min(FIT_POINTS, len(allp)), replace=False)]
        plane = fit_plane(sample, rng)
        if plane is None:
            return results, None, f"no floor plane {LIDAR_HEIGHT_RANGE[0]}-{LIDAR_HEIGHT_RANGE[1]} m from the lidar"
        normal, lidar_height, inliers = plane
        results["floor"] = {"normal_in_lidar_frame": normal.round(4).tolist(), "lidar_height_m": round(float(lidar_height), 4),
                            "share_of_points": round(float(inliers.mean()), 3),
                            "rms_m": round(float(np.sqrt(np.mean((sample[inliers] @ normal + lidar_height) ** 2))), 4)}
        cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
        up = np.array([-sp, cp * sr, cp * cr])  # the world's up, in the body frame
        choices = []
        for yaw in (0.0, math.pi):
            r_m, p_m, err = mount_rotation(normal, up, yaw)
            # tz from the floor: up . t = lidar height - body height (tx is found below, if there's a board).
            choices.append([yaw, r_m, p_m, err])
        choices.sort(key=lambda c: rotation_difference(rotation_rpy(c[1], c[2], c[0]), URDF_MOUNT.rotation()))
        measured = {"roll_pitch": True, "height": True, "forward": False, "yaw": False}
        yaw, r_m, p_m, err = choices[0]
        tx = URDF_MOUNT.xyz[0]
        wall = None
        if args.wall_distance:
            # Facing forward, as in Unitree's model, only: turned round, something behind the robot
            # can pass for the board.
            m = Mount(frame, (0.0, 0.0, (lidar_height - h) / up[2]), (r_m, p_m, yaw))
            wall = find_wall(np.concatenate([to_level_base(p, frame, roll, pitch, m) for p in raw]), h)
            if wall is None:
                results["wall"] = f"no board found {WALL_HALF_WIDTH} m either side of straight ahead"
            else:
                tx = args.wall_distance - wall[0]
                measured.update(forward=True, yaw=True)
                results["wall"] = {"seen_at_m_before_offset": round(wall[0], 4), "points": int(len(wall[2])),
                                   "board_yaw_deg": round(math.degrees(wall[1]), 2),
                                   "note": "board_yaw_deg is the board's angle to the lidar; a square board "
                                           "means the lidar is turned by about minus that"}
        tz = (lidar_height - h - tx * up[0]) / up[2]
        mount = Mount(frame, (round(tx, 4), 0.0, round(float(tz), 4)), (round(r_m, 4), round(p_m, 4), yaw), measured)
        results["mount"] = mount.as_dict()
        results["mount_fit_error"] = round(err, 5)
        results["mount_vs_urdf_deg"] = round(math.degrees(rotation_difference(mount.rotation(),
                                                                               URDF_MOUNT.rotation())), 2)
        if not measured["forward"]:
            results["mount_note"] = (f"forward position and which way it faces not measured (no --wall-distance or "
                                     f"no board): x is Unitree's {URDF_MOUNT.xyz[0]:g} m, facing forward as in its "
                                     f"model")
        points = [to_level_base(p, frame, roll, pitch, mount) for p in raw]
        problem = None

    # The check: the env vector from each window of clouds, as run_policy.py would build it.
    # The robot stands still, so the clouds need no turning.
    together = windows(points, rec.times[-len(points):], args.window)
    layout = EnvLayout()
    envs, fallbacks = [], []
    for p in together:
        env, info = build_env(p, layout, h)
        envs.append(env)
        fallbacks.append(info["fallback"])
    envs, fallbacks = np.array(envs), np.array(fallbacks)
    beams = np.array([build_env(p, PUSH_BOX_TURN, h)[0] for p in together])
    results["push_box_turn_beams"] = {"mean": beams.mean(axis=0).round(3).tolist(),
                                      "lit_share": (beams[:, :PUSH_BOX_TURN.n_probes] > 0).mean(axis=0).round(2).tolist()}
    results["window_s"] = args.window
    drop = (h + RAY_START_HEIGHT) / OPEN_OBSTACLE_DISTANCE
    expect_object = args.wall_distance * math.sqrt(1 + drop * drop) if args.wall_distance else OPEN_OBSTACLE_DISTANCE
    probe_error = np.abs(envs[:, :3] + h)
    probe_error[fallbacks[:, :3]] = np.nan
    results["check"] = {
        "env_mean": envs.mean(axis=0).round(4).tolist(), "env_std": envs.std(axis=0).round(4).tolist(),
        "expected": [-h, -h, -h, 2.0, round(expect_object, 3), 1.0],
        "probe_fallback_share": fallbacks[:, :3].mean(axis=0).round(3).tolist(),
        "probe_error_m": [None if np.isnan(v) else round(float(v), 4) for v in np.nanmedian(probe_error, axis=0)]
        if np.isfinite(probe_error).any() else None,
        "object_dist_error_m": round(float(np.median(envs[:, 4]) - expect_object), 4),
    }
    allpts = np.concatenate(points)
    floor_ahead = allpts[(np.abs(allpts[:, 2] + h) < 0.03) & (np.abs(allpts[:, 1]) < 0.15) & (allpts[:, 0] > 0)]
    results["nearest_floor_ahead_m"] = round(float(floor_ahead[:, 0].min()), 3) if len(floor_ahead) else None
    return results, mount, problem


def verdict(results, mount, problem):
    lines = []
    if problem:
        return False, [f"NOT MEASURED: {problem}"]
    check = results.get("check", {})
    if mount is not None:
        lines.append(f"Mount ({mount.frame!r} in the base frame): xyz {list(mount.xyz)} m, "
                     f"rpy {[round(math.degrees(v), 2) for v in mount.rpy]} deg; measured {mount.measured}")
    if results.get("mount_note"):
        lines.append(f"Note: {results['mount_note']}")
    if isinstance(results.get("wall"), str):
        lines.append(f"Board: {results['wall']}")
    ok = True
    errs = check.get("probe_error_m")
    if errs is None:
        lines.append("Probes: no points at any probe spot (all fell back to flat floor)")
        ok = False
    else:
        bad = [e for e in errs if e is not None and e > EXPECTED_TOL["probe"]]
        lines.append(f"Probes: error {errs} m against minus the body height "
                     f"(fell back on {check['probe_fallback_share']} of clouds)")
        ok &= not bad
    lines.append(f"object_dist: {check['env_mean'][4]:.3f} m, expected {check['expected'][4]:.3f} m")
    ok &= abs(check["object_dist_error_m"]) <= EXPECTED_TOL["object_dist"] or not results.get("wall")
    lines.append(f"Nearest floor seen ahead: {results.get('nearest_floor_ahead_m')} m (the probes are 0.30 m ahead)")
    beams = results.get("push_box_turn_beams")
    if beams:
        lines.append(f"push_box_turn beams +30..-30 deg: {beams['mean'][:7]}, lit in {beams['lit_share']} of windows, "
                     f"object_dist {beams['mean'][8]:.2f} m. With the crate dead ahead 2 m away, beams 2-4 should "
                     f"read about 0.55 and object_dist about 1.8 m")
    return ok, lines


# --------------------------------------------------------------------------- main


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", metavar="INTERFACE", nargs="?", const=DEFAULT_INTERFACE,
                        help="the robot, on this network interface (if none is given: %(const)s); "
                             "without --robot, go2/sim_headless.py")
    parser.add_argument("--domain", type=int, help="DDS domain id (default: 0 with --robot, otherwise 1)")
    parser.add_argument("--seconds", type=float, default=SECONDS, help="how long to record (default: %(default)s)")
    parser.add_argument("--wall-distance", type=float, metavar="M",
                        help="a flat board stands square to the robot, its face this far ahead of the robot's centre")
    parser.add_argument("--body-height", type=float, metavar="M",
                        help="the base's height above the floor; default: sport mode's body_height")
    parser.add_argument("--topic", choices=CLOUD_TOPICS, default="rt/utlidar/cloud",
                        help="the cloud to measure the mount of (default: %(default)s, the one run_policy.py reads "
                             "with sport mode off)")
    parser.add_argument("--window", type=float, default=CLOUD_WINDOW_S, metavar="SECONDS",
                        help="the check builds each env vector from this many seconds of clouds, as run_policy.py "
                             "does (default: %(default)s)")
    parser.add_argument("--mount-out", type=Path, default=MOUNT_FILE,
                        help="where to save a measured mount (default: go2/logs/lidar_mount.json)")
    parser.add_argument("--log", type=Path, help="where to save (default: go2/logs/lidar/<time>.npz, and .json)")
    return parser.parse_args()


def main():
    args = parse_args()
    interface = args.robot or "lo"
    domain = args.domain if args.domain is not None else (0 if args.robot else 1)
    if args.robot and not check_interface(args.robot):
        return 2
    ChannelFactoryInitialize(domain, interface)
    clouds = [Recorder(topic, PointCloud2_, depth=8) for topic in CLOUD_TOPICS]
    lowstate = Recorder("rt/lowstate", LowState_)
    sport = Recorder("rt/sportmodestate", SportModeState_)
    odom = Recorder(ODOM_TOPIC, Odometry_)
    lidar_state = Recorder(LIDAR_STATE_TOPIC, LidarState_, depth=8)
    height_map = Recorder(HEIGHT_MAP_TOPIC, HeightMap_, depth=8)
    print(f"Recording the lidar for {args.seconds:g} s on {interface}, domain {domain}. Read-only. "
          f"Keep the robot still and stand clear.", flush=True)
    rpy, heights, poses = [], [], []
    end = time.perf_counter() + args.seconds
    while time.perf_counter() < end:
        now = time.perf_counter()
        for rec in clouds:
            rec.poll(now, keep=KEEP_CLOUDS)
        for rec in (lidar_state, height_map):
            rec.poll(now, keep=2)
        lowstate.poll(now, keep=1)
        sport.poll(now, keep=1)
        odom.poll(now, keep=1)
        if lowstate.messages:
            rpy.append(quat_rpy(*lowstate.messages[-1].imu_state.quaternion))
        if sport.messages:
            heights.append(sport.messages[-1].body_height)
        if odom.messages:
            p, q = odom.messages[-1].pose.pose.position, odom.messages[-1].pose.pose.orientation
            poses.append((p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        time.sleep(POLL_PERIOD)

    topics = describe_topics(clouds, lidar_state, height_map, odom)
    for topic, info in topics.items():
        print(f"  {topic}: {info['messages']} messages" + (f", {info['rate_hz']:.1f} Hz" if info.get("rate_hz") else "")
              + (f", frames {info['frames']}, {info['points_median']} points" if info.get("frames") else ""), flush=True)
    if not rpy:
        print("No rt/lowstate: is the robot (or go2/sim_headless.py) running?")
        return 2
    imu = tuple(np.median(np.array(rpy), axis=0))
    body_height = args.body_height if args.body_height is not None else (float(np.median(heights)) if heights else None)
    odom_pose = tuple(np.median(np.array(poses), axis=0)) if poses else None
    try:
        results, mount, problem = analyse(args, clouds, imu, body_height, odom_pose)
    except NoReading as error:
        results, mount, problem = {}, None, str(error)
    results["topics"] = topics
    results["body_height_source"] = "--body-height" if args.body_height is not None else "rt/sportmodestate body_height"
    results["wall_distance_m"] = args.wall_distance
    ok, lines = verdict(results, mount, problem)
    results["ok"], results["summary"] = ok, lines

    path = args.log or LOG_DIR / f"{datetime.now():%Y-%m-%d_%H-%M-%S}.npz"
    path.parent.mkdir(parents=True, exist_ok=True)
    saved = {}
    for i, rec in enumerate(clouds):
        if rec.messages:
            pts = [decode_points(m).astype(np.float32) for m in rec.messages]
            saved[f"cloud{i}_points"] = np.concatenate(pts)
            saved[f"cloud{i}_offsets"] = np.cumsum([0] + [len(p) for p in pts])
            saved[f"cloud{i}_stamp"] = np.array([cloud_stamp(m) for m in rec.messages])
            saved[f"cloud{i}_frame"] = np.array([cloud_frame(m) for m in rec.messages])
    np.savez_compressed(path, meta=json.dumps({"topics": list(CLOUD_TOPICS)}), imu_rpy=np.array(rpy),
                        body_height=np.array(heights), odom_pose=np.array(poses).reshape(-1, 7), **saved)
    json_path = path.with_suffix(".json")
    json_path.write_text(json.dumps({"results": results}, indent=1, default=str))
    placed = mount is not None or (not problem and results.get("fit_frame") in WORLD_FRAMES | BODY_FRAMES)
    good = ok and placed
    print("\n" + "\n".join(lines))
    print(f"\n{'OK' if good else 'CHECK THE ABOVE'}. Saved {json_path} and {path.name}")
    if mount is not None:
        args.mount_out.parent.mkdir(parents=True, exist_ok=True)
        args.mount_out.write_text(json.dumps(dict(mount.as_dict(), measured_by=str(json_path)), indent=1))
        print(f"Mount saved to {args.mount_out}, for go2/run_policy.py")
    if problem and not any(rec.messages for rec in clouds):
        return 2
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
