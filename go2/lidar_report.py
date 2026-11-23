#!/usr/bin/env python
"""Summarise what the lidar sent during a go2/run_policy.py run. Never talks to the robot.

    python go2/lidar_report.py go2/logs/<time>.npz          # reads <time>_lidar.npz beside it too
    python go2/lidar_report.py go2/logs/<time>.npz --replay 0,0.4,0.8
                                     # rebuild the walk's env vectors from its saved clouds,
                                     # with these accumulation windows (s), to compare them

It answers the questions the lidar's first robot runs are for:

  per stage   how many clouds came on each topic before the run (sport mode on), and in
              each stage after it (sport mode off): does the lidar keep publishing?
  walking     the env vector the policy got against the lidar's, column by column; how
              often a value fell back to flat floor; how old the clouds were; the nearest
              object it saw ahead
  timing      the control loop's cycle times, to compare with runs from before the lidar
              was read

Exit status: 0 = read, 1 = no lidar readings in the log.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from lidar_env import CloudEnv, EnvLayout, Mount


def stage_spans(log):
    """(stage, first t, last t) in the run log's clock, in order."""
    stage, t = log["stage"].astype(str), log["t"]
    starts = np.flatnonzero(np.r_[True, stage[1:] != stage[:-1]])
    ends = np.r_[starts[1:], len(stage)] - 1
    return [(stage[a], t[a], t[b]) for a, b in zip(starts, ends)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", type=Path, help="a go2/run_policy.py log (go2/logs/<time>.npz)")
    parser.add_argument("--replay", metavar="LIST",
                        help="comma-separated accumulation windows (s): rebuild the walk's env vectors from its "
                             "saved clouds with each, using the run's mount (or --mount-from)")
    parser.add_argument("--mount-from", type=Path, metavar="JSON",
                        help="with --replay: take the lidar mount from this calibration file instead "
                             "(go2/logs/calibration/latest.json)")
    args = parser.parse_args()
    log = np.load(args.log, allow_pickle=False)
    meta = json.loads(str(log["meta"]))
    lidar_meta = meta.get("lidar")
    readings_path = args.log.with_name(args.log.stem + "_lidar.npz")
    print(f"Run {args.log.name}: {meta.get('outcome')}; env source {meta.get('env_source', 'flat (before lidar support)')}")
    if lidar_meta is None:
        print("No lidar in this run (--no-lidar, or from before lidar support).")
        return 1
    print(f"  layout {lidar_meta['layout']['mode']}, mount "
          + (f"{lidar_meta['mount']['xyz']} m, rpy {lidar_meta['mount']['rpy']} in {lidar_meta['mount']['frame']!r}"
             if lidar_meta.get("mount") else "none (only body- and odom-frame clouds can be used)")
          + (f" ({lidar_meta['mount']['measured'].get('source')})"
             if lidar_meta.get("mount") and lidar_meta["mount"].get("measured", {}).get("source") else "")
          + (f"; clouds of the last {lidar_meta['window_s']:g} s" if lidar_meta.get("window_s") is not None else ""))
    for problem, n in (lidar_meta.get("problems") or {}).items():
        print(f"  unusable x{n}: {problem}")
    if lidar_meta.get("error"):
        print(f"  lidar process error: {lidar_meta['error']}")
    topics = lidar_meta["topics"]

    if readings_path.exists():
        r = np.load(readings_path, allow_pickle=False)
        clock = meta["clock_start"]
        print(f"\nClouds per stage ({readings_path.name}; frames {json.loads(str(r['meta']))['frames']}):")
        if "env_clouds_used" in r.files and len(r["env_clouds_used"]):
            print(f"  each env vector built from {np.median(r['env_clouds_used']):.0f} clouds (median)")
        if "arrival_t" in r.files:
            at, topic, usable, points = r["arrival_t"] - clock, r["arrival_topic"], r["arrival_usable"], r["arrival_points"]
            spans = [("before the run (sport mode on)", -np.inf, stage_spans(log)[0][1])]
            spans += stage_spans(log)
            spans[-1] = (spans[-1][0], spans[-1][1], np.inf)  # and afterwards, until the process stopped
            for name, t0, t1 in spans:
                sel = (at >= t0) & (at < t1) if np.isfinite(t1) else at >= t0
                parts = []
                for i, name_t in enumerate(topics):
                    s = sel & (topic == i)
                    if s.any():
                        span = max(1e-3, (min(t1, at[s].max()) - max(t0, at[s].min()))) if s.sum() > 1 else None
                        rate = f", {(s.sum() - 1) / span:.1f} Hz" if span else ""
                        parts.append(f"{name_t} {s.sum()} ({usable[s].sum()} usable{rate}, "
                                     f"{int(np.median(points[s]))} points)")
                    else:
                        parts.append(f"{name_t} 0")
                print(f"  {name:32s} " + "; ".join(parts))
        else:
            print("  no clouds arrived at all")
    else:
        print(f"\nNo readings file {readings_path.name} (the lidar process didn't save one).")

    if "env_t" in log.files:
        given, lid = log["env_given"], log["env_lidar"]
        fresh = np.isfinite(lid).all(axis=1)
        n = given.shape[1]
        names = [f"probe{i}" for i in range(n - 3)] + ["clearance", "object_dist", "bias"]
        print(f"\nWhile walking: {len(given)} env rebuilds, {fresh.mean():.0%} with a fresh usable cloud")
        if fresh.any():
            fb = log["env_fallback"][fresh].mean(axis=0)
            age = log["env_cloud_age"][fresh] * 1000.0
            print(f"  cloud age {np.median(age):.0f} ms median, {np.percentile(age, 95):.0f} ms p95, "
                  f"{age.max():.0f} ms worst; {np.median(log['env_cloud_points'][fresh]):.0f} points after the self-filter")
            print(f"  {'value':12s} {'given mean':>11s} {'lidar mean':>11s} {'lidar min':>10s} {'lidar max':>10s} {'fell back':>10s}")
            for i, name in enumerate(names):
                g, l = given[fresh, i], lid[fresh, i]
                print(f"  {name:12s} {g.mean():+11.3f} {l.mean():+11.3f} {l.min():+10.3f} {l.max():+10.3f} {fb[i]:10.0%}")
            hits = log["env_object_hit"][fresh]
            print(f"  object ahead on {hits.mean():.0%} of rebuilds" +
                  (f", nearest {lid[fresh, n - 2].min():.2f} m" if hits.any() else ""))
            # object_dist over the walk, so a box coming closer shows as falling numbers
            et = log["env_t"][fresh]
            et = et - et[0]
            od = lid[fresh, n - 2]
            line = []
            for a in np.arange(0.0, et[-1] + 1e-9, 0.5):
                sel = (et >= a) & (et < a + 0.5)
                if sel.any():
                    v = od[sel].min()
                    line.append(f"{a:.1f}s {'-' if v >= 10.0 else f'{v:.2f}'}")
            print("  object_dist, nearest per 0.5 s ('-' = nothing): " + ", ".join(line))
    if args.replay:
        replay(args, log, meta, lidar_meta, readings_path)
    t = log["t"]
    dt = np.diff(t) * 1000.0
    print(f"\nControl loop: cycle {np.median(dt):.2f} ms median, {np.percentile(dt, 99):.2f} ms p99, "
          f"{np.mean(dt > 4.0):.2%} over 4 ms, worst {dt.max():.1f} ms")
    return 0


def replay(args, log, meta, lidar_meta, readings_path):
    """Rebuild the walk's env vectors from the saved clouds, once per window."""
    if not readings_path.exists():
        print("\nReplay: no readings file")
        return
    r = np.load(readings_path, allow_pickle=False)
    if "cloud_t" not in r.files:
        print("\nReplay: no clouds were saved while walking")
        return
    mount = Mount.from_dict(lidar_meta.get("mount"))
    if args.mount_from:
        mount = Mount.from_dict(json.loads(args.mount_from.read_text()).get("lidar_mount"))
    if mount is None:
        print("\nReplay: no lidar mount (give --mount-from)")
        return
    layout = EnvLayout(**lidar_meta["layout"])
    t, clock = log["t"], meta["clock_start"]
    if "cloud_pose" in r.files:
        poses, heights = r["cloud_pose"], r["cloud_height"]
    else:  # from before poses were saved: rebuild them as run_policy.py does
        dt = np.diff(t, prepend=t[0])
        x, y = np.cumsum(log["est_velocity"][:, 0] * dt), np.cumsum(log["est_velocity"][:, 1] * dt)
        k = np.clip(np.searchsorted(t, r["cloud_t"] - clock), 0, len(t) - 1)
        poses = np.column_stack((x[k], y[k], log["imu_rpy"][k, 2]))
        heights = log["est_height"][k]
    topics = r["cloud_topic"]
    use = topics == np.bincount(topics).argmax()  # the topic with the most saved clouds
    offsets, points = r["cloud_offsets"], r["cloud_points"].astype(np.float64)
    frames = json.loads(str(r["meta"]))["frames"]
    frame = mount.frame if mount.frame in frames else frames[0]
    n = layout.size
    print(f"\nReplay of {use.sum()} saved clouds, mount {mount.xyz} m, rpy {mount.rpy}:")
    for window in (float(v) for v in args.replay.split(",")):
        env = CloudEnv(layout, mount, window)
        rows = []
        for i in np.flatnonzero(use):
            ok = env.take_points(points[offsets[i]:offsets[i + 1]], frame, r["cloud_t"][i], *r["cloud_rpy"][i],
                                 None, tuple(poses[i]), float(heights[i]))
            if ok:
                vector, info = env.build(float(heights[i]))
                rows.append((r["cloud_t"][i] - clock, float(heights[i]), vector, info["object_hit"], info["clouds_used"]))
        if not rows:
            print(f"  window {window:g} s: no usable clouds ({env.problem})")
            continue
        ts = np.array([row[0] for row in rows]); ts -= ts[0]
        h = np.array([row[1] for row in rows]); vec = np.array([row[2] for row in rows])
        hits = np.array([row[3] for row in rows]); used = np.median([row[4] for row in rows])
        od = vec[:, n - 2]
        line = ", ".join(f"{a:.1f}s {'-' if od[(ts >= a) & (ts < a + 0.5)].min() >= 10.0 else f'{od[(ts >= a) & (ts < a + 0.5)].min():.2f}'}"
                         for a in np.arange(0.0, ts[-1] + 1e-9, 0.5) if ((ts >= a) & (ts < a + 0.5)).any())
        probe = ""
        if layout.mode == "terrain":
            probe = f"probes minus (-height) {np.median(vec[:, :n - 3] + h[:, None], axis=0).round(3).tolist()}; "
        print(f"  window {window:g} s ({used:.0f} clouds): {probe}object hit on {hits.sum()}/{len(rows)}; "
              f"nearest per 0.5 s: {line}")


if __name__ == "__main__":
    sys.exit(main())
