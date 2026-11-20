#!/usr/bin/env python
"""Summarise what the lidar sent during a go2/run_policy.py run. Never talks to the robot.

    python go2/lidar_report.py go2/logs/<time>.npz          # reads <time>_lidar.npz beside it too

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


def stage_spans(log):
    """(stage, first t, last t) in the run log's clock, in order."""
    stage, t = log["stage"].astype(str), log["t"]
    starts = np.flatnonzero(np.r_[True, stage[1:] != stage[:-1]])
    ends = np.r_[starts[1:], len(stage)] - 1
    return [(stage[a], t[a], t[b]) for a, b in zip(starts, ends)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", type=Path, help="a go2/run_policy.py log (go2/logs/<time>.npz)")
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
             if lidar_meta.get("mount") else "none (only body- and odom-frame clouds can be used)"))
    for problem, n in (lidar_meta.get("problems") or {}).items():
        print(f"  unusable x{n}: {problem}")
    if lidar_meta.get("error"):
        print(f"  lidar process error: {lidar_meta['error']}")
    topics = lidar_meta["topics"]

    if readings_path.exists():
        r = np.load(readings_path, allow_pickle=False)
        clock = meta["clock_start"]
        print(f"\nClouds per stage ({readings_path.name}; frames {json.loads(str(r['meta']))['frames']}):")
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
    t = log["t"]
    dt = np.diff(t) * 1000.0
    print(f"\nControl loop: cycle {np.median(dt):.2f} ms median, {np.percentile(dt, 99):.2f} ms p99, "
          f"{np.mean(dt > 4.0):.2%} over 4 ms, worst {dt.max():.1f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
