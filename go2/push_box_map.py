#!/usr/bin/env python
"""Show the lidar height map and push_box_turn's seven crate beams. READ-ONLY.

This script only subscribes to topics. It creates no publishers and no SDK
clients, so it can't command the robot or switch the lidar on or off.

    python go2/push_box_map.py [interface]                   # live window; close it to stop
    python go2/push_box_map.py [interface] --save crate.png  # one picture, with no display needed

The interface defaults to DEFAULT_INTERFACE in check_connection.py.

push_box_turn (func_config/go2/push_box/push_box_turn.yaml in S-M-ICRL) senses in
obstacle_bearing mode. Each beam is a 10 deg fan of 5 near-level rays, at bearings
+30 to -30 deg (env[0] is the left one). RobotAPI._object_distance_ahead casts each
ray from 0.10 m above the base, dropping so that it would meet open floor 10 m out.
A beam reads max(0, 1 - d / 4) for the nearest hit d over its rays, and env[8]
(object_dist) is the nearest hit over all of them.

Here each ray is marched across the height map instead, and it hits the first cell
that is higher than the ray. Unknown cells count as empty, so the picture and the
"known" column show how much of each beam the lidar actually saw.

Exit status: 0 = OK, 1 = no usable map, 2 = bad network interface or no display.
"""
import argparse
import math
import sys
import time

import matplotlib
import numpy as np

from check_connection import DEFAULT_INTERFACE, TopicMonitor, check_interface
from check_lidar import HEIGHT_MAP_TOPIC, MAX_PLAUSIBLE_HEIGHT, ODOM_TOPIC, NoReading, beam_locations
from lidar_map import REFRESH_SECONDS, STALE_SECONDS, print_common_values
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_

# func_config/go2/push_box/push_box_turn.yaml and world/robot_api.py in S-M-ICRL.
BEARINGS_DEG = np.array([30.0, 20.0, 10.0, 0.0, -10.0, -20.0, -30.0])
SECTOR_DEG = 10.0
RAYS_PER_SECTOR = 5
PROBE_RANGE = 4.0  # m; a beam reads 0 beyond this
MAX_DIST = 10.0  # m; the ray meets open floor here
START_HEIGHT = 0.10  # m above the base
NOMINAL_BASE_HEIGHT = 0.27  # m above the floor; push_box_turn's target height

# The crate is 0.45 m tall and the base stands at 0.27 m, so it is +0.18 m; the floor is -0.27 m.
COLOUR_RANGE = (-0.45, 0.25)  # m, relative to the base
FLOOR_RADIUS = 1.0  # m; the floor height is the median of known cells this close to the base
ZOOM_HALF_WIDTH = PROBE_RANGE + 0.3  # m
PRINT_SECONDS = 1.0

RAY_OFFSETS = np.linspace(-0.5, 0.5, RAYS_PER_SECTOR) * SECTOR_DEG


def base_pose(frame, odom):
    """Return the base's x, y, heading and height in the map's frame."""
    base = beam_locations(frame, odom, 0.0)[0][3]
    ahead, base_height = beam_locations(frame, odom, 1.0)
    forward = ahead[3] - base
    return base, math.atan2(forward[1], forward[0]), base_height


def floor_height(cells, known, extent, base, resolution):
    """Median of known cells near the base, or None if there are none."""
    ys, xs = np.mgrid[0:cells.shape[0], 0:cells.shape[1]]
    near = np.hypot(extent[0] + (xs + 0.5) * resolution - base[0],
                    extent[2] + (ys + 0.5) * resolution - base[1]) < FLOOR_RADIUS
    values = cells[near & known]
    return float(np.median(values)) if values.size else None


def cast_beams(msg, cells, known, base, yaw, ray_start_height, floor, skip):
    """March every ray across the map.

    Returns per ray (beam, ray): the hit distance (inf if none within PROBE_RANGE),
    and the share of its samples that fell on known cells.
    """
    step = msg.resolution / 2.0
    s = np.arange(skip, PROBE_RANGE + step / 2.0, step)
    drop = (ray_start_height - floor) / MAX_DIST
    ray_height = ray_start_height - drop * s
    slant = math.sqrt(1.0 + drop * drop)  # mj_ray returns distance along the ray, not along the floor

    hits = np.full((len(BEARINGS_DEG), RAYS_PER_SECTOR), np.inf)
    seen = np.zeros_like(hits)
    for b, bearing in enumerate(BEARINGS_DEG):
        for r, offset in enumerate(RAY_OFFSETS):
            heading = yaw + math.radians(bearing + offset)
            ix = np.rint((base[0] + s * math.cos(heading) - msg.origin[0]) / msg.resolution).astype(int)
            iy = np.rint((base[1] + s * math.sin(heading) - msg.origin[1]) / msg.resolution).astype(int)
            inside = (ix >= 0) & (ix < msg.width) & (iy >= 0) & (iy < msg.height)
            ok = np.zeros(s.shape, dtype=bool)
            ok[inside] = known[iy[inside], ix[inside]]
            hit = np.zeros(s.shape, dtype=bool)
            hit[ok] = cells[iy[ok], ix[ok]] > ray_height[ok]
            if hit.any():
                first = int(np.argmax(hit))
                hits[b, r] = s[first] * slant
                seen[b, r] = ok[: first + 1].mean()
            else:
                seen[b, r] = ok.mean()
    return hits, seen


def read_map(msg, odom, base_height_override, skip):
    """Return what the panels need from a map message. Raises NoReading if it can't be read."""
    cells = np.asarray(msg.data, dtype=np.float64)
    if msg.resolution <= 0.0 or cells.size != msg.width * msg.height:
        raise NoReading(f"malformed map: {msg.width} x {msg.height} cells of {msg.resolution} m, {cells.size} values")
    cells = cells.reshape(msg.height, msg.width)
    frame = msg.frame_id.strip("/")
    base, yaw, base_height = base_pose(frame, odom)
    known = np.isfinite(cells) & (np.abs(cells) < MAX_PLAUSIBLE_HEIGHT)
    half_cell = msg.resolution / 2.0
    # Each cell's centre is at origin + index * resolution, as check_lidar.py reads it.
    extent = (msg.origin[0] - half_cell, msg.origin[0] + msg.width * msg.resolution - half_cell,
              msg.origin[1] - half_cell, msg.origin[1] + msg.height * msg.resolution - half_cell)

    if base_height_override is not None:
        floor, floor_source = base_height - base_height_override, "--base-height"
    else:
        floor = floor_height(cells, known, extent, base, msg.resolution)
        floor_source = f"median within {FLOOR_RADIUS:g} m"
        if floor is None:
            floor, floor_source = base_height - NOMINAL_BASE_HEIGHT, "nominal, no cells near the base"

    hits, seen = cast_beams(msg, cells, known, base, yaw, base_height + START_HEIGHT, floor, skip)
    nearest = hits.min(axis=1)
    return {
        "frame": frame,
        "cells": cells,
        "known": known,
        "relative": np.where(known, cells - base_height, np.nan),
        "extent": extent,
        "base": base,
        "yaw": yaw,
        "base_height": base_height,
        "floor": floor,
        "floor_source": floor_source,
        "skip": skip,
        "hits": hits,
        "seen": seen,
        "env": np.where(np.isfinite(nearest), np.maximum(0.0, 1.0 - nearest / PROBE_RANGE), 0.0),
        "object_dist": float(nearest.min()),
    }


def format_env(view):
    values = ", ".join(f"{v:.2f}" for v in view["env"])
    dist = view["object_dist"]
    dist = f"{dist:.2f} m" if math.isfinite(dist) else f"> {PROBE_RANGE:g} m"
    known = ", ".join(f"{100 * v:.0f}" for v in view["seen"].min(axis=1))
    return f"env[0:7] = [{values}]  object_dist {dist}\nknown % (worst ray per beam) = [{known}]"


def summary(view):
    above = view["base_height"] - view["floor"]
    return (f"{view['known'].sum()} of {view['cells'].size} cells known, base {above:.3f} m above the floor "
            f"({view['floor_source']})\n{format_env(view)}")


def draw_panel(axes, cmap, view, label_beams):
    axes.clear()
    axes.imshow(view["relative"], origin="lower", extent=view["extent"], cmap=cmap,
                vmin=COLOUR_RANGE[0], vmax=COLOUR_RANGE[1], interpolation="nearest")
    base = view["base"]
    # Grey first, so that hit rays sit on top.
    for lit in (False, True):
        for b, bearing in enumerate(BEARINGS_DEG):
            for r, offset in enumerate(RAY_OFFSETS):
                d = view["hits"][b, r]
                if np.isfinite(d) != lit:
                    continue
                heading = view["yaw"] + math.radians(bearing + offset)
                direction = np.array([math.cos(heading), math.sin(heading)])
                start, end = base + view["skip"] * direction, base + min(d, PROBE_RANGE) * direction
                axes.plot(*zip(start, end), color="tab:blue" if lit else "0.85", lw=1.4 if lit else 0.8)
                if lit:
                    axes.plot(*end, "o", color="tab:blue", markersize=4)
    heading = np.array([math.cos(view["yaw"]), math.sin(view["yaw"])])
    axes.annotate("", xy=base + 0.5 * heading, xytext=base, arrowprops={"arrowstyle": "->", "color": "red", "lw": 2})
    axes.plot(*base, "o", color="red", markersize=5)
    if label_beams:
        for b, bearing in enumerate(BEARINGS_DEG):
            angle = view["yaw"] + math.radians(bearing)
            x, y = base + (PROBE_RANGE + 0.15) * np.array([math.cos(angle), math.sin(angle)])
            axes.text(x, y, f"{b}\n{view['env'][b]:.2f}", ha="center", va="center", fontsize=8,
                      fontweight="bold", color="white",
                      bbox={"boxstyle": "round,pad=0.15", "facecolor": "black", "edgecolor": "none"})
    axes.set_xlabel(f"x in {view['frame']!r} (m)")
    axes.set_ylabel(f"y in {view['frame']!r} (m)")
    axes.set_aspect("equal")


def draw(figure, panels, cmap, view, warning):
    whole, zoom = panels
    draw_panel(whole, cmap, view, label_beams=False)
    whole.set_title("Whole map")
    draw_panel(zoom, cmap, view, label_beams=True)
    zoom.set_xlim(view["base"][0] - ZOOM_HALF_WIDTH, view["base"][0] + ZOOM_HALF_WIDTH)
    zoom.set_ylim(view["base"][1] - ZOOM_HALF_WIDTH, view["base"][1] + ZOOM_HALF_WIDTH)
    zoom.set_title(f"Within {PROBE_RANGE:g} m (beam 0 should be on the robot's left)\n"
                   "blue: ray hits something; grey: nothing seen")
    figure.suptitle(f"{warning}\n{summary(view)}" if warning else summary(view),
                    color="red" if warning else "black")


def wait_for_map(height_map, odom, seconds, read):
    """Return the first readable map, or None. Prints why if there isn't one."""
    deadline = time.monotonic() + seconds
    problem = f"no {HEIGHT_MAP_TOPIC} received"
    while time.monotonic() < deadline:
        msg, odom_msg = height_map.snapshot()[1], odom.snapshot()[1]
        if msg is not None:
            try:
                return read(msg, odom_msg)
            except NoReading as reason:
                problem = str(reason)
        time.sleep(0.05)
    print(f"FAIL: {problem}. Is the lidar on, and are the interface and --domain right?")
    return None


def show_live(figure, panels, cmap, view, height_map, odom, plt, read):
    """Redraw until the window closes, printing the readings about once a second."""
    last_count, last_change, last_print, warning = height_map.snapshot()[0], time.monotonic(), 0.0, ""
    draw(figure, panels, cmap, view, warning)
    while plt.fignum_exists(figure.number):
        plt.pause(REFRESH_SECONDS)
        count, msg = height_map.snapshot()
        if count != last_count:
            last_count, last_change = count, time.monotonic()
            try:
                view, warning = read(msg, odom.snapshot()[1]), ""
            except NoReading as reason:
                warning = f"Showing an old map: {reason}"
            if not warning and last_change - last_print >= PRINT_SECONDS:
                last_print = last_change
                print(format_env(view).replace("\n", "  "), flush=True)
        elif time.monotonic() - last_change > STALE_SECONDS:
            warning = f"Showing an old map: nothing new for {time.monotonic() - last_change:.0f} s"
        else:
            continue
        draw(figure, panels, cmap, view, warning)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("interface", nargs="?", default=DEFAULT_INTERFACE,
                        help="network interface wired to the robot (default: %(default)s)")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (the robot uses 0)")
    parser.add_argument("--seconds", type=float, default=10.0, help="how long to wait for the first map")
    parser.add_argument("--save", metavar="FILE", help="save one picture to FILE and exit, instead of opening a window")
    parser.add_argument("--base-height", type=float, metavar="M",
                        help="base height above the floor, instead of estimating the floor from the map")
    parser.add_argument("--skip", type=float, default=0.0, metavar="M",
                        help="ignore the first M m of every ray, in case the map shows the robot's own body")
    args = parser.parse_args()
    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")
    if not 0.0 <= args.skip < PROBE_RANGE:
        parser.error(f"--skip must be from 0 to below {PROBE_RANGE:g}")
    if args.base_height is not None and args.base_height <= 0.0:
        parser.error("--base-height must be positive")

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # after choosing the backend

    if not args.save and matplotlib.get_backend().lower() == "agg":
        print("ERROR: no display for a window. Use --save crate.png, or check DISPLAY and that tkinter is installed.")
        return 2
    if not check_interface(args.interface):
        return 2

    def read(msg, odom_msg):
        return read_map(msg, odom_msg, args.base_height, args.skip)

    ChannelFactoryInitialize(args.domain, args.interface)
    height_map, odom = TopicMonitor(), TopicMonitor()
    # Kept in a variable because a live window can run for a long time.
    subscribers = [ChannelSubscriber(HEIGHT_MAP_TOPIC, HeightMap_), ChannelSubscriber(ODOM_TOPIC, Odometry_)]
    for subscriber, monitor in zip(subscribers, (height_map, odom)):
        subscriber.Init(monitor, 10)

    print(f"Listening on {args.interface}, domain {args.domain} (read-only)...")
    view = wait_for_map(height_map, odom, args.seconds, read)
    if view is None:
        return 1
    print_common_values(view["cells"])
    print(summary(view))

    figure, panels = plt.subplots(1, 2, figsize=(14, 7.5), constrained_layout=True)
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("0.6")
    mappable = plt.cm.ScalarMappable(matplotlib.colors.Normalize(*COLOUR_RANGE), cmap)
    figure.colorbar(mappable, ax=panels, label="height relative to the base (m); grey = unknown", shrink=0.8)

    if args.save:
        draw(figure, panels, cmap, view, "")
        figure.savefig(args.save, dpi=130)
        print(f"Saved {args.save}")
    else:
        try:
            show_live(figure, panels, cmap, view, height_map, odom, plt, read)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
