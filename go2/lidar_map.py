#!/usr/bin/env python
"""Show the lidar height map and the seven policy beams. READ-ONLY.

This script only subscribes to topics. It creates no publishers and no SDK
clients, so it can't command the robot or switch the lidar on or off.

    python go2/lidar_map.py [interface]                   # live window; close it to stop
    python go2/lidar_map.py [interface] --save map.png    # one picture, with no display needed

The interface defaults to DEFAULT_INTERFACE in check_connection.py.

The left panel is the whole map and the right panel is the area around the robot.
Heights are relative to the base, like the beam heights from check_lidar.py, and
grey cells are unknown. It draws cells where check_lidar.py reads them, so a shift
in the picture would also be a shift in that script's beam readings.

Exit status: 0 = OK, 1 = no usable map, 2 = bad network interface or no display.
"""
import argparse
import sys
import time

import matplotlib
import numpy as np

from check_connection import DEFAULT_INTERFACE, TopicMonitor, check_interface
from check_lidar import (
    BEAM_AHEAD,
    BEAM_PAIRS,
    HEIGHT_MAP_TOPIC,
    MAX_PLAUSIBLE_HEIGHT,
    ODOM_TOPIC,
    NoReading,
    beam_locations,
    read_height_map,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_

# Fixed so that colours mean the same thing in every picture. In S-M-ICRL's simulation,
# a beam reads about -0.27 on the platform and -0.57 past its edge.
COLOUR_RANGE = (-0.7, 0.1)  # m, relative to the base
ZOOM_HALF_WIDTH = 1.2  # m
CENTRE_BEAM = 3
REFRESH_SECONDS = 0.2
STALE_SECONDS = 2.0
COMMON_VALUES_SHOWN = 5


def read_map(msg, odom):
    """Return what the panels need from a map message. Raises NoReading if it can't be read."""
    beam_heights = read_height_map(msg, odom, BEAM_AHEAD)
    frame = msg.frame_id.strip("/")
    beams, base_height = beam_locations(frame, odom, BEAM_AHEAD)
    base = beam_locations(frame, odom, 0.0)[0][CENTRE_BEAM]

    cells = np.asarray(msg.data, dtype=np.float64).reshape(msg.height, msg.width)
    known = np.isfinite(cells) & (np.abs(cells) < MAX_PLAUSIBLE_HEIGHT)
    half_cell = msg.resolution / 2.0
    # Each cell's centre is at origin + index * resolution, as check_lidar.py reads it.
    extent = (msg.origin[0] - half_cell, msg.origin[0] + msg.width * msg.resolution - half_cell,
              msg.origin[1] - half_cell, msg.origin[1] + msg.height * msg.resolution - half_cell)
    return {
        "frame": frame,
        "cells": cells,
        "known": known,
        "relative": np.where(known, cells - base_height, np.nan),
        "extent": extent,
        "beams": beams,
        "beam_heights": beam_heights,
        "base": base,
        "base_height": base_height,
    }


def print_common_values(cells):
    """Print the most repeated cell values, because a marker for unknown cells would repeat."""
    values, counts = np.unique(cells, return_counts=True)
    print(f"Most common values in the first map of {cells.size} cells:")
    for i in np.argsort(-counts)[:COMMON_VALUES_SHOWN]:
        print(f"  {values[i]:12g}: {counts[i]} cells")


def draw_panel(axes, cmap, view, label_beams):
    axes.clear()
    axes.imshow(view["relative"], origin="lower", extent=view["extent"], cmap=cmap,
                vmin=COLOUR_RANGE[0], vmax=COLOUR_RANGE[1], interpolation="nearest")
    base, centre = view["base"], view["beams"][CENTRE_BEAM]
    axes.annotate("", xy=centre, xytext=base, arrowprops={"arrowstyle": "->", "color": "red", "lw": 2})
    axes.plot(*base, "o", color="red", markersize=5)
    axes.plot(view["beams"][:, 0], view["beams"][:, 1], "o", markersize=4,
              markerfacecolor="white", markeredgecolor="black")
    if label_beams:
        # Ahead of the row of beams, so that a label can't sit on a neighbouring beam at any heading.
        heading = (centre - base) / np.linalg.norm(centre - base)
        for i, (x, y) in enumerate(view["beams"]):
            axes.annotate(str(i), (x, y), xytext=tuple(14 * heading), textcoords="offset points",
                          ha="center", va="center", fontsize=9, fontweight="bold", color="white",
                          bbox={"boxstyle": "round,pad=0.15", "facecolor": "black", "edgecolor": "none"})
    axes.set_xlabel(f"x in {view['frame']!r} (m)")
    axes.set_ylabel(f"y in {view['frame']!r} (m)")


def summary(view):
    known_values = view["cells"][view["known"]]
    span = f"odom heights {known_values.min():+.3f} to {known_values.max():+.3f} m, " if known_values.size else ""
    heights = view["beam_heights"]
    differences = ", ".join(f"env[{a}]-env[{b}] {heights[a] - heights[b]:+.3f}" for a, b in BEAM_PAIRS)
    return (f"{view['known'].sum()} of {view['cells'].size} cells known, {span}base at {view['base_height']:+.3f} m\n"
            f"{differences}")


def draw(figure, panels, cmap, view, warning):
    whole, zoom = panels
    draw_panel(whole, cmap, view, label_beams=False)
    whole.set_title("Whole map")
    draw_panel(zoom, cmap, view, label_beams=True)
    zoom.set_xlim(view["base"][0] - ZOOM_HALF_WIDTH, view["base"][0] + ZOOM_HALF_WIDTH)
    zoom.set_ylim(view["base"][1] - ZOOM_HALF_WIDTH, view["base"][1] + ZOOM_HALF_WIDTH)
    zoom.set_title("Around the robot (env[0] should be on its left)")
    figure.suptitle(f"{warning}\n{summary(view)}" if warning else summary(view),
                    color="red" if warning else "black")


def wait_for_map(height_map, odom, seconds):
    """Return the first readable map, or None. Prints why if there isn't one."""
    deadline = time.monotonic() + seconds
    problem = f"no {HEIGHT_MAP_TOPIC} received"
    while time.monotonic() < deadline:
        msg, odom_msg = height_map.snapshot()[1], odom.snapshot()[1]
        if msg is not None:
            try:
                return read_map(msg, odom_msg)
            except NoReading as reason:
                problem = str(reason)
        time.sleep(0.05)
    print(f"FAIL: {problem}. Is the lidar on, and are the interface and --domain right?")
    return None


def show_live(figure, panels, cmap, view, height_map, odom, plt):
    """Redraw until the window closes."""
    last_count, last_change, warning = height_map.snapshot()[0], time.monotonic(), ""
    draw(figure, panels, cmap, view, warning)
    while plt.fignum_exists(figure.number):
        plt.pause(REFRESH_SECONDS)
        count, msg = height_map.snapshot()
        if count != last_count:
            last_count, last_change = count, time.monotonic()
            try:
                view, warning = read_map(msg, odom.snapshot()[1]), ""
            except NoReading as reason:
                warning = f"Showing an old map: {reason}"
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
    args = parser.parse_args()
    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # after choosing the backend

    if not args.save and matplotlib.get_backend().lower() == "agg":
        print("ERROR: no display for a window. Use --save map.png, or check DISPLAY and that tkinter is installed.")
        return 2
    if not check_interface(args.interface):
        return 2

    ChannelFactoryInitialize(args.domain, args.interface)
    height_map, odom = TopicMonitor(), TopicMonitor()
    # Kept in a variable because a live window can run for a long time.
    subscribers = [ChannelSubscriber(HEIGHT_MAP_TOPIC, HeightMap_), ChannelSubscriber(ODOM_TOPIC, Odometry_)]
    for subscriber, monitor in zip(subscribers, (height_map, odom)):
        subscriber.Init(monitor, 10)

    print(f"Listening on {args.interface}, domain {args.domain} (read-only)...")
    view = wait_for_map(height_map, odom, args.seconds)
    if view is None:
        return 1
    print_common_values(view["cells"])

    figure, panels = plt.subplots(1, 2, figsize=(13, 7), constrained_layout=True)
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
            show_live(figure, panels, cmap, view, height_map, odom, plt)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
