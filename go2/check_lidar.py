#!/usr/bin/env python
"""Check whether a Go2's lidar can supply S-M-ICRL's terrain beams. READ-ONLY.

This script only subscribes to topics. It creates no publishers and no SDK
clients, so it can't command the robot or switch the lidar on or off.

    python go2/check_lidar.py <interface>              # e.g. enp3s0
    python go2/check_lidar.py <interface> --seconds 30

It reports the rate of each lidar topic, then reads the seven downward beams
used by the platform_curve policies from every source it can. Keep the robot
still, because a message and the pose used to read it can be from different times.

Exit status: 0 = a source covered all seven beams, 1 = none did, 2 = bad network interface.
"""
import argparse
import math
import sys
import threading
import time
from collections import Counter

import numpy as np

from check_connection import TopicMonitor, check_interface
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LidarState_, LowState_, SportModeState_

CLOUD_TOPICS = ("rt/utlidar/cloud", "rt/utlidar/cloud_deskewed")
HEIGHT_MAP_TOPIC = "rt/utlidar/height_map_array"
LIDAR_STATE_TOPIC = "rt/utlidar/lidar_state"
ODOM_TOPIC = "rt/utlidar/robot_odom"

# func_config/go2/platform_curve/platform_curve.yaml in S-M-ICRL, left to right.
BEAM_AHEAD = 0.45
BEAM_LATERAL = np.array([0.36, 0.24, 0.12, 0.0, -0.12, -0.24, -0.36])
BEAM_PAIRS = ((0, 6), (1, 5), (2, 4))
# Half the beam spacing, so that neighbouring beams never share a point.
BEAM_RADIUS = 0.06

WORLD_FRAMES = {"odom", "map", "world"}
BODY_FRAMES = {"base", "base_link", "body"}
POINT_FIELD_FORMATS = {7: "f4", 8: "f8"}  # sensor_msgs PointField FLOAT32, FLOAT64
# Unitree's HeightMap definition doesn't say how unknown cells are marked.
MAX_PLAUSIBLE_HEIGHT = 100.0
# Share of messages that must cover every beam for a source to count as covering them.
MIN_COVERAGE = 0.9


class NoReading(Exception):
    """A message can't be turned into beam heights."""


class RateMonitor(TopicMonitor):
    """Also measure the rate from the first arrival, so DDS discovery time doesn't count."""

    def __init__(self):
        super().__init__()
        self.first_arrival = None

    def __call__(self, msg):
        if self.first_arrival is None:
            self.first_arrival = time.monotonic()
        super().__call__(msg)

    def rate(self):
        count, _ = self.snapshot()
        if count == 0:
            return 0.0
        return count / max(time.monotonic() - self.first_arrival, 1.0)


class BeamSource(RateMonitor):
    """Read the beams from every message on a topic."""

    def __init__(self, read_beams):
        super().__init__()
        self._read_beams = read_beams
        self._results_lock = threading.Lock()
        self._readings = []
        self._problems = Counter()

    def __call__(self, msg):
        super().__call__(msg)
        try:
            heights = self._read_beams(msg)
        except NoReading as reason:
            with self._results_lock:
                self._problems[str(reason)] += 1
        else:
            with self._results_lock:
                self._readings.append(heights)

    def results(self):
        with self._results_lock:
            return list(self._readings), dict(self._problems)


def beam_locations(frame, odom, ahead):
    """Return the beams' x, y in a message's frame, and the height of the base in that frame."""
    if frame in BODY_FRAMES:
        return np.column_stack((np.full(BEAM_LATERAL.shape, ahead), BEAM_LATERAL)), 0.0
    if frame not in WORLD_FRAMES:
        raise NoReading(f"frame {frame!r} isn't handled, because its pose relative to the base isn't known")
    if odom is None:
        raise NoReading(f"frame {frame!r} needs the robot's pose, but {ODOM_TOPIC} hasn't arrived")

    position, q = odom.pose.pose.position, odom.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    base = np.array([position.x, position.y])
    return base + ahead * forward + BEAM_LATERAL[:, None] * left, position.z


def decode_points(msg):
    """Return a PointCloud2's finite x, y, z points as an (N, 3) array."""
    fields = {field.name: field for field in msg.fields}
    try:
        layout = [(fields[axis].offset, POINT_FIELD_FORMATS[fields[axis].datatype]) for axis in "xyz"]
    except KeyError as missing:
        raise NoReading(f"unsupported point layout: {missing} in {sorted(fields)}") from None

    order = ">" if msg.is_bigendian else "<"
    try:
        dtype = np.dtype({
            "names": list("xyz"),
            "formats": [order + fmt for _, fmt in layout],
            "offsets": [offset for offset, _ in layout],
            "itemsize": msg.point_step,
        })
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    except ValueError as error:
        raise NoReading(f"malformed cloud: {error}") from None

    points = np.column_stack([raw[axis].astype(np.float64) for axis in "xyz"])
    return points[np.isfinite(points).all(axis=1)]


def level_points(points, rpy):
    """Rotate body-frame points to remove roll and pitch, keeping the robot's heading."""
    roll, pitch = rpy[0], rpy[1]
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rotation = np.array([[cp, sp * sr, sp * cr], [0.0, cr, -sr], [-sp, cp * sr, cp * cr]])
    return points @ rotation.T


def read_cloud(msg, odom, low_state, ahead):
    """Return each beam's height relative to the base: NaN where the cloud has no points."""
    frame = msg.header.frame_id.strip("/")
    beams, base_height = beam_locations(frame, odom, ahead)
    points = decode_points(msg)
    if frame in BODY_FRAMES:
        if low_state is None:
            raise NoReading(f"frame {frame!r} needs the IMU, but rt/lowstate hasn't arrived")
        points = level_points(points, low_state.imu_state.rpy)

    heights = np.full(BEAM_LATERAL.shape, np.nan)
    for i, (x, y) in enumerate(beams):
        near = np.hypot(points[:, 0] - x, points[:, 1] - y) < BEAM_RADIUS
        if near.any():
            heights[i] = np.median(points[near, 2]) - base_height
    return heights


def read_height_map(msg, odom, ahead):
    """Return each beam's height relative to the base: NaN outside the map or in unknown cells."""
    frame = msg.frame_id.strip("/")
    beams, base_height = beam_locations(frame, odom, ahead)
    cells = np.asarray(msg.data, dtype=np.float64)
    if msg.resolution <= 0.0 or cells.size != msg.width * msg.height:
        raise NoReading(f"malformed map: {msg.width} x {msg.height} cells of {msg.resolution} m, {cells.size} values")

    # Unitree's HeightMap: cell [ix, iy] is at origin + [ix, iy] * resolution, stored at data[width * iy + ix].
    ix = np.rint((beams[:, 0] - msg.origin[0]) / msg.resolution).astype(int)
    iy = np.rint((beams[:, 1] - msg.origin[1]) / msg.resolution).astype(int)
    inside = (ix >= 0) & (ix < msg.width) & (iy >= 0) & (iy < msg.height)

    heights = np.full(BEAM_LATERAL.shape, np.nan)
    values = cells[iy[inside] * msg.width + ix[inside]]
    known = np.isfinite(values) & (np.abs(values) < MAX_PLAUSIBLE_HEIGHT)
    heights[np.flatnonzero(inside)[known]] = values[known] - base_height
    return heights


def describe_cloud(msg):
    try:
        points = f"latest cloud has {len(decode_points(msg))} finite points"
    except NoReading:
        points = "latest cloud is unreadable"
    names = ",".join(field.name for field in msg.fields)
    return f"frame {msg.header.frame_id!r}, {points}, fields: {names}"


def describe_height_map(msg):
    cells = np.asarray(msg.data, dtype=np.float64)
    known = cells[np.isfinite(cells) & (np.abs(cells) < MAX_PLAUSIBLE_HEIGHT)]
    span = f", heights {known.min():+.3f} to {known.max():+.3f} m" if known.size else ""
    return (f"frame {msg.frame_id!r}, {msg.width} x {msg.height} cells of {msg.resolution:g} m, "
            f"{known.size} of {cells.size} known{span}")


def describe_lidar_state(msg):
    return (f"\nLidar state: error_state={msg.error_state}, firmware {msg.firmware_version!r}, "
            f"software {msg.software_version!r}\n"
            f"  cloud {msg.cloud_frequency:.1f} Hz, {msg.cloud_packet_loss_rate:.3f} packet loss, "
            f"{msg.cloud_size} points, {msg.cloud_scan_num} scans; "
            f"rotation {msg.sys_rotation_speed:.0f} (motor {msg.com_rotation_speed:.0f})")


def spread(values):
    """Width of the middle 90% of the finite values, or NaN if there are none."""
    finite = values[np.isfinite(values)]
    return float(np.percentile(finite, 95) - np.percentile(finite, 5)) if finite.size else math.nan


def median(values):
    finite = values[np.isfinite(values)]
    return float(np.median(finite)) if finite.size else math.nan


def number(value, signed=True):
    if not math.isfinite(value):
        return f"{'--':>7s}"
    return f"{value:+7.3f}" if signed else f"{value:7.3f}"


def report_source(topic, source, describe):
    """Print what a source said about the beams. Return the share of messages that covered them all."""
    count, latest = source.snapshot()
    print(f"\n{topic}: ", end="")
    if latest is None:
        print("no messages")
        return 0.0
    print(describe(latest))

    readings, problems = source.results()
    for problem, times in problems.items():
        print(f"  {times} messages gave no reading: {problem}")
    if not readings:
        return 0.0

    heights = np.array(readings)
    valid = np.isfinite(heights)
    print("  beam      " + " ".join(f" env[{i}]" for i in range(len(BEAM_LATERAL))))
    print("  lateral   " + " ".join(number(value) for value in BEAM_LATERAL))
    print("  height    " + " ".join(number(median(column)) for column in heights.T))
    print("  spread    " + " ".join(number(spread(column), signed=False) for column in heights.T))
    print("  valid %   " + " ".join(f"{100.0 * column.mean():7.0f}" for column in valid.T))
    for left, right in BEAM_PAIRS:
        difference = heights[:, left] - heights[:, right]
        print(f"  env[{left}]-env[{right}]: {number(median(difference))}, spread {number(spread(difference), False)}, "
              f"valid {100.0 * np.isfinite(difference).mean():.0f}%")

    coverage = float(valid.all(axis=1).sum()) / count
    print(f"  All seven beams valid in {100.0 * coverage:.0f}% of {count} messages.")
    return coverage


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("interface", help="network interface wired to the robot")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (the robot uses 0)")
    parser.add_argument("--seconds", type=float, default=10.0, help="how long to listen")
    parser.add_argument("--ahead", type=float, default=BEAM_AHEAD,
                        help="distance of the beams ahead of the base, in m (default: %(default)s)")
    args = parser.parse_args()
    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")

    if not check_interface(args.interface):
        return 2

    ChannelFactoryInitialize(args.domain, args.interface)
    low_state, sport_state, lidar_state, odom = RateMonitor(), RateMonitor(), RateMonitor(), RateMonitor()

    def cloud_source():
        return BeamSource(lambda msg: read_cloud(msg, odom.snapshot()[1], low_state.snapshot()[1], args.ahead))

    clouds = {topic: cloud_source() for topic in CLOUD_TOPICS}
    height_map = BeamSource(lambda msg: read_height_map(msg, odom.snapshot()[1], args.ahead))

    topics = {
        "rt/lowstate": (LowState_, low_state),
        "rt/sportmodestate": (SportModeState_, sport_state),
        LIDAR_STATE_TOPIC: (LidarState_, lidar_state),
        ODOM_TOPIC: (Odometry_, odom),
        **{topic: (PointCloud2_, source) for topic, source in clouds.items()},
        HEIGHT_MAP_TOPIC: (HeightMap_, height_map),
    }
    for topic, (message_type, monitor) in topics.items():
        ChannelSubscriber(topic, message_type).Init(monitor, 10)

    print(f"Listening on {args.interface}, domain {args.domain}, for {args.seconds:g} s (read-only)...")
    time.sleep(args.seconds)

    print()
    for topic, (_, monitor) in topics.items():
        print(f"{topic + ':':30s} {monitor.snapshot()[0]:6d} messages ({monitor.rate():.1f} Hz)")

    if sport_state.snapshot()[1] is None:
        print("\nNo rt/sportmodestate, so sport mode looks off.")
    else:
        print("\nrt/sportmodestate is arriving, so sport mode looks on.")

    latest_state = lidar_state.snapshot()[1]
    if latest_state is not None:
        print(describe_lidar_state(latest_state))
        if latest_state.error_state != 0:
            print(f"WARNING: lidar reports error_state {latest_state.error_state}.")

    print(f"\nBeams {args.ahead:g} m ahead of the base, left to right. Heights are relative to the base, in m.")
    print("In simulation, a beam reads about -0.27 on the platform and -0.57 past its edge.")
    print("The policies steer on the differences, which should be about 0 on a flat floor.")

    coverage = {topic: report_source(topic, source, describe_cloud) for topic, source in clouds.items()}
    coverage[HEIGHT_MAP_TOPIC] = report_source(HEIGHT_MAP_TOPIC, height_map, describe_height_map)

    if not any(monitor.snapshot()[0] for monitor in (lidar_state, odom, height_map, *clouds.values())):
        print("\nFAIL: no lidar topics received. The lidar may be switched off, which this script "
              "can't change, or this firmware may use other topic names.")
        return 1

    covering = [topic for topic, share in coverage.items() if share >= MIN_COVERAGE]
    if not covering:
        print(f"\nFAIL: no source covered all seven beams in at least {100.0 * MIN_COVERAGE:.0f}% of its messages.")
        return 1

    print(f"\nPASS: all seven beams have data from {', '.join(covering)}.")
    print("This only means data reached the beam locations. Check the readings against the real floor.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
