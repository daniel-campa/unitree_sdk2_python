"""Rebuild S-M-ICRL's env vector on the Go2 from the lidar's point cloud.

S-M-ICRL (world/robot_api.py, RobotAPI._build_env_vector) gives every policy an env
vector of N probes, then up_clearance, object_dist and a bias of 1. In the simulator
each value is a MuJoCo ray cast; here the same rays are traced through the lidar's
points instead. The geometry (distances, offsets, bearings and the rays' angles) is
S-M-ICRL's, so with the lidar's mount known the robot and the simulator see the same
numbers for the same scene.

  terrain (the walk configs)     probe i = ground height at probe_ahead m ahead and
                                 probe_lateral[i] m to the left, minus the base height
  obstacle_bearing (push_box)    probe i = max(0, 1 - d / probe_range), d the nearest hit
                                 in a fan of probe_rays_per_sector rays, probe_sector_deg
                                 wide, around bearing probe_bearings_deg[i]
  up_clearance                   gap from the base to the nearest thing above it, at most 2 m,
                                 counting only things higher than 0.5 m off the floor
  object_dist                    first hit of a near-level ray from 0.10 m above the base,
                                 angled down to meet open floor 10 m ahead (terrain mode); the
                                 nearest fan hit (obstacle_bearing mode)

Points are first put in the "level base frame": origin at the base, x forward along
the robot's heading, y to the left, z straight up. A cloud in a body frame is levelled
with the IMU's roll and pitch. A cloud in the lidar's own frame also needs the lidar's
mount on the body, which go2/lidar_calibrate.py measures. A cloud in a world frame
(odom) needs the robot's pose, from rt/utlidar/robot_odom, and that stops publishing
once sport mode is off.

Where the lidar has no points (a probe spot in its blind zone, say), a value falls
back to what S-M-ICRL would read on open, flat floor, and the reading says which
values fell back.

One rt/utlidar/cloud message is too sparse for the rays: about 0.04 points per square
degree straight ahead (the robot, 2026-11-20), so a ray's strip on a crate 2 m out holds
about half a point, and a hit needs RAY_MIN_POINTS. CloudEnv can therefore trace the
rays through the clouds of the last `window` seconds together. The older clouds are
turned to the newest one's heading by the IMU's yaw. Nothing corrects for the base
moving, which is right for turning on the spot.
"""
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

CLOUD_TOPICS = ("rt/utlidar/cloud_deskewed", "rt/utlidar/cloud")
HEIGHT_MAP_TOPIC = "rt/utlidar/height_map_array"
LIDAR_STATE_TOPIC = "rt/utlidar/lidar_state"
WORLD_FRAMES = {"odom", "map", "world"}
BODY_FRAMES = {"base", "base_link", "body", "trunk"}
# go2/lidar_calibrate.py writes the measured mount here, and go2/run_policy.py reads it.
MOUNT_FILE = Path(__file__).resolve().parent / "logs" / "lidar_mount.json"
CLOUD_WINDOW_S = 0.5  # go2/run_policy.py's default window: see CloudEnv
POINT_FIELD_FORMATS = {7: "f4", 8: "f8"}  # sensor_msgs PointField FLOAT32, FLOAT64

# S-M-ICRL's values (world/robot_api.py).
OPEN_CLEARANCE = 2.0  # _clearance_above's max_clear
OPEN_OBSTACLE_DISTANCE = 10.0  # _object_distance_ahead's max_dist
RAY_START_HEIGHT = 0.10  # object rays start this far above the base
CEILING_CAST_FROM = 0.5  # the clearance ray starts at this height above the floor
TERRAIN_CAST_FROM = 1.0  # terrain probes are cast down from this height above the floor

# How a ray is traced through points. A MuJoCo ray is a line and points are samples,
# so a ray "hits" where, in a thin strip along it, an object's points reach the ray.
PROBE_RADIUS = 0.06  # m around a terrain probe spot (half check_lidar.py's beam spacing)
PROBE_MIN_POINTS = 3
RAY_HALF_WIDTH = 0.05  # m either side of an object ray
RAY_STEP = 0.05  # m stretches along the ray, checked one at a time
RAY_NEAR, RAY_NEAR_PER_M = 0.03, 0.01  # m (+ per m of distance) from the ray's height that's "at" it
RAY_MIN_POINTS = 2  # points in a stretch needed for a hit, so one stray point isn't
FLOOR_TOL, FLOOR_TOL_PER_M = 0.04, 0.01  # m (+ per m) above the floor, by the estimated height, that's still floor
CLEARANCE_RADIUS = 0.10  # m around the base for the overhead ray

# The robot's own body, legs and lidar housing, in the level base frame. Points inside
# are dropped. The floor (about -0.27 m) is below it.
SELF_BOX = ((-0.45, 0.42), (-0.25, 0.25), (-0.15, 0.20))


class NoReading(Exception):
    """A cloud can't be turned into the level base frame."""


@dataclass
class EnvLayout:
    """S-M-ICRL's probe settings (the env block of a task config); defaults are the walk's."""

    mode: str = "terrain"
    probe_ahead: float = 0.30
    probe_lateral: tuple = (0.15, 0.0, -0.15)
    probe_bearings_deg: tuple = (30.0, 0.0, -30.0)
    probe_range: float = 2.0
    probe_sector_deg: float = 40.0
    probe_rays_per_sector: int = 5

    def __post_init__(self):
        if self.mode not in ("terrain", "obstacle_bearing"):
            raise ValueError(f"env mode must be terrain or obstacle_bearing, not {self.mode!r}")
        self.probe_lateral = tuple(float(v) for v in self.probe_lateral)
        self.probe_bearings_deg = tuple(float(v) for v in self.probe_bearings_deg)

    @property
    def n_probes(self):
        return len(self.probe_bearings_deg) if self.mode == "obstacle_bearing" else len(self.probe_lateral)

    @property
    def size(self):
        return self.n_probes + 3

    def ray_headings(self):
        """Per probe, the headings (rad, + = left) of its fan's rays, as S-M-ICRL casts them."""
        n = max(1, int(self.probe_rays_per_sector))
        spread = np.linspace(-0.5, 0.5, n) * math.radians(self.probe_sector_deg) if n > 1 else np.zeros(1)
        return [math.radians(b) + spread for b in self.probe_bearings_deg]

    def open_floor(self, height):
        """The env vector on clear, flat floor with the base at this height: S-M-ICRL's values."""
        probes = [-height] * self.n_probes if self.mode == "terrain" else [0.0] * self.n_probes
        return np.array([*probes, OPEN_CLEARANCE, OPEN_OBSTACLE_DISTANCE, 1.0])

    def as_dict(self):
        return {"mode": self.mode, "probe_ahead": self.probe_ahead, "probe_lateral": list(self.probe_lateral),
                "probe_bearings_deg": list(self.probe_bearings_deg), "probe_range": self.probe_range,
                "probe_sector_deg": self.probe_sector_deg, "probe_rays_per_sector": self.probe_rays_per_sector}


@dataclass
class Mount:
    """Where the lidar sits on the body: a point in its frame is R(rpy) @ p + xyz in the base frame."""

    frame: str
    xyz: tuple = (0.0, 0.0, 0.0)
    rpy: tuple = (0.0, 0.0, 0.0)
    measured: dict = field(default_factory=dict)  # which parts go2/lidar_calibrate.py measured

    @classmethod
    def from_dict(cls, d):
        if not d:
            return None
        return cls(frame=d["frame"], xyz=tuple(d["xyz"]), rpy=tuple(d["rpy"]), measured=dict(d.get("measured", {})))

    def as_dict(self):
        return {"frame": self.frame, "xyz": [round(float(v), 4) for v in self.xyz],
                "rpy": [round(float(v), 4) for v in self.rpy], "measured": self.measured}

    def rotation(self):
        return rotation_rpy(*self.rpy)


def load_mount(path):
    """The Mount saved at path, or None if there's no file."""
    path = Path(path)
    if not path.exists():
        return None
    return Mount.from_dict(json.loads(path.read_text()))


def rotation_rpy(roll, pitch, yaw):
    """Rz(yaw) @ Ry(pitch) @ Rx(roll), the same ZYX convention as S-M-ICRL's rpy."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def decode_points(msg):
    """A PointCloud2's finite x, y, z points as an (N, 3) array, in the message's frame."""
    fields = {f.name: f for f in msg.fields}
    try:
        layout = [(fields[axis].offset, POINT_FIELD_FORMATS[fields[axis].datatype]) for axis in "xyz"]
    except KeyError as missing:
        raise NoReading(f"unsupported point layout: {missing} in {sorted(fields)}") from None
    order = ">" if msg.is_bigendian else "<"
    try:
        dtype = np.dtype({"names": list("xyz"), "formats": [order + fmt for _, fmt in layout],
                          "offsets": [offset for offset, _ in layout], "itemsize": msg.point_step})
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    except ValueError as error:
        raise NoReading(f"malformed cloud: {error}") from None
    points = np.column_stack([raw[axis].astype(np.float64) for axis in "xyz"])
    return points[np.isfinite(points).all(axis=1)]


def cloud_frame(msg):
    return msg.header.frame_id.strip("/")


def cloud_stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def to_level_base(points, frame, roll, pitch, mount=None, odom_pose=None):
    """Points in the level base frame (origin at the base, x along the heading, z up).

    roll and pitch are the IMU's. odom_pose (x, y, z, qw, qx, qy, qz) is needed for a
    cloud in a world frame, mount for one in the lidar's own frame.
    """
    if frame in WORLD_FRAMES:
        if odom_pose is None:
            raise NoReading(f"a cloud in frame {frame!r} needs the robot's pose, and rt/utlidar/robot_odom "
                            f"isn't arriving (it stops once sport mode is off)")
        x, y, z, w, qx, qy, qz = odom_pose
        yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        c, s = math.cos(yaw), math.sin(yaw)
        d = points - np.array([x, y, z])
        return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]))
    if frame in BODY_FRAMES:
        body = points
    elif mount is not None and frame == mount.frame:
        body = points @ mount.rotation().T + np.asarray(mount.xyz)
    else:
        known = f"the measured mount is for {mount.frame!r}" if mount else f"there's no lidar mount ({MOUNT_FILE.name})"
        raise NoReading(f"a cloud in frame {frame!r} can't be placed on the body: {known} "
                        f"(measure it with go2/lidar_calibrate.py)")
    return body @ rotation_rpy(roll, pitch, 0.0).T


def drop_self(points):
    (x0, x1), (y0, y1), (z0, z1) = SELF_BOX
    inside = ((points[:, 0] > x0) & (points[:, 0] < x1) & (points[:, 1] > y0) & (points[:, 1] < y1)
              & (points[:, 2] > z0) & (points[:, 2] < z1))
    return points[~inside]


def ray_hit(points, heading, height, max_dist=OPEN_OBSTACLE_DISTANCE):
    """S-M-ICRL's _object_distance_ahead, traced through points: distance along the ray, or None.

    The ray leaves RAY_START_HEIGHT above the base at this heading and drops just enough
    to meet the floor (height below the base) max_dist ahead.
    """
    c, s = math.cos(heading), math.sin(heading)
    along = c * points[:, 0] + s * points[:, 1]
    across = -s * points[:, 0] + c * points[:, 1]
    drop = (height + RAY_START_HEIGHT) / max_dist
    above = points[:, 2] - (RAY_START_HEIGHT - drop * along)  # height over the ray where the point is
    # The ray hits a surface that crosses its height, so the hit has points at the ray's
    # height: a box face does, a ceiling over the ray or a low step under it doesn't. The
    # floor itself is left out, and points spread out with distance, so both tolerances grow.
    near = ((along > 0.0) & (along <= max_dist) & (np.abs(across) <= RAY_HALF_WIDTH)
            & (points[:, 2] > -height + FLOOR_TOL + FLOOR_TOL_PER_M * along)
            & (np.abs(above) <= RAY_NEAR + RAY_NEAR_PER_M * along))
    if near.sum() < RAY_MIN_POINTS:
        return None
    along = np.sort(along[near])
    bins = np.floor(along / RAY_STEP).astype(int)
    starts = np.flatnonzero(np.r_[True, bins[1:] != bins[:-1]])
    count = np.diff(np.r_[starts, len(along)])
    if not (count >= RAY_MIN_POINTS).any():
        return None
    i = int(np.argmax(count >= RAY_MIN_POINTS))  # the nearest stretch with enough points at the ray
    hit = float(np.median(along[starts[i]:starts[i] + count[i]]))
    return min(hit * math.sqrt(1.0 + drop * drop), max_dist)


def build_env(points, layout, height):
    """The env vector from level-base-frame points, and what each value came from.

    height is the base's height above the floor (the estimator's), which S-M-ICRL's
    rays use for their angle and the fallbacks use for open floor.
    Returns (env, info): info["fallback"] marks values with no points behind them.
    """
    points = drop_self(points)
    open_env = layout.open_floor(height)
    env = open_env.copy()
    fallback = np.zeros(layout.size, dtype=bool)
    counts = np.zeros(layout.n_probes, dtype=int)

    if layout.mode == "terrain":
        for i, lateral in enumerate(layout.probe_lateral):
            near = ((np.hypot(points[:, 0] - layout.probe_ahead, points[:, 1] - lateral) < PROBE_RADIUS)
                    & (points[:, 2] <= TERRAIN_CAST_FROM - height))
            counts[i] = int(near.sum())
            if counts[i] >= PROBE_MIN_POINTS:
                env[i] = float(np.median(points[near, 2]))
            else:
                fallback[i] = True
        hit = ray_hit(points, 0.0, height)
        env[layout.n_probes + 1] = OPEN_OBSTACLE_DISTANCE if hit is None else hit
        fan_hits = [hit]
    else:
        fan_hits = []
        for i, headings in enumerate(layout.ray_headings()):
            ranges = [ray_hit(points, h, height) for h in headings]
            found = [r for r in ranges if r is not None]
            counts[i] = len(found)
            nearest = min(found) if found else OPEN_OBSTACLE_DISTANCE
            fan_hits.append(nearest if found else None)
            env[i] = max(0.0, 1.0 - nearest / layout.probe_range)
        env[layout.n_probes + 1] = min(r if r is not None else OPEN_OBSTACLE_DISTANCE for r in fan_hits)

    # Overhead: only what's higher than CEILING_CAST_FROM off the floor counts, as in S-M-ICRL.
    over = (np.hypot(points[:, 0], points[:, 1]) < CLEARANCE_RADIUS) & (points[:, 2] > CEILING_CAST_FROM - height)
    if over.sum() >= PROBE_MIN_POINTS:
        env[layout.n_probes] = min(float(np.median(points[over, 2])), OPEN_CLEARANCE)

    info = {"points": int(len(points)), "probe_points": counts, "fallback": fallback,
            "overhead_points": int(over.sum()), "object_hit": env[layout.n_probes + 1] < OPEN_OBSTACLE_DISTANCE}
    return env, info


class CloudEnv:
    """The newest cloud in the level base frame, and the env vector built from it.

    With window > 0 the env vector is built from every usable cloud taken in within
    window seconds of the newest, each turned by its yaw to the newest one's heading.
    """

    def __init__(self, layout, mount=None, window=0.0):
        self.layout = layout
        self.mount = mount
        self.window = window
        self._recent = []  # (received, yaw, points) of the usable clouds kept for the window
        self.points = None
        self.frame = None
        self.received = None  # time.perf_counter() when the newest usable cloud was taken in
        self.stamp = None  # the cloud's own header stamp
        self.raw = None  # the newest cloud's points in its own frame, usable or not, for the log
        self.raw_frame = None
        self.clouds = self.unusable = 0
        self.problem = None  # why the newest cloud couldn't be used, if it couldn't

    def take(self, msg, now, roll, pitch, odom_pose=None, yaw=0.0):
        """Take in a cloud message. yaw is the IMU's, for the window. Returns True if it could be used."""
        self.clouds += 1
        try:
            self.raw, self.raw_frame = None, cloud_frame(msg)
            self.raw = decode_points(msg)
            points = to_level_base(self.raw, self.raw_frame, roll, pitch, self.mount, odom_pose)
        except NoReading as error:
            self.unusable += 1
            self.problem = str(error)
            return False
        self.points = points
        self.frame = self.raw_frame
        self.received = now
        self.stamp = cloud_stamp(msg)
        self.problem = None
        if self.window > 0:
            self._recent = [r for r in self._recent if now - r[0] <= self.window] + [(now, float(yaw), points)]
        return True

    def age(self, now):
        return math.inf if self.received is None else now - self.received

    def combined(self):
        """The points the env vector is built from: the window's clouds, at the newest one's heading."""
        if len(self._recent) < 2:
            return self.points
        newest_yaw = self._recent[-1][1]
        parts = []
        for _, yaw, points in self._recent:
            c, s = math.cos(yaw - newest_yaw), math.sin(yaw - newest_yaw)
            parts.append(np.column_stack((c * points[:, 0] - s * points[:, 1], s * points[:, 0] + c * points[:, 1],
                                          points[:, 2])))
        return np.concatenate(parts)

    def build(self, height):
        if self.points is None:
            env = self.layout.open_floor(height)
            return env, {"points": 0, "probe_points": np.zeros(self.layout.n_probes, dtype=int),
                         "fallback": np.ones(self.layout.size, dtype=bool), "overhead_points": 0,
                         "object_hit": False, "clouds": 0}
        env, info = build_env(self.combined(), self.layout, height)
        info["clouds"] = max(1, len(self._recent))
        return env, info
