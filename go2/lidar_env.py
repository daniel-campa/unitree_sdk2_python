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
"""
import math
from dataclasses import dataclass, field

import numpy as np

# The raw cloud comes first: it is in the lidar's own frame and keeps publishing with
# sport mode off. The deskewed cloud is in the odom frame and stops about 1.3 s after
# sport mode is released (every run on 2026-11-23), so it only helps with sport mode on.
CLOUD_TOPICS = ("rt/utlidar/cloud", "rt/utlidar/cloud_deskewed")
LIDAR_FRAME = "utlidar_lidar"
HEIGHT_MAP_TOPIC = "rt/utlidar/height_map_array"
LIDAR_STATE_TOPIC = "rt/utlidar/lidar_state"
WORLD_FRAMES = {"odom", "map", "world"}
BODY_FRAMES = {"base", "base_link", "body", "trunk"}
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

# The raw cloud has about 1,400 points, 14.7 times a second, so one cloud puts only a
# couple of points on a box 2-3 m ahead (2026-11-23). The env vector is built from the
# clouds of the last ACCUMULATE_S seconds, each moved to where the robot is now with the
# pose (x, y, yaw) it had when it arrived.
ACCUMULATE_S = 0.4


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


# Unitree's Go2 model (go2_description.urdf, radar_joint): the lidar 0.289 m ahead of the
# base and 0.047 m below it, turned 164.9 degrees about y (upside down, tilted forward).
UNITREE_MOUNT = Mount(LIDAR_FRAME, (0.28945, 0.0, -0.046825), (0.0, 2.8782, 0.0),
                      {"source": "Unitree's go2_description.urdf radar_joint"})

# Fitted to the floor in the raw clouds recorded by the 2026-11-23 15:22 calibration
# (60 clouds, 9 mm rms): roll 177.7, pitch 16.0, yaw 180 degrees, which is 1-2 degrees
# from Unitree's model. Height from sport mode's body height (0.316 m); forward offset
# from Unitree's model, since the board wasn't in view. run_policy.py uses this when the
# calibration has no lidar mount.
RECOVERED_MOUNT = Mount(LIDAR_FRAME, (0.2895, 0.0, 0.0166), (3.1013, 0.2789, math.pi),
                        {"roll_pitch": True, "height": True, "forward": False, "yaw": False,
                         "source": "fitted offline to the 2026-11-23 15:22 calibration recording"})


def rotation_angle(a, b):
    """The angle (rad) between two rotation matrices."""
    return math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1.0) / 2.0)))


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
        known = f"the calibrated mount is for {mount.frame!r}" if mount else "there's no lidar mount in the calibration"
        raise NoReading(f"a cloud in frame {frame!r} can't be placed on the body: {known} "
                        f"(run go2/lidar_calibrate.py)")
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
    """The recent clouds in the level base frame, and the env vector built from them.

    Each usable cloud is kept for window seconds with the robot's pose (x, y, yaw in the
    IMU's world frame) and base height when it arrived. build() moves them all to where the
    robot was at the newest cloud and builds one env vector from the lot. window 0 uses
    only the newest cloud, as before.
    """

    def __init__(self, layout, mount=None, window=ACCUMULATE_S):
        self.layout = layout
        self.mount = mount
        self.window = window
        self.points = None  # the newest cloud alone
        self.frame = None
        self.received = None  # time.perf_counter() when the newest usable cloud was taken in
        self.stamp = None  # the cloud's own header stamp
        self.raw = None  # the newest cloud's points in its own frame, usable or not, for the log
        self.raw_frame = None
        self.clouds = self.unusable = 0
        self.problem = None  # why the newest cloud couldn't be used, if it couldn't
        self.history = []  # (received, pose (x, y, yaw), height, level-base points)

    def take(self, msg, now, roll, pitch, odom_pose=None, pose=None, height=None):
        """Take in a cloud message. Returns True if it could be used.

        pose is the robot's (x, y, yaw) and height its base height when the cloud arrived;
        without a pose the robot is taken to have stood still.
        """
        self.clouds += 1
        try:
            self.raw, self.raw_frame = None, cloud_frame(msg)
            self.raw = decode_points(msg)
        except NoReading as error:
            self.unusable += 1
            self.problem = str(error)
            return False
        self.clouds -= 1
        return self.take_points(self.raw, self.raw_frame, now, roll, pitch, odom_pose, pose, height,
                                cloud_stamp(msg))

    def take_points(self, raw, frame, now, roll, pitch, odom_pose=None, pose=None, height=None, stamp=None):
        """take() for points already decoded (go2/lidar_report.py --replay uses this)."""
        self.clouds += 1
        self.raw, self.raw_frame = raw, frame
        try:
            points = to_level_base(raw, frame, roll, pitch, self.mount, odom_pose)
        except NoReading as error:
            self.unusable += 1
            self.problem = str(error)
            return False
        self.points = points
        self.frame = frame
        self.received = now
        self.stamp = stamp
        self.problem = None
        self.history.append((now, tuple(pose) if pose is not None else (0.0, 0.0, 0.0), height, points))
        self.history = [h for h in self.history if now - h[0] <= self.window + 1e-9]
        return True

    def age(self, now):
        return math.inf if self.received is None else now - self.received

    def accumulated(self):
        """The kept clouds' points, moved into the level base frame of the newest: (points, clouds used)."""
        if not self.history:
            return None, 0
        _, (x0, y0, yaw0), h0, newest = self.history[-1]
        parts = [newest]
        for _, (x, y, yaw), h, pts in self.history[:-1]:
            # level base frame then -> world -> level base frame now (z only shifts with the height)
            c, s = math.cos(yaw), math.sin(yaw)
            wx, wy = x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]
            c0, s0 = math.cos(yaw0), math.sin(yaw0)
            dx, dy = wx - x0, wy - y0
            dz = (h - h0) if (h is not None and h0 is not None) else 0.0
            parts.append(np.column_stack((c0 * dx + s0 * dy, -s0 * dx + c0 * dy, pts[:, 2] + dz)))
        return np.concatenate(parts), len(parts)

    def build(self, height):
        points, used = self.accumulated()
        if points is None:
            env = self.layout.open_floor(height)
            return env, {"points": 0, "probe_points": np.zeros(self.layout.n_probes, dtype=int),
                         "fallback": np.ones(self.layout.size, dtype=bool), "overhead_points": 0,
                         "object_hit": False, "clouds_used": 0}
        env, info = build_env(points, self.layout, height)
        info["clouds_used"] = used
        return env, info
