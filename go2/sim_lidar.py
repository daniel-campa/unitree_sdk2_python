"""Publish simulated lidar topics from a MuJoCo scene, for go2/sim_headless.py --lidar.

This exists to show what go2/check_lidar.py prints. It doesn't imitate the real
lidar: the topic names come from check_lidar.py, and the frames, scan pattern,
mount and unknown-cell marker below are assumptions. There's no noise, field of
view limit or packet loss. A passing check here says nothing about the real robot.

One ray is cast from the lidar's position to each point of a floor grid around the
robot. The rays stop at the first thing they hit, so the floor behind a raised
surface is hidden from them, as it can be from a real lidar. A ring of level and
upward rays is also cast, so that the sides of things taller than the lidar show up
(the floor grid alone only reaches the part of an object below the lidar).

The rays are cast and the messages sent by this file run as a process of its own,
from a copy of the pose taken when a publish is due. Writing the clouds holds the
Python interpreter for 5-7 ms each, so done in the simulator's process it held back
rt/lowstate by up to 29 ms, and run_policy.py walks that run cleanly without --lidar
tripped its joint speed stop. The simulator sends it the model, then the poses, down
its stdin; it stops when that closes.
"""
import os
import pickle
import signal
import struct
import subprocess
import sys

import mujoco
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import (
    nav_msgs_msg_dds__Odometry_,
    std_msgs_msg_dds__Header_,
    unitree_go_msg_dds__HeightMap_,
    unitree_go_msg_dds__LidarState_,
)
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_, PointField_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LidarState_

RATE = 10.0  # Hz
FRAME = "odom"
RESOLUTION = 0.05  # m
GRID_CELLS = 81  # per side, so the grid reaches 2 m from the robot
MOUNT = np.array([0.28, 0.0, 0.1])  # lidar position in the base frame; a guess, because it's undocumented
UNKNOWN = float("nan")  # height map value for cells that no ray reached
POINT_FLOAT32 = 7  # sensor_msgs PointField FLOAT32
# Extra rays, in the lidar's frame, for objects taller than the lidar.
RING_AZIMUTHS = np.radians(np.arange(0.0, 360.0, 2.0))
RING_ELEVATIONS = np.radians(np.arange(-30.0, 46.0, 3.0))
WORLD_GEOMS_ONLY = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)  # the robot's geoms are in groups 2 and 3
SIZE = struct.Struct("<Q")


class SimLidar:
    """The simulator's side: starts the publisher and hands it the pose when a publish is due."""

    def __init__(self, model, data, domain):
        self._data = data
        self._next_publish = 0.0
        # Its own session, so Ctrl-C in the simulator's terminal reaches only the simulator.
        process = subprocess.Popen([sys.executable, __file__, str(domain)], stdin=subprocess.PIPE,
                                   start_new_session=True)
        model_bytes = pickle.dumps(model)
        process.stdin.write(SIZE.pack(len(model_bytes)) + model_bytes)
        process.stdin.flush()
        self._pipe = process.stdin.fileno()
        self._process = process  # keeps the pipe open
        os.set_blocking(self._pipe, False)
        self._pose = struct.Struct(f"<{1 + model.nq}d")  # sim time, qpos; under PIPE_BUF, so written whole

    def step(self):
        """Hand over the pose if a publish is due. Call from the thread that steps the physics."""
        if self._data.time < self._next_publish or self._pipe is None:
            return
        self._next_publish = self._data.time + 1.0 / RATE
        try:
            os.write(self._pipe, self._pose.pack(self._data.time, *self._data.qpos))
        except BlockingIOError:
            pass  # the publisher is behind: it gets the next pose instead
        except OSError:
            self._pipe = None  # it has stopped, and said why on this terminal; the simulation goes on


def _read_exactly(fd, size):
    data = b""
    while len(data) < size:
        chunk = os.read(fd, size - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def _publish_from_pipe(domain):
    """The publisher process: pose a copy of the model from the newest pose, cast and publish."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    fd = sys.stdin.fileno()
    header = _read_exactly(fd, SIZE.size)
    if header is None:
        return
    model = pickle.loads(_read_exactly(fd, SIZE.unpack(header)[0]))
    pose = struct.Struct(f"<{1 + model.nq}d")
    ChannelFactoryInitialize(domain, "lo")
    publisher = _Publisher(model)
    data = mujoco.MjData(model)
    pending = b""
    while True:
        chunk = os.read(fd, 65536)
        if not chunk:
            return  # the simulator has stopped
        pending += chunk
        whole = len(pending) // pose.size
        if whole == 0:
            continue
        values = pose.unpack(pending[(whole - 1) * pose.size:whole * pose.size])  # the newest only
        pending = pending[whole * pose.size:]
        data.qpos[:] = values[1:]
        mujoco.mj_kinematics(model, data)
        publisher.publish(values[0], data)


class _Publisher:
    def __init__(self, model):
        self._model = model
        self._odom = ChannelPublisher("rt/utlidar/robot_odom", Odometry_)
        self._state = ChannelPublisher("rt/utlidar/lidar_state", LidarState_)
        self._clouds = [ChannelPublisher(topic, PointCloud2_)
                        for topic in ("rt/utlidar/cloud", "rt/utlidar/cloud_deskewed")]
        self._map = ChannelPublisher("rt/utlidar/height_map_array", HeightMap_)
        for publisher in (self._odom, self._state, *self._clouds, self._map):
            publisher.Init()

        offsets = np.arange(GRID_CELLS) * RESOLUTION
        self._grid_x, self._grid_y = np.meshgrid(offsets, offsets)  # indexed [iy, ix]
        azimuth, elevation = np.meshgrid(RING_AZIMUTHS, RING_ELEVATIONS)
        self._ring = np.column_stack((np.cos(elevation.ravel()) * np.cos(azimuth.ravel()),
                                      np.cos(elevation.ravel()) * np.sin(azimuth.ravel()),
                                      np.sin(elevation.ravel())))
        rays = GRID_CELLS * GRID_CELLS + len(self._ring)
        self._directions = np.zeros(rays * 3)
        self._geoms = np.zeros(rays, dtype=np.int32)
        self._distances = np.zeros(rays)

    def publish(self, stamp, data):
        origin = self._grid_origin(data)
        points = self._cast_rays(origin, data)
        self._odom.Write(odom_message(stamp, data.qpos))
        self._state.Write(self._state_message(stamp, len(points)))
        cloud = self._cloud_message(stamp, points)
        for publisher in self._clouds:
            publisher.Write(cloud)
        self._map.Write(self._map_message(stamp, origin, points))

    def _grid_origin(self, data):
        """Return the grid's corner, on whole cells so that cells stay put as the robot moves."""
        half_width = (GRID_CELLS - 1) * RESOLUTION / 2.0
        return np.round((data.qpos[:2] - half_width) / RESOLUTION) * RESOLUTION

    def _cast_rays(self, origin, data):
        """Return the world x, y, z of everything the rays hit, as an (N, 3) array."""
        lidar = np.zeros(3)
        mujoco.mju_rotVecQuat(lidar, MOUNT, data.qpos[3:7])
        lidar += data.qpos[:3]

        directions = self._directions.reshape(-1, 3)
        grid = directions[: GRID_CELLS * GRID_CELLS]
        grid[:, 0] = origin[0] + self._grid_x.ravel() - lidar[0]
        grid[:, 1] = origin[1] + self._grid_y.ravel() - lidar[1]
        grid[:, 2] = -lidar[2]
        grid /= np.linalg.norm(grid, axis=1, keepdims=True)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, data.qpos[3:7])
        directions[GRID_CELLS * GRID_CELLS:] = self._ring @ rotation.reshape(3, 3).T

        mujoco.mj_multiRay(self._model, data, lidar, self._directions, WORLD_GEOMS_ONLY, True, -1,
                           self._geoms, self._distances, None, len(self._geoms), mujoco.mjMAXVAL)
        hit = self._geoms >= 0
        return lidar + directions[hit] * self._distances[hit, None]

    def _state_message(self, stamp, num_points):
        msg = unitree_go_msg_dds__LidarState_()
        msg.stamp = stamp
        msg.firmware_version = msg.software_version = "simulated"
        msg.cloud_frequency = RATE
        msg.cloud_size = num_points
        return msg

    def _cloud_message(self, stamp, points):
        fields = [PointField_(axis, 4 * i, POINT_FLOAT32, 1) for i, axis in enumerate("xyz")]
        data = list(points.astype("<f4").tobytes())
        return PointCloud2_(header(stamp), 1, len(points), fields, False, 12, 12 * len(points), data, True)

    def _map_message(self, stamp, origin, points):
        """Keep the highest point in each cell."""
        ix = np.rint((points[:, 0] - origin[0]) / RESOLUTION).astype(int)
        iy = np.rint((points[:, 1] - origin[1]) / RESOLUTION).astype(int)
        inside = (ix >= 0) & (ix < GRID_CELLS) & (iy >= 0) & (iy < GRID_CELLS)
        heights = np.full(GRID_CELLS * GRID_CELLS, -np.inf)
        np.maximum.at(heights, iy[inside] * GRID_CELLS + ix[inside], points[inside, 2])
        heights[np.isneginf(heights)] = UNKNOWN

        msg = unitree_go_msg_dds__HeightMap_()
        msg.stamp = stamp
        msg.frame_id = FRAME
        msg.resolution = RESOLUTION
        msg.width = msg.height = GRID_CELLS
        msg.origin = [float(origin[0]), float(origin[1])]
        msg.data = heights.tolist()
        return msg


def header(stamp):
    msg = std_msgs_msg_dds__Header_()
    msg.stamp.sec = int(stamp)
    msg.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    msg.frame_id = FRAME
    return msg


def odom_message(stamp, qpos):
    msg = nav_msgs_msg_dds__Odometry_()
    msg.header = header(stamp)
    msg.child_frame_id = "base_link"
    position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
    position.x, position.y, position.z = (float(value) for value in qpos[:3])
    orientation.w, orientation.x, orientation.y, orientation.z = (float(value) for value in qpos[3:7])
    return msg


ODOM_ONLY_RATE = 50.0  # Hz


class SimOdom:
    """Publish only rt/utlidar/robot_odom, the simulator's true base pose, for go2/sim_headless.py --odom.

    No rays are cast, so unlike SimLidar it hardly slows the simulation. It's for
    scoring a run in the simulator from run_policy.py's log, not for check_lidar.py.
    """

    def __init__(self, model, data):
        self._data = data
        self._next_publish = 0.0
        self._odom = ChannelPublisher("rt/utlidar/robot_odom", Odometry_)
        self._odom.Init()

    def step(self):
        """Publish if it's time to. Call from the thread that steps the physics."""
        if self._data.time < self._next_publish:
            return
        self._next_publish = self._data.time + 1.0 / ODOM_ONLY_RATE
        self._odom.Write(odom_message(self._data.time, self._data.qpos))


if __name__ == "__main__":
    _publish_from_pipe(int(sys.argv[1]))
