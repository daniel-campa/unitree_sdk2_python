"""Publish simulated lidar topics from a MuJoCo scene, for go2/sim_headless.py --lidar.

This exists to show what go2/check_lidar.py prints. It doesn't imitate the real
lidar: the topic names come from check_lidar.py, and the frames, scan pattern,
mount and unknown-cell marker below are assumptions. There's no noise, field of
view limit or packet loss. A passing check here says nothing about the real robot.

One ray is cast from the lidar's position to each point of a floor grid around the
robot. The rays stop at the first thing they hit, so the floor behind a raised
surface is hidden from them, as it can be from a real lidar.
"""
import mujoco
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher
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
WORLD_GEOMS_ONLY = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)  # the robot's geoms are in groups 2 and 3


class SimLidar:
    def __init__(self, model, data):
        self._model, self._data = model, data
        self._next_publish = 0.0
        self._odom = ChannelPublisher("rt/utlidar/robot_odom", Odometry_)
        self._state = ChannelPublisher("rt/utlidar/lidar_state", LidarState_)
        self._clouds = [ChannelPublisher(topic, PointCloud2_)
                        for topic in ("rt/utlidar/cloud", "rt/utlidar/cloud_deskewed")]
        self._map = ChannelPublisher("rt/utlidar/height_map_array", HeightMap_)
        for publisher in (self._odom, self._state, *self._clouds, self._map):
            publisher.Init()

        offsets = np.arange(GRID_CELLS) * RESOLUTION
        self._grid_x, self._grid_y = np.meshgrid(offsets, offsets)  # indexed [iy, ix]
        rays = GRID_CELLS * GRID_CELLS
        self._directions = np.zeros(rays * 3)
        self._geoms = np.zeros(rays, dtype=np.int32)
        self._distances = np.zeros(rays)

    def step(self):
        """Publish if it's time to. Call from the thread that steps the physics."""
        if self._data.time < self._next_publish:
            return
        self._next_publish = self._data.time + 1.0 / RATE

        origin = self._grid_origin()
        points = self._cast_rays(origin)
        stamp = self._data.time
        self._odom.Write(self._odom_message(stamp))
        self._state.Write(self._state_message(stamp, len(points)))
        cloud = self._cloud_message(stamp, points)
        for publisher in self._clouds:
            publisher.Write(cloud)
        self._map.Write(self._map_message(stamp, origin, points))

    def _grid_origin(self):
        """Return the grid's corner, on whole cells so that cells stay put as the robot moves."""
        half_width = (GRID_CELLS - 1) * RESOLUTION / 2.0
        return np.round((self._data.qpos[:2] - half_width) / RESOLUTION) * RESOLUTION

    def _cast_rays(self, origin):
        """Return the world x, y, z of everything the rays hit, as an (N, 3) array."""
        lidar = np.zeros(3)
        mujoco.mju_rotVecQuat(lidar, MOUNT, self._data.qpos[3:7])
        lidar += self._data.qpos[:3]

        directions = self._directions.reshape(-1, 3)
        directions[:, 0] = origin[0] + self._grid_x.ravel() - lidar[0]
        directions[:, 1] = origin[1] + self._grid_y.ravel() - lidar[1]
        directions[:, 2] = -lidar[2]
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)

        mujoco.mj_multiRay(self._model, self._data, lidar, self._directions, WORLD_GEOMS_ONLY, True, -1,
                           self._geoms, self._distances, None, len(self._geoms), mujoco.mjMAXVAL)
        hit = self._geoms >= 0
        return lidar + directions[hit] * self._distances[hit, None]

    def _header(self, stamp):
        header = std_msgs_msg_dds__Header_()
        header.stamp.sec = int(stamp)
        header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        header.frame_id = FRAME
        return header

    def _odom_message(self, stamp):
        msg = nav_msgs_msg_dds__Odometry_()
        msg.header = self._header(stamp)
        msg.child_frame_id = "base_link"
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        position.x, position.y, position.z = (float(value) for value in self._data.qpos[:3])
        orientation.w, orientation.x, orientation.y, orientation.z = (float(value) for value in self._data.qpos[3:7])
        return msg

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
        return PointCloud2_(self._header(stamp), 1, len(points), fields, False, 12, 12 * len(points), data, True)

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
