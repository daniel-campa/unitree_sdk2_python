"""Estimate the Go2's body velocity and height, robot_state[30:34], from legs and IMU.

S-M-ICRL gives policies the body's linear velocity in the world frame (qvel[0:3]) and
the base height above the floor (qpos[2]). The robot measures neither once sport mode
is released, so this rebuilds them from what rt/lowstate does carry:

- Which feet are standing: from rt/lowstate's foot_force, which is in raw sensor
  units, so each foot's unloaded reading and range are learned as it goes. If the
  sensors don't vary (missing or dead), the lowest feet count as standing instead,
  which is less reliable because these gaits lift the feet only 2-3 cm.
- Height: with the IMU's attitude, leg kinematics gives each foot's depth below the
  base, so a standing foot gives the base height above a flat floor.
- Velocity: a standing foot's contact point doesn't move, so the base moves opposite
  to that point's velocity relative to the base. The foot is a 22 mm sphere that rolls,
  so the point is the bottom of the sphere, not its centre: that alone made walks read
  about 10% slow. The accelerometer carries the estimate between foot readings, and
  the kinematics pull it back so it doesn't drift. A foot that slips makes the robot
  read slower than it is.

Velocity is in the frame S-M-ICRL uses: world axes, with x along the robot's heading
when the walk started (S-M-ICRL starts every rollout at yaw 0).

The leg geometry is the Go2 model's, shared by S-M-ICRL and unitree_mujoco. It needs
no simulator: only numpy.
"""
import numpy as np

# Legs in rt/lowstate's motor order: FR, FL, RR, RL. +1 is the left side.
SIDE = np.array([-1.0, 1.0, -1.0, 1.0])
FRONT = np.array([1.0, 1.0, -1.0, -1.0])
HIP_X, HIP_Y = 0.1934, 0.0465  # base_link to the hip joint, m
THIGH_Y = 0.0955  # hip joint to the thigh joint, sideways
THIGH_LENGTH = 0.213  # thigh joint to the knee
FOOT = np.array([-0.002, 0.0, -0.213])  # knee to the foot sphere's centre
FOOT_RADIUS = 0.022
GRAVITY = np.array([0.0, 0.0, -9.81])

CONTACT_MARGIN = 0.005  # m: without force sensors, feet within this of the lowest count as standing
FORCE_RISE_TAU = 2.0  # s: how fast a foot's learned unloaded reading and range forget old extremes
FORCE_DEADBAND = 0.15  # share of a foot's range treated as unloaded
MIN_FORCE_SPAN = 5.0  # raw units: a sensor that varies less than this is treated as missing
VELOCITY_TAU = 0.05  # s: how fast the kinematic velocity corrects the accelerometer's
VELOCITY_TAU_NO_ACCEL = 0.02  # s: plain low-pass when there's no accelerometer
HEIGHT_TAU = 0.03  # s
MAX_SPEED = 3.0  # m/s; an estimate beyond this is a glitch, not motion
HEIGHT_RANGE = (0.05, 0.6)  # m
# The accelerometer is used only while it averages about 1 g upward in the world frame,
# so a missing, dead or wrongly signed one can't make the estimate drift.
ACCEL_CHECK_TAU = 1.0  # s
ACCEL_UP_RANGE = (7.0, 12.5)  # m/s^2


def rotation(w, x, y, z):
    """Body-to-world rotation matrix from a (w, x, y, z) quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def foot_positions(q):
    """Foot sphere centres in the base frame, (4, 3), for joint angles q in motor order."""
    q = np.asarray(q, dtype=float).reshape(4, 3)
    hip, thigh, knee = q[:, 0], q[:, 1], q[:, 2]
    # In the thigh frame, which turns with the thigh joint about y.
    ck, sk = np.cos(knee), np.sin(knee)
    knee_to_foot = np.stack([ck * FOOT[0] + sk * FOOT[2], np.zeros(4), -sk * FOOT[0] + ck * FOOT[2]], axis=1)
    below = knee_to_foot + np.array([0.0, 0.0, -THIGH_LENGTH])
    ct, st = np.cos(thigh), np.sin(thigh)
    leg = np.stack([ct * below[:, 0] + st * below[:, 2], SIDE * THIGH_Y, -st * below[:, 0] + ct * below[:, 2]],
                   axis=1)
    # The hip joint turns the whole leg about x.
    ch, sh = np.cos(hip), np.sin(hip)
    out = np.stack([leg[:, 0], ch * leg[:, 1] - sh * leg[:, 2], sh * leg[:, 1] + ch * leg[:, 2]], axis=1)
    out[:, 0] += FRONT * HIP_X
    out[:, 1] += SIDE * HIP_Y
    return out


def foot_velocities(q, dq, step=1e-5):
    """Foot sphere centre velocities relative to the base, in the base frame: J(q) dq, (4, 3)."""
    q, dq = np.asarray(q, dtype=float), np.asarray(dq, dtype=float)
    return (foot_positions(q + step * dq) - foot_positions(q - step * dq)) / (2 * step)


def calf_rates(q, dq):
    """Each calf's angular velocity relative to the base, in the base frame, (4, 3)."""
    q, dq = np.asarray(q, dtype=float).reshape(4, 3), np.asarray(dq, dtype=float).reshape(4, 3)
    pitch_axis = np.stack([np.zeros(4), np.cos(q[:, 0]), np.sin(q[:, 0])], axis=1)  # y, turned by the hip
    return dq[:, [0]] * np.array([1.0, 0.0, 0.0]) + (dq[:, 1] + dq[:, 2])[:, None] * pitch_axis


class ForceContact:
    """Standing weights from raw foot_force, learning each foot's unloaded reading and range.

    unloaded and standing, from go2/calibrate.py, give each foot's readings with the
    robot hanging and standing, so contact is right from the first step; learning
    still follows them as they drift.
    """

    def __init__(self, unloaded=None, standing=None):
        self.low = None if unloaded is None else np.asarray(unloaded, dtype=float).copy()
        self.high = None if standing is None else np.asarray(standing, dtype=float).copy()
        if self.low is not None and self.high is None:
            self.high = self.low.copy()

    def weights(self, force, dt):
        force = np.asarray(force, dtype=float)
        if self.low is None:
            self.low = force.copy()
        if self.high is None:
            self.high = force.copy()
        rise = min(1.0, dt / FORCE_RISE_TAU)
        self.low = np.minimum(force, self.low + rise * (force - self.low))
        self.high = np.maximum(force, self.high + rise * (force - self.high))
        span = self.high - self.low
        if np.median(span) < MIN_FORCE_SPAN:
            return None
        share = (force - self.low) / np.maximum(span, MIN_FORCE_SPAN)
        return np.clip(share - FORCE_DEADBAND, 0.0, None)


class BaseEstimator:
    """Body velocity and height from joint state and IMU, updated every control cycle."""

    def __init__(self, start_yaw=0.0, use_accel=True, use_force=True, calibration=None):
        """calibration: go2/calibrate.py's "estimator" section, or None to learn everything."""
        self.use_accel, self.use_force = use_accel, use_force
        self.calibration = calibration or {}
        if self.calibration.get("use_force") is False:
            self.use_force = False
        self.reset(start_yaw)

    def reset(self, start_yaw=0.0):
        """Start over, measuring heading from start_yaw (rad, the IMU's yaw now)."""
        c, s = np.cos(-start_yaw), np.sin(-start_yaw)
        self._unyaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.velocity = np.zeros(3)
        self.height = None
        self.contact = np.ones(4, dtype=bool)
        self.weight = np.full(4, 0.25)
        self.contact_source = "none"
        self._force = ForceContact(self.calibration.get("foot_force_unloaded"),
                                   self.calibration.get("foot_force_standing"))
        self.kinematic_velocity = np.zeros(3)
        self.kinematic_height = np.nan
        self.accel_ok = False
        self._mean_up_accel = None
        self._t = None

    def update(self, t, q, dq, quaternion, gyro, accel=None, foot_force=None):
        """Fold in one state. Returns (velocity (3,), height)."""
        dt = 0.0 if self._t is None else float(np.clip(t - self._t, 0.0, 0.05))
        self._t = t
        gyro = np.asarray(gyro, dtype=float)
        R = self._unyaw @ rotation(*quaternion)
        feet = foot_positions(q)
        feet_world = feet @ R.T
        heights = FOOT_RADIUS - feet_world[:, 2]  # base height above the floor, per foot
        weight = None
        if self.use_force and foot_force is not None:
            weight = self._force.weights(foot_force, dt)
            if weight is not None and weight.sum() <= 0.0:
                weight = None
        if weight is None:
            weight = np.clip(1.0 - (heights.max() - heights) / CONTACT_MARGIN, 0.0, 1.0)  # lowest foot is 1
            self.contact_source = "height"
        else:
            self.contact_source = "force"
        weight = weight / weight.sum()
        self.weight = weight
        self.contact = weight > 0.5 / 4

        # Velocity of the calf's material point at the bottom of the foot sphere.
        down = R.T @ np.array([0.0, 0.0, -FOOT_RADIUS])  # centre to contact point, base frame
        relative = (foot_velocities(q, dq) + np.cross(gyro, feet)
                    + np.cross(gyro + calf_rates(q, dq), down))
        per_foot = -(relative @ R.T)
        v_kin = weight @ per_foot
        h_kin = float(weight @ heights)
        self.kinematic_velocity, self.kinematic_height = v_kin, h_kin

        if self.height is None or dt == 0.0:
            self.velocity = v_kin.copy() if self.height is None else self.velocity
            self.height = h_kin if self.height is None else self.height
        else:
            specific = None if accel is None else R @ np.asarray(accel, dtype=float)
            if specific is not None and np.isfinite(specific).all():
                self._mean_up_accel = (specific[2] if self._mean_up_accel is None else
                                       self._mean_up_accel + min(1.0, dt / ACCEL_CHECK_TAU)
                                       * (specific[2] - self._mean_up_accel))
                self.accel_ok = ACCEL_UP_RANGE[0] <= self._mean_up_accel <= ACCEL_UP_RANGE[1]
            else:
                self.accel_ok = False
            if self.use_accel and self.accel_ok:
                self.velocity = self.velocity + (specific + GRAVITY) * dt
                gain = min(1.0, dt / VELOCITY_TAU)
            else:
                gain = min(1.0, dt / VELOCITY_TAU_NO_ACCEL)
            self.velocity = self.velocity + gain * (v_kin - self.velocity)
            self.height += min(1.0, dt / HEIGHT_TAU) * (h_kin - self.height)
        speed = np.linalg.norm(self.velocity)
        if not np.isfinite(speed) or speed > MAX_SPEED:
            self.velocity = np.zeros(3) if not np.isfinite(speed) else self.velocity * (MAX_SPEED / speed)
        if not np.isfinite(self.height):
            self.height = HEIGHT_RANGE[0]
        self.height = float(np.clip(self.height, *HEIGHT_RANGE))
        return self.velocity.copy(), self.height
