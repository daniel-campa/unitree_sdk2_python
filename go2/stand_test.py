#!/usr/bin/env python
"""Stand the simulated Go2 up and sit it back down through the Unitree SDK.

SIMULATOR ONLY: the interface is fixed to loopback, so this cannot reach a robot.
Real hardware also needs sport mode released and proper safety checks first.

Start go2/sim_headless.py first, then:

    python go2/stand_test.py

Exits 0 if the robot reached standing height and held still and level near the
standing pose, 1 otherwise.
"""
import argparse
import sys
import time

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.utils.crc import CRC

# Joint order: FR, FL, RR, RL, each hip / thigh / calf. Poses are from Unitree's
# stand_go2.py example.
STAND = np.array([0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                  0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763])
SIT = np.array([0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
                0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375])
KP, KD = 50.0, 3.5
DT = 0.002
RAMP_S = 2.0
HOLD_S = 3.0

# Pass criteria for the end of the hold.
# With position gains alone, each joint sags by torque / KP under the robot's
# weight. At KP = 50 that measures up to 0.19 rad (rear hips), so this limit only
# catches gross failures.
MAX_JOINT_ERROR = 0.25  # rad
MIN_HEIGHT = 0.33  # m, at the IMU, which sits 4 cm above the base origin; sitting reads ~0.18
MIN_UP_Z = 0.98  # z component of the body's up vector; 1 = level
MAX_JOINT_SPEED = 0.05  # rad/s


class Latest:
    """Keep the most recent message on a topic."""

    def __init__(self):
        self.msg = None

    def __call__(self, msg):
        self.msg = msg


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--domain", type=int, default=1, help="DDS domain id")
args = parser.parse_args()

ChannelFactoryInitialize(args.domain, "lo")

low_state, sport_state = Latest(), Latest()
ChannelSubscriber("rt/lowstate", LowState_).Init(low_state, 10)
ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(sport_state, 10)
publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
publisher.Init()

deadline = time.perf_counter() + 5.0
while low_state.msg is None or sport_state.msg is None:
    if time.perf_counter() > deadline:
        sys.exit("No state from the simulator after 5 s. Is sim_headless.py running on this domain?")
    time.sleep(0.01)


def joints():
    return np.array([low_state.msg.motor_state[i].q for i in range(12)])


def height():
    return float(sport_state.msg.position[2])


def up_z():
    _, x, y, _ = low_state.msg.imu_state.quaternion  # w, x, y, z
    return float(1.0 - 2.0 * (x * x + y * y))


def joint_speed():
    return max(abs(low_state.msg.motor_state[i].dq) for i in range(12))


cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0], cmd.head[1] = 0xFE, 0xEF
cmd.level_flag = 0xFF
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01
crc = CRC()


def run_phase(start_q, target_q, seconds):
    """Interpolate joint targets from start_q to target_q, at the control rate."""
    steps = int(seconds / DT)
    for step in range(1, steps + 1):
        tick = time.perf_counter()
        q = start_q + (target_q - start_q) * min(step / max(steps, 1), 1.0)
        for i in range(12):
            cmd.motor_cmd[i].q = float(q[i])
            cmd.motor_cmd[i].kp = KP
            cmd.motor_cmd[i].kd = KD
        cmd.crc = crc.Crc(cmd)
        publisher.Write(cmd)
        remaining = DT - (time.perf_counter() - tick)
        if remaining > 0:
            time.sleep(remaining)


start_q, start_z = joints(), height()
print(f"start:      imu_z={start_z:.3f} m", flush=True)

run_phase(start_q, STAND, RAMP_S)
run_phase(STAND, STAND, HOLD_S)
stand_error, stand_z, stand_up, stand_speed = (
    float(np.abs(joints() - STAND).max()), height(), up_z(), joint_speed()
)
print(
    f"standing:   imu_z={stand_z:.3f} m  up_z={stand_up:.3f}  "
    f"max joint error={stand_error:.3f} rad  max speed={stand_speed:.3f} rad/s",
    flush=True,
)

run_phase(STAND, SIT, RAMP_S)
run_phase(SIT, SIT, 1.0)
print(f"sitting:    imu_z={height():.3f} m", flush=True)

passed = (
    stand_error <= MAX_JOINT_ERROR
    and stand_z >= MIN_HEIGHT
    and stand_up >= MIN_UP_Z
    and stand_speed <= MAX_JOINT_SPEED
)
print("PASS" if passed else "FAIL", flush=True)
sys.exit(0 if passed else 1)
