#!/usr/bin/env python
"""Check that this computer can hear a Go2 over Ethernet. READ-ONLY.

This script only subscribes to state topics. It creates no publishers and no
SDK clients, so it cannot command the robot.

    python go2/check_connection.py <interface>            # real robot, e.g. enp3s0
    python go2/check_connection.py lo --domain 1          # go2/sim_headless.py

Exit status: 0 = healthy state received, 1 = no state or invalid state,
2 = bad network interface.
"""
import argparse
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

JOINTS = [f"{leg}_{part}" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
ROBOT_SUBNET = "192.168.123."


class TopicMonitor:
    """Count messages on a topic and keep the latest one."""

    def __init__(self):
        self.count = 0
        self.latest = None
        self._lock = threading.Lock()

    def __call__(self, msg):
        with self._lock:
            self.count += 1
            self.latest = msg

    def snapshot(self):
        with self._lock:
            return self.count, self.latest


def check_interface(name):
    """Return False if the interface can't work; warn about likely setup mistakes."""
    device = Path("/sys/class/net") / name
    if not device.exists():
        available = ", ".join(sorted(p.name for p in Path("/sys/class/net").iterdir()))
        print(f"ERROR: no network interface called {name!r}. Available: {available}")
        return False
    if name == "lo":
        return True

    if (device / "operstate").read_text().strip() == "down":
        print(f"ERROR: {name} is down. Check the cable and that the robot is powered on.")
        return False

    output = subprocess.run(
        ["ip", "-o", "-4", "addr", "show", name], capture_output=True, text=True, check=False
    ).stdout
    if ROBOT_SUBNET not in output:
        print(f"WARNING: {name} has no {ROBOT_SUBNET}x address, so the robot's state may not arrive.")
    return True


def state_problems(msg):
    """Return reasons this LowState can't be trusted."""
    problems = []
    for i, joint in enumerate(JOINTS):
        motor = msg.motor_state[i]
        if not (math.isfinite(motor.q) and math.isfinite(motor.dq)):
            problems.append(f"{joint} reading is not finite")
    norm = math.sqrt(sum(value * value for value in msg.imu_state.quaternion))
    if not 0.95 <= norm <= 1.05:
        problems.append(f"IMU quaternion norm is {norm:.3f}, expected about 1")
    return problems


def print_state(msg):
    print("\nJoint        angle (rad)  temp (C)  lost")
    for i, joint in enumerate(JOINTS):
        motor = msg.motor_state[i]
        print(f"{joint:12s} {motor.q:+10.3f}  {motor.temperature:8d}  {motor.lost:4d}")

    roll, pitch, yaw = msg.imu_state.rpy
    print(f"\nIMU rpy:      {roll:+.3f} {pitch:+.3f} {yaw:+.3f} rad")
    print(f"Battery:      {msg.bms_state.soc}%  {msg.power_v:.1f} V  {msg.power_a:.1f} A")
    print(f"Foot force:   {list(msg.foot_force)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("interface", help="network interface wired to the robot")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (the robot uses 0)")
    parser.add_argument("--seconds", type=float, default=5.0, help="how long to listen")
    args = parser.parse_args()

    if not check_interface(args.interface):
        return 2

    ChannelFactoryInitialize(args.domain, args.interface)
    low_state, sport_state = TopicMonitor(), TopicMonitor()
    ChannelSubscriber("rt/lowstate", LowState_).Init(low_state, 10)
    ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(sport_state, 10)

    print(f"Listening on {args.interface}, domain {args.domain}, for {args.seconds:g} s (read-only)...")
    time.sleep(args.seconds)

    low_count, low_msg = low_state.snapshot()
    sport_count, sport_msg = sport_state.snapshot()
    print(f"rt/lowstate:       {low_count} messages ({low_count / args.seconds:.0f} Hz)")
    print(f"rt/sportmodestate: {sport_count} messages ({sport_count / args.seconds:.0f} Hz)")

    if low_msg is None:
        print("\nFAIL: no rt/lowstate received. Check the robot's power, the cable, the PC's address and --domain.")
        return 1

    print_state(low_msg)
    if sport_msg is None:
        print("\nNo rt/sportmodestate: this is expected if sport mode is off.")
    else:
        print(f"\nSport mode:   mode={sport_msg.mode}  body_height={sport_msg.body_height:.3f} m")

    problems = state_problems(low_msg)
    if problems:
        print("\nFAIL: invalid state:")
        for problem in problems:
            print(f"  - {problem}")
        return 1

    print("\nPASS: receiving valid robot state.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
