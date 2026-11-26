#!/usr/bin/env python
"""Check the pendulum rig's hinge encoder, and measure which way it turns. READ-ONLY.

This script only subscribes to rt/lowstate, for the IMU. It creates no publishers and no
SDK clients, so it can't command the robot: leave sport mode on, with the robot standing
or lying still.

    python go2/check_pole.py --robot --pole /dev/ttyACM0 --calibrate   # once: measure the encoder's direction
    python go2/check_pole.py --robot --pole /dev/ttyACM0               # print the pole's readings
    python go2/check_pole.py                                           # go2/sim_headless.py --pendulum

--calibrate asks for the pole hanging still, then held still with its tip toward the
robot's head, and writes the direction to go2/logs/pole_mount.json for run_policy.py
--pole. Either way, the readings are then zeroed on the pole hanging still, as
run_policy.py zeroes them before each walk.

Exit status: 0 = PASS, 1 = FAIL, 2 = couldn't start.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
from check_connection import DEFAULT_INTERFACE, TopicMonitor, check_interface
from pole_encoder import (MAX_POLE_AGE, MOUNT_FILE, SIM_PORT, ZERO_WINDOW_S, PoleMount, PoleReader, load_mount,
                          mount_stamp, pole_state, save_mount, wait_still, wrap)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

MIN_RATE_HZ = 200.0  # run_policy.py calls the policy at up to 100 Hz
CAPTURE_S = 1.0
SETTLE_S = 0.2  # after Enter, before the capture: the hand that pressed it may have just let go
STILL_WAIT_S = 15.0
STILL_STD = math.radians(0.5)
FORWARD_MIN = math.radians(15.0)
FORWARD_MAX = math.radians(120.0)


def imu(monitor):
    msg = monitor.latest
    return np.array(msg.imu_state.quaternion, dtype=float), np.array(msg.imu_state.gyroscope, dtype=float)


def capture(reader, seconds):
    """(circular mean of the raw angle, its standard deviation) over seconds."""
    raws = []
    begin = time.perf_counter() + SETTLE_S
    end = begin + seconds
    while time.perf_counter() < end:
        reader.poll()
        reading = reader.latest()
        if reading is not None and reading[2] < MAX_POLE_AGE and time.perf_counter() >= begin:
            raws.append(reading[0])
        time.sleep(0.005)
    if len(raws) < 10:
        raise RuntimeError(f"only {len(raws)} fresh readings in {seconds:g} s")
    mean = math.atan2(np.sin(raws).mean(), np.cos(raws).mean())
    return mean, float(np.std([wrap(r - mean) for r in raws]))


def ask(prompt):
    try:
        input(prompt)
    except EOFError:
        sys.exit(2)


def calibrate(reader):
    """Measure the encoder's direction; return its mount, or None with the reason printed."""
    ask("\n1. Let the pole hang straight down and come to rest. Press Enter: ")
    raw_down, spread = capture(reader, CAPTURE_S)
    if spread > STILL_STD:
        print(f"FAIL: the pole moved while it was read (spread {math.degrees(spread):.2f} deg). Try again.")
        return None
    ask("2. Hold the pole still with its tip about 45 deg toward the robot's head, "
        "swung forward from hanging. Press Enter: ")
    raw_forward, spread = capture(reader, CAPTURE_S)
    moved = wrap(raw_forward - raw_down)
    if spread > STILL_STD * 4:
        print(f"FAIL: the pole moved while it was read (spread {math.degrees(spread):.1f} deg). Try again.")
        return None
    if not FORWARD_MIN <= abs(moved) <= FORWARD_MAX:
        print(f"FAIL: the encoder turned {math.degrees(moved):+.0f} deg; swing the tip "
              f"{math.degrees(FORWARD_MIN):.0f}-{math.degrees(FORWARD_MAX):.0f} deg forward. Try again.")
        return None
    # From hanging, the tip forward is a smaller forward angle: the hinge angle falls.
    sign = -1.0 if moved > 0 else 1.0
    print(f"   the encoder turned {math.degrees(moved):+.0f} deg, so its direction is {sign:+.0f}")
    return PoleMount(sign=sign, measured=mount_stamp())


def watch(reader, monitor, mount, zero, seconds):
    """Print the pole's readings twice a second; return (readings/s, longest gap, oldest reading) while watching."""
    start = time.perf_counter()
    next_print = start
    oldest = 0.0
    while time.perf_counter() - start < seconds:
        reader.poll()
        reading = reader.latest()
        now = time.perf_counter()
        if reading is not None:
            oldest = max(oldest, reading[2])
        if now >= next_print:
            next_print += 0.5
            if reading is None:
                print("  no reading yet", flush=True)
            else:
                raw, raw_rate, age = reading
                quat, gyro = imu(monitor)
                hinge = mount.hinge(raw, zero)
                theta, rate, _, _, up = pole_state(hinge, mount.sign * raw_rate, quat, gyro)
                print(f"  {math.degrees(math.acos(up)):5.1f} deg from up   theta {theta:+.3f} rad   rate {rate:+6.2f} "
                      f"rad/s   hinge {hinge:+.3f} rad   raw {raw:.4f}   {age * 1000:4.1f} ms old", flush=True)
        time.sleep(0.002)
    return reader.rate_hz(start), reader.worst_gap(start), oldest


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", metavar="INTERFACE", nargs="?", const=DEFAULT_INTERFACE,
                        help="the robot, on this network interface (if none is given: %(const)s); "
                             "without --robot, go2/sim_headless.py on lo")
    parser.add_argument("--domain", type=int, help="DDS domain id (default: 0 with --robot, otherwise 1)")
    parser.add_argument("--pole", metavar="SOURCE",
                        help=f"the encoder: a serial device (/dev/ttyACM0, or /dev/ttyACM0@BAUD), or udp:PORT "
                             f"(default: /dev/ttyACM0 with --robot, otherwise udp:{SIM_PORT})")
    parser.add_argument("--pole-mount", type=Path, metavar="FILE",
                        help="the direction to read with, or with --calibrate to write (default: "
                             "go2/logs/pole_mount.json; for udp: without --calibrate, none)")
    parser.add_argument("--calibrate", action="store_true", help="measure which way the encoder turns first")
    parser.add_argument("--sign", type=int, choices=(1, -1),
                        help="with --calibrate, write this direction instead of measuring it")
    parser.add_argument("--seconds", type=float, default=10.0, help="how long to print readings (default: %(default)s)")
    args = parser.parse_args()
    if args.pole is None:
        args.pole = "/dev/ttyACM0" if args.robot else f"udp:{SIM_PORT}"
    return args


def main():
    args = parse_args()
    interface = args.robot or "lo"
    domain = args.domain if args.domain is not None else (0 if args.robot else 1)
    if args.robot and not check_interface(args.robot):
        return 2
    try:
        reader = PoleReader(args.pole)
    except (OSError, ValueError) as error:
        print(f"FAIL: can't open the encoder {args.pole}: {error}")
        return 2
    print(f"Encoder: {reader.describe()}")
    if not reader.wait(3.0):
        print("FAIL: no reading in 3 s. " + ("Is go2/sim_headless.py running with --pendulum?" if reader.simulated
                                             else "Is the microcontroller plugged in, and sending lines?"))
        return 2
    ChannelFactoryInitialize(domain, interface)
    monitor = TopicMonitor()
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(monitor)
    deadline = time.perf_counter() + 5.0
    while monitor.latest is None:
        if time.perf_counter() > deadline:
            print(f"FAIL: no rt/lowstate on {interface}, domain {domain}, after 5 s; it's needed for the IMU")
            return 2
        reader.poll()
        time.sleep(0.01)

    mount_path = args.pole_mount or MOUNT_FILE
    if args.calibrate:
        mount = PoleMount(sign=float(args.sign), measured=f"--sign {args.sign}") if args.sign else calibrate(reader)
        if mount is None:
            return 1
        save_mount(mount_path, mount, source=args.pole)
        print(f"   Wrote direction {mount.sign:+.0f} to {mount_path}")
    elif args.pole_mount or not reader.simulated:
        try:
            mount = load_mount(mount_path)
        except (OSError, ValueError, KeyError) as error:
            print(f"FAIL: no direction: {error}. Measure it with --calibrate")
            return 2
        print(f"Direction: {mount.sign:+.0f}, from {mount_path} ({mount.measured})")
    else:
        mount = PoleMount()
        print("Direction: the simulator's, +1")

    print(f"\nLet the pole hang still: the readings are zeroed once it has for {ZERO_WINDOW_S:g} s.", flush=True)
    restarts = reader.restarts
    found = wait_still(reader, lambda: imu(monitor)[0], mount.sign, STILL_WAIT_S)
    if found is None:
        print(f"FAIL: the pole didn't hang still for {ZERO_WINDOW_S:g} s within {STILL_WAIT_S:g} s")
        return 1
    zero, spread = found
    print(f"Zeroed: the pole moved {math.degrees(spread):.1f} deg while it hung. Hanging reads 180 deg from up; "
          f"the tip swung forward makes theta smaller and positive, back makes it negative.")
    rate, gap, oldest = watch(reader, monitor, mount, zero, args.seconds)
    problems = []
    if rate < MIN_RATE_HZ:
        problems.append(f"{rate:.0f} readings/s, below {MIN_RATE_HZ:g}")
    if gap > MAX_POLE_AGE:
        problems.append(f"a {gap * 1000:.0f} ms gap between readings: run_policy.py stops a walk after "
                        f"{MAX_POLE_AGE * 1000:.0f} ms")
    if reader.restarts != restarts:
        problems.append(f"the microcontroller restarted {reader.restarts - restarts} times, which loses the zero")
    print(f"\n{rate:.0f} readings/s, longest gap {gap * 1000:.1f} ms, oldest reading {oldest * 1000:.1f} ms, "
          f"{reader.bad_lines} unreadable lines")
    print("PASS" if not problems else "FAIL: " + "; ".join(problems))
    reader.close()
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
