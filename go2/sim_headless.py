#!/usr/bin/env python
"""Run Unitree's Go2 MuJoCo simulator without a viewer, for use over SSH.

It speaks the same DDS topics as the real robot (rt/lowcmd, rt/lowstate,
rt/sportmodestate), on the loopback interface. This replaces unitree_mujoco.py,
which needs a display; the DDS bridge itself is Unitree's, unmodified.

    python go2/sim_headless.py [--duration SECONDS] [--video OUT.mp4]
"""
import argparse
import os
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("MUJOCO_GL", "egl")  # offscreen rendering, for --video

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--unitree-mujoco",
    type=Path,
    default=Path(__file__).resolve().parents[2] / "unitree_mujoco",
    help="path to the unitree_mujoco checkout (default: next to this repo)",
)
parser.add_argument("--domain", type=int, default=1, help="DDS domain id")
parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; 0 = forever")
parser.add_argument("--dt", type=float, default=0.005, help="physics timestep, seconds")
parser.add_argument("--video", type=Path, help="record the run to this .mp4 (needs --duration or Ctrl-C to finish)")
args = parser.parse_args()

sim_dir = args.unitree_mujoco / "simulate_python"
scene = args.unitree_mujoco / "unitree_robots" / "go2" / "scene.xml"
if not scene.exists():
    sys.exit(f"Go2 scene not found at {scene}; pass --unitree-mujoco")
sys.path.insert(0, str(sim_dir))  # the bridge does `import config` from this folder

import mujoco  # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
from unitree_sdk2py_bridge import UnitreeSdk2Bridge  # noqa: E402

model = mujoco.MjModel.from_xml_path(str(scene))
model.opt.timestep = args.dt  # set first: the bridge reads it for its publish rate
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
data.ctrl[:] = 0.0  # the keyframe's ctrl holds joint angles, but these are torque motors

ChannelFactoryInitialize(args.domain, "lo")
bridge = UnitreeSdk2Bridge(model, data)
print(f"sim up: domain={args.domain} interface=lo dt={args.dt}", flush=True)

VIDEO_FPS = 30
renderer = writer = camera = None
if args.video:
    import cv2

    args.video.parent.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, height=480, width=640)
    writer = cv2.VideoWriter(str(args.video), cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, (640, 480))
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = model.body("base_link").id
    camera.distance, camera.azimuth, camera.elevation = 1.4, 135.0, -15.0
frames_written = 0


def record_frame():
    global frames_written
    renderer.update_scene(data, camera)
    writer.write(renderer.render()[:, :, ::-1])  # RGB to BGR
    frames_written += 1


stop = threading.Event()


def report():
    """Print ground truth once a second, independent of anything sent over DDS."""
    while not stop.wait(1.0):
        w, x, y, z = data.qpos[3:7]
        tilt = 1.0 - 2.0 * (x * x + y * y)  # body-up z component; 1 = level
        print(f"t={data.time:7.2f}  base_z={data.qpos[2]:.3f} m  up_z={tilt:+.3f}", flush=True)


threading.Thread(target=report, daemon=True).start()

start = time.perf_counter()
try:
    while not args.duration or time.perf_counter() - start < args.duration:
        step_start = time.perf_counter()
        mujoco.mj_step(model, data)
        if writer is not None and data.time * VIDEO_FPS >= frames_written:
            record_frame()
        remaining = model.opt.timestep - (time.perf_counter() - step_start)
        if remaining > 0:
            time.sleep(remaining)
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    if writer is not None:
        writer.release()
        print(f"video: {args.video} ({frames_written} frames, {data.time:.1f} s sim time)", flush=True)

# Unitree's publisher threads do not stop on their own.
os._exit(0)
