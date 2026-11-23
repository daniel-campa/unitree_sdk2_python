#!/usr/bin/env python
"""Run Unitree's Go2 MuJoCo simulator without a viewer, for use over SSH, or with one.

It speaks the same DDS topics as the real robot (rt/lowcmd, rt/lowstate,
rt/sportmodestate), on the loopback interface. This replaces unitree_mujoco.py,
which needs a display and has none of the options below; the DDS bridge itself is
Unitree's, unmodified.

    python go2/sim_headless.py [--duration SECONDS] [--video OUT.mp4]
    python go2/sim_headless.py --viewer            # watch it in a window; closing the window stops it
"""
import argparse
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

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
parser.add_argument("--video", type=Path,
                    help="record the run to this .mp4, written when the simulator stops (--duration, Ctrl-C or SIGTERM)")
parser.add_argument("--viewer", action="store_true",
                    help="show the simulation in a window that follows the robot; closing it stops the simulator")
parser.add_argument("--flat", action="store_true", help="remove the scene's boxes, leaving only the floor")
parser.add_argument("--lidar", action="store_true", help="publish simulated lidar topics, for check_lidar.py")
parser.add_argument("--odom", action="store_true",
                    help="publish only rt/utlidar/robot_odom, the true base pose, at 50 Hz; much cheaper than --lidar")
parser.add_argument("--box", action="store_true",
                    help="add a 10 cm box ahead and to the left of the robot, like the README's lidar test")
parser.add_argument("--crate", type=float, metavar="DEG",
                    help="add push_box_turn's 0.45 m crate, fixed, 2 m from the robot at this bearing (+ = left)")
args = parser.parse_args()
if args.lidar and args.odom:
    parser.error("--lidar already publishes rt/utlidar/robot_odom, so leave out --odom")
if args.viewer and args.video:
    parser.error("--viewer and --video can't be used together")
if args.viewer:
    # Without a working GPU driver the window is drawn in software (llvmpipe), which would
    # otherwise take every core and starve a run_policy.py running alongside.
    os.environ.setdefault("LP_NUM_THREADS", "2")
else:
    os.environ.setdefault("MUJOCO_GL", "egl")  # offscreen rendering, for --video

sim_dir = args.unitree_mujoco / "simulate_python"
scene = args.unitree_mujoco / "unitree_robots" / "go2" / "scene.xml"
if not scene.exists():
    sys.exit(f"Go2 scene not found at {scene}; pass --unitree-mujoco")
sys.path.insert(0, str(sim_dir))  # the bridge does `import config` from this folder

import mujoco  # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
from unitree_sdk2py_bridge import UnitreeSdk2Bridge  # noqa: E402

BOX_NAME = "lidar_test_box"
CRATE_NAME = "push_box_crate"
if args.box or args.crate is not None:
    spec = mujoco.MjSpec.from_file(str(scene))
    if args.box:
        spec.worldbody.add_geom(name=BOX_NAME, type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0.45, 0.25, 0.05],
                                size=[0.10, 0.20, 0.05], rgba=[0.8, 0.4, 0.2, 1.0])
    if args.crate is not None:
        # S-M-ICRL's scene_push_box_v2.xml crate, at push_box_start_x and turned to face the robot.
        bearing = math.radians(args.crate)
        spec.worldbody.add_geom(name=CRATE_NAME, type=mujoco.mjtGeom.mjGEOM_BOX,
                                pos=[2.0 * math.cos(bearing), 2.0 * math.sin(bearing), 0.225],
                                quat=[math.cos(bearing / 2), 0.0, 0.0, math.sin(bearing / 2)],
                                size=[0.225, 0.30, 0.225], rgba=[0.7, 0.5, 0.3, 1.0])
    model = spec.compile()
else:
    model = mujoco.MjModel.from_xml_path(str(scene))
model.opt.timestep = args.dt  # set first: the bridge reads it for its publish rate
if args.flat:
    keep = {model.geom("floor").id, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, BOX_NAME),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, CRATE_NAME)}
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] == 0 and geom not in keep:  # body 0 is the world
            model.geom_contype[geom] = model.geom_conaffinity[geom] = 0
            model.geom_rgba[geom, 3] = 0.0
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
data.ctrl[:] = 0.0  # the keyframe's ctrl holds joint angles, but these are torque motors

STATE_PERIOD = 0.005  # s, rt/lowstate and rt/sportmodestate
ChannelFactoryInitialize(args.domain, "lo")
# The bridge publishes state at its model's timestep. At 500 Hz its threads hog the GIL and the sim falls behind real time.
model.opt.timestep = max(args.dt, STATE_PERIOD)
bridge = UnitreeSdk2Bridge(model, data)
model.opt.timestep = args.dt
lidar = None
if args.lidar:
    from sim_lidar import SimLidar

    lidar = SimLidar(model, data, args.domain)
elif args.odom:
    from sim_lidar import SimOdom

    lidar = SimOdom(model, data)

VIDEO_FPS = 30
# Rendering while stepping would slow the sim below real time, so poses are kept and rendered at exit.
video_poses = [] if args.video else None

VIEWER_FPS = 60
viewer = None
if args.viewer:
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = model.body("base_link").id
    viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.6, 135.0, -15.0
    with viewer.lock():  # shadows and reflections cost most of the drawing time in software
        viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
viewer_synced = 0.0


def write_video():
    import cv2

    args.video.parent.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, height=480, width=640)
    writer = cv2.VideoWriter(str(args.video), cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, (640, 480))
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = model.body("base_link").id
    camera.distance, camera.azimuth, camera.elevation = 1.4, 135.0, -15.0
    frame = mujoco.MjData(model)  # the bridge's threads still read `data`
    for qpos in video_poses:
        frame.qpos[:] = qpos
        mujoco.mj_kinematics(model, frame)
        renderer.update_scene(frame, camera)
        writer.write(renderer.render()[:, :, ::-1])  # RGB to BGR
    writer.release()
    print(f"video: {args.video} ({len(video_poses)} frames, {len(video_poses) / VIDEO_FPS:.1f} s)", flush=True)


def on_sigterm(signum, frame):
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, on_sigterm)


stop = threading.Event()


def report():
    """Print ground truth once a second, independent of anything sent over DDS."""
    while not stop.wait(1.0):
        w, x, y, z = data.qpos[3:7]
        tilt = 1.0 - 2.0 * (x * x + y * y)  # body-up z component; 1 = level
        print(
            f"t={data.time:7.2f}  base_x={data.qpos[0]:+.3f} m  base_y={data.qpos[1]:+.3f} m  "
            f"base_z={data.qpos[2]:.3f} m  up_z={tilt:+.3f}",
            flush=True,
        )


print(f"sim up: domain={args.domain} interface=lo dt={args.dt}", flush=True)
threading.Thread(target=report, daemon=True).start()

start = time.perf_counter()
try:
    while not args.duration or time.perf_counter() - start < args.duration:
        step_start = time.perf_counter()
        mujoco.mj_step(model, data)
        if lidar is not None:
            lidar.step()
        if video_poses is not None and data.time * VIDEO_FPS >= len(video_poses):
            video_poses.append(data.qpos.copy())
        if viewer is not None and step_start - viewer_synced >= 1.0 / VIEWER_FPS:
            if not viewer.is_running():
                break
            viewer.sync()
            viewer_synced = step_start
        ahead = data.time - (time.perf_counter() - start)  # against the clock, so slow steps are made up
        if ahead > 0:
            time.sleep(ahead)
except KeyboardInterrupt:
    pass
finally:
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    stop.set()
    if viewer is not None:
        viewer.close()
    if video_poses is not None:
        write_video()

# Unitree's publisher threads do not stop on their own.
os._exit(0)
