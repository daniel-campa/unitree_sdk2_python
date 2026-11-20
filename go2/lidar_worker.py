"""Read the lidar in a process of its own, so the 500 Hz control loop never waits for it.

A point cloud takes milliseconds to receive, decode and turn into an env vector (about
8 ms for 33,000 points on a laptop), and the lidar sends 10 or more a second. Done in
go2/run_policy.py's control loop, that would stretch cycles meant to take 2 ms, which is
the kind of delay that fed the policy stale states on 2026-11-19. So a second process
does all of it (go2/lidar_env.py) and shares only small arrays with the control loop:

  control loop -> worker   the IMU's roll and pitch, the estimated base height, the
                           odometry pose (for clouds in the odom frame), and whether
                           the robot is walking (while it is, every cloud's points are kept)
  worker -> control loop   the newest env vector, when its cloud arrived, which topic it
                           came from, how many points it had and which values fell back
                           to flat floor; and twice a second a status summary

The worker writes a readings file of its own when the run ends, beside the run's log
(<log>_lidar.npz): every cloud's arrival, topic, frame and point count, the env vector
built from every usable cloud, and the points of every cloud that arrived while walking
(a subset, in the cloud's own frame, so the mount can be refitted later). Times are
time.perf_counter() readings, the same clock as the run log's meta "clock_start".

It only subscribes, and it ignores Ctrl-C and a closed terminal: go2/run_policy.py stops
it after damping, or it stops by itself when run_policy.py is gone.
"""
import json
import math
import os
import signal
import time
import traceback

import numpy as np

from lidar_env import CLOUD_TOPICS, CloudEnv, EnvLayout, Mount, cloud_stamp

POLL_PERIOD = 0.005  # s between looks for a new cloud
STATUS_PERIOD = 0.5  # s between status messages to the control loop
SAVED_CLOUD_POINTS = 4096  # points kept per cloud while walking (an evenly spaced subset)
MAX_SAVED_CLOUDS = 3000  # about 5 minutes of two topics at 10 Hz
START_TIMEOUT = 20.0  # s for the worker to start before the control loop gives up on it

# The shared array. The control loop writes the inputs; the worker writes the output
# block under a sequence number that is odd while it writes (a seqlock).
IN_ROLL, IN_PITCH, IN_HEIGHT, IN_ODOM, IN_ODOM_OK, IN_SAVING, IN_STOP = 0, 1, 2, 3, 10, 11, 12
OUT = 16
SEQ, T, TOPIC, POINTS, HIT, BUILD_MS, READY = range(OUT, OUT + 7)
ENV = OUT + 8


def shared_size(n_env):
    return ENV + 2 * n_env


class LidarProcess:
    """The control loop's side: same calls as before, nothing in them takes more than a few µs."""

    def __init__(self, interface, domain, layout, mount, max_age, readings_path):
        import multiprocessing

        self.layout = layout
        self.mount = mount
        self.max_age = max_age
        self.topics = list(CLOUD_TOPICS)
        self.readings_path = readings_path
        ctx = multiprocessing.get_context("spawn")  # a fresh process: DDS doesn't survive fork
        self._shared = ctx.Array("d", shared_size(layout.size), lock=False)
        self._shared[IN_HEIGHT] = 0.27
        self._status_in, status_out = ctx.Pipe(duplex=False)
        self._process = ctx.Process(target=_worker, name="lidar", daemon=True, args=(
            interface, domain, layout.as_dict(), mount.as_dict() if mount else None, max_age,
            self._shared, status_out, str(readings_path) if readings_path else None, os.getpid()))
        self._process.start()
        status_out.close()
        self.last_status = {}
        self._next_status = 0.0
        self.stopped = None  # the worker's last word, once stopped

    @property
    def saving(self):
        return bool(self._shared[IN_SAVING])

    @saving.setter
    def saving(self, value):
        self._shared[IN_SAVING] = 1.0 if value else 0.0

    def ready(self):
        return bool(self._shared[READY])

    def wait_ready(self, timeout=START_TIMEOUT):
        end = time.perf_counter() + timeout
        while not self.ready() and self._process.is_alive() and time.perf_counter() < end:
            time.sleep(0.05)
        return self.ready()

    def poll(self, now, rpy, odom_pose):
        s = self._shared
        s[IN_ROLL], s[IN_PITCH] = float(rpy[0]), float(rpy[1])
        if odom_pose is not None:
            s[IN_ODOM:IN_ODOM + 7] = [float(v) for v in odom_pose]
            s[IN_ODOM_OK] = 1.0
        else:
            s[IN_ODOM_OK] = 0.0
        if now >= self._next_status:
            self._next_status = now + STATUS_PERIOD
            self._read_status()

    def _read_status(self):
        try:
            while self._status_in.poll():
                self.last_status = self._status_in.recv()
        except (EOFError, OSError):
            pass

    def _read(self):
        s, n = self._shared, self.layout.size
        for _ in range(5):
            seq = s[SEQ]
            if seq % 2:
                continue
            out = (s[T], int(s[TOPIC]), int(s[POINTS]), bool(s[HIT]), s[BUILD_MS],
                   np.array(s[ENV:ENV + n]), np.array(s[ENV + n:ENV + 2 * n]) > 0.5)
            if s[SEQ] == seq:
                return None if seq == 0 else out
        return None

    def current(self, now):
        out = self._read()
        if out is None or now - out[0] > self.max_age:
            return None
        return out

    def build(self, now, height):
        """(env vector, info, cloud age, topic index), or None without a fresh usable cloud."""
        self._shared[IN_HEIGHT] = float(height)
        out = self.current(now)
        if out is None:
            return None
        t, topic, points, hit, build_ms, env, fallback = out
        return env, {"points": points, "fallback": fallback, "object_hit": hit, "build_ms": build_ms}, now - t, topic

    def status(self, now):
        self._read_status()
        st = self.last_status
        if not self._process.is_alive() and self.stopped is None:
            error = (st.get("error") or "").strip().splitlines()
            return "the lidar process has stopped" + (f": {error[-1]}" if error else "")
        if not st:
            return "the lidar process is starting" if not self.ready() else "no status yet"
        parts = []
        out = self._read()  # the newest env vector's time is current; the status message can be 0.5 s old
        for i, (topic, clouds, usable, newest, problem) in enumerate(zip(
                self.topics, st["clouds"], st["usable"], st["newest_usable"], st["last_problem"])):
            if out is not None and out[1] == i:
                newest = max(newest, out[0])
            if clouds == 0:
                parts.append(f"{topic}: nothing")
            elif usable == 0:
                parts.append(f"{topic}: {clouds} clouds, none usable ({problem})")
            else:
                parts.append(f"{topic}: {usable}/{clouds} usable, newest {now - newest:.2f} s ago")
        return "; ".join(parts)

    def stop(self, timeout=15.0):
        """Tell the worker to save its readings and stop. Returns its final status."""
        if self.stopped is not None:
            return self.stopped
        self._shared[IN_STOP] = 1.0
        self._process.join(timeout)
        self._read_status()
        if self._process.is_alive():
            self._process.terminate()
            self.last_status = dict(self.last_status, error="the lidar process didn't stop in time")
        self.stopped = self.last_status
        return self.stopped

    def meta(self):
        st = dict(self.stopped or self.last_status)
        st.pop("newest_usable", None)
        return {"topics": self.topics, "layout": self.layout.as_dict(), "max_cloud_age": self.max_age,
                "mount": self.mount.as_dict() if self.mount else None,
                "readings_file": str(self.readings_path) if self.readings_path else None, **st}


def _worker(interface, domain, layout_dict, mount_dict, max_age, shared, status_out, readings_path, parent_pid):
    """The lidar process."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # run_policy.py stops it, after damping
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    try:
        os.nice(5)  # the control loop comes first
    except OSError:
        pass
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

    layout, mount = EnvLayout(**layout_dict), Mount.from_dict(mount_dict)
    n = layout.size
    status = {"clouds": [0] * len(CLOUD_TOPICS), "usable": [0] * len(CLOUD_TOPICS), "frames": [],
              "newest_usable": [-math.inf] * len(CLOUD_TOPICS), "last_problem": [None] * len(CLOUD_TOPICS),
              "problems": {}, "worst_cloud_ms": 0.0, "saved_clouds": 0, "error": None}
    arrivals, envs, saved = [], [], []
    try:
        ChannelFactoryInitialize(domain, interface)
        subscribers = []
        for topic in CLOUD_TOPICS:
            sub = ChannelSubscriber(topic, PointCloud2_)
            sub.InitPolled()  # newest message only
            subscribers.append(sub)
        cloud_envs = [CloudEnv(layout, mount) for _ in CLOUD_TOPICS]
        shared[READY] = 1.0
        next_status = 0.0
        while not shared[IN_STOP]:
            if os.getppid() != parent_pid:  # run_policy.py has gone
                break
            now = time.perf_counter()
            for i, sub in enumerate(subscribers):
                msg = sub.ReadLatest()
                if msg is None:
                    continue
                start = time.perf_counter()
                odom = tuple(shared[IN_ODOM:IN_ODOM + 7]) if shared[IN_ODOM_OK] else None
                env = cloud_envs[i]
                usable = env.take(msg, start, shared[IN_ROLL], shared[IN_PITCH], odom)
                frame = env.raw_frame or ""
                if frame not in status["frames"]:
                    status["frames"].append(frame)
                points = 0 if env.raw is None else len(env.raw)
                status["clouds"][i] += 1
                arrivals.append((start, i, status["frames"].index(frame), cloud_stamp(msg), points, usable))
                if usable:
                    status["usable"][i] += 1
                    status["newest_usable"][i] = start
                else:
                    status["last_problem"][i] = env.problem
                    status["problems"][env.problem] = status["problems"].get(env.problem, 0) + 1
                if status["clouds"][i] == 1 or (usable and status["usable"][i] == 1):
                    next_status = 0.0  # tell the control loop about a topic's first cloud straight away
                if shared[IN_SAVING] and env.raw is not None and len(saved) < MAX_SAVED_CLOUDS:
                    step = max(1, int(math.ceil(points / SAVED_CLOUD_POINTS)))
                    saved.append((start, i, shared[IN_ROLL], shared[IN_PITCH], env.raw[::step].astype(np.float32)))
                if usable:
                    vector, info = env.build(shared[IN_HEIGHT])
                    done = time.perf_counter()
                    envs.append((start, i, shared[IN_HEIGHT], vector, info["fallback"], info["points"]))
                    # Publish it if it's from the topic the control loop should use: the first
                    # in CLOUD_TOPICS order with a usable cloud within max_age.
                    first = next((j for j, e in enumerate(cloud_envs) if e.age(done) <= max_age), i)
                    if first == i:
                        shared[SEQ] += 1
                        shared[T], shared[TOPIC], shared[POINTS] = start, i, info["points"]
                        shared[HIT], shared[BUILD_MS] = float(info["object_hit"]), (done - start) * 1000.0
                        shared[ENV:ENV + n] = [float(v) for v in vector]
                        shared[ENV + n:ENV + 2 * n] = [float(v) for v in info["fallback"]]
                        shared[SEQ] += 1
                status["worst_cloud_ms"] = max(status["worst_cloud_ms"], (time.perf_counter() - start) * 1000.0)
            now = time.perf_counter()
            if now >= next_status:
                next_status = now + STATUS_PERIOD
                status["saved_clouds"] = len(saved)
                status_out.send(status)
            time.sleep(POLL_PERIOD)
    except Exception:
        status["error"] = traceback.format_exc(limit=3)
    finally:
        status["saved_clouds"] = len(saved)
        if readings_path:
            try:
                _save(readings_path, layout, mount, max_age, status, arrivals, envs, saved)
                status["readings_saved"] = True
            except Exception:
                status["error"] = (status["error"] or "") + traceback.format_exc(limit=3)
        try:
            status_out.send(status)
        except (BrokenPipeError, OSError):
            pass


def _save(path, layout, mount, max_age, status, arrivals, envs, saved):
    out = {}
    if arrivals:
        t, topic, frame, stamp, points, usable = (np.array(c) for c in zip(*arrivals))
        out.update(arrival_t=t, arrival_topic=topic, arrival_frame=frame, arrival_stamp=stamp,
                   arrival_points=points, arrival_usable=usable)
    if envs:
        t, topic, height, vector, fallback, points = (np.array(c) for c in zip(*envs))
        out.update(env_t=t, env_topic=topic, env_height=height, env=vector, env_fallback=fallback,
                   env_points=points)
    if saved:
        t, topic, roll, pitch, clouds = zip(*saved)
        out.update(cloud_t=np.array(t), cloud_topic=np.array(topic), cloud_rpy=np.column_stack((roll, pitch)),
                   cloud_offsets=np.cumsum([0] + [len(c) for c in clouds]), cloud_points=np.concatenate(clouds))
    st = {k: v for k, v in status.items() if k != "newest_usable"}
    meta = {"topics": list(CLOUD_TOPICS), "frames": status["frames"], "layout": layout.as_dict(),
            "mount": mount.as_dict() if mount else None, "max_cloud_age": max_age, "status": st,
            "clock": "time.perf_counter(), as the run log's meta clock_start",
            "columns": "arrival_*: every cloud; env*: the env vector from every usable cloud, built with "
                       "env_height; cloud_*: the points of the clouds that arrived while walking, in the cloud's "
                       "own frame (cloud i is cloud_points[cloud_offsets[i]:cloud_offsets[i+1]]), with the "
                       "IMU's roll and pitch at the time"}
    np.savez_compressed(path, meta=json.dumps(meta, default=str), **out)
