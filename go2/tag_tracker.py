#!/usr/bin/env python
"""Measure where the Go2 walked from a phone video of AprilTags. Never touches the robot.

The robot has no position measurement once run_policy.py releases sport mode
(check_odom.py showed that every odometry topic stops). This script measures it from
outside instead. A phone on a tripod films the lane, recorded with phone_camera.py.
One tag is on the robot's back and one lies flat on the floor. For push_box_turn a
third lies flat on the crate's top, and is tracked when it's in the picture. Print
them with go2/make_tag_sheets.py.

    python go2/tag_tracker.py calibrate CALIB.mp4                 # once per phone and zoom; writes go2/logs/camera.json
    python go2/tag_tracker.py calibrate CALIB.mp4 --live URL      # records CALIB.mp4 first, with a live window
    python go2/tag_tracker.py track WALK.mp4                      # robot tag relative to the floor tag
    python go2/tag_tracker.py track WALK.mp4 --run go2/logs/RUN.npz   # also report the policy's walk

CALIB.mp4 is a phone_camera.py recording of the printed checkerboard, held flat and
moved slowly all over the picture at different tilts. Record it with the same
--max-width as the walk videos, because the calibration belongs to that picture size.
With --live URL, calibrate records it from the stream itself, as phone_camera.py
record does, while a window shows the board and the parts of the picture it hasn't
reached yet. Press q in the window to stop recording and calibrate.

track writes WALK_track.npz (and WALK_track.png) next to the video. Positions are in
the floor tag's frame: forward along its arrow, left of it, up from the floor, and the
robot tag's yaw from the arrow (anticlockwise seen from above). The crate tag's are the
same columns with a crate_ prefix, NaN where it wasn't seen. Times are the frames'
arrival times from WALK.json, on time.perf_counter() like run_policy.py's log, minus
--latency. With --run, it reports the forward and left distance over the log's
walking stage, along the robot's heading at the start, and the walk reward's
distance terms as check_odom.py computes them.

Exit status: 0 = done, 1 = calibration or tracking failed.
"""
import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

LOG_DIR = Path(__file__).resolve().parent / "logs"
DEFAULT_CAMERA = LOG_DIR / "camera.json"

ROBOT_TAG_ID, ROBOT_TAG_M = 0, 0.120  # black square edge, as printed by make_tag_sheets.py
FLOOR_TAG_ID, FLOOR_TAG_M = 1, 0.140
CRATE_TAG_ID, CRATE_TAG_M = 2, 0.140
BOARD_INNER = (6, 8)  # inner corners of make_tag_sheets.py's 7 x 9 board
BOARD_SQUARE_MM = 25.0

CALIB_SAMPLES = 120  # frames looked at for the board
CALIB_MIN_VIEWS = 12
CALIB_MIN_SHIFT_PX = 25.0  # a view must move the board this far from every kept view
CALIB_MAX_RMS_PX = 1.0
CALIB_MIN_COVERAGE = 0.5  # fraction of the picture the board's corners should reach

# Floor tag corners further than this from their median: the camera or the tag moved.
# Set on 960-wide pictures, so it grows with the picture's width.
FLOOR_MOVED_PX = 3.0
FLOOR_MOVED_WIDTH = 960
FLOOR_MOVED_FRACTION = 0.05
MIN_ROBOT_DETECTION = 0.5  # of frames, below which tracking fails
ENDPOINT_S = 0.25  # positions at the ends of the walk are line fits over this long either side
REWARD_FILTERS = (0.0, 0.5)  # s; frame-to-frame noise counts as sideways and backward steps, so raw reads low


class TrackError(Exception):
    pass


def tag_object_points(side):
    """Corner order of OpenCV's detector and SOLVEPNP_IPPE_SQUARE: x right, y towards the arrow."""
    h = side / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def read_frames(path, every=1):
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise TrackError(f"can't open {path}")
    index = 0
    try:
        while True:
            if not capture.grab():
                return
            if index % every == 0:
                ok, frame = capture.retrieve()
                if not ok:
                    return
                yield index, frame
            index += 1
    finally:
        capture.release()


def frame_count(path):
    capture = cv2.VideoCapture(str(path))
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    capture.release()
    return count


# Recording the board live ----------------------------------------------------------
LIVE_WINDOW = "tag_tracker.py calibrate (q stops and calibrates)"
LIVE_GRID = (6, 4)  # columns and rows; a cell is reached once a board corner has been in it
LIVE_SEARCH_WIDTH = 960  # the window searches a smaller copy of each frame, to keep up with the stream
LIVE_SHOW_WIDTH = 1280


class BoardWatch:
    """Searches the newest frame for the board in the background, and remembers where the board has been."""

    def __init__(self, size):
        self.size = size
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closing = False
        self._frame = None
        self._corners = None  # in the newest frame searched; None if the board wasn't found there
        self._cells = np.zeros((LIVE_GRID[1], LIVE_GRID[0]), bool)
        self._low = self._high = None  # the box around every corner found so far
        self._places = []  # board centres at least CALIB_MIN_SHIFT_PX apart, as calibrate keeps views
        self._thread = threading.Thread(target=self._search, daemon=True)
        self._thread.start()

    def offer(self, frame):
        with self._lock:
            self._frame = frame
        self._wake.set()

    def close(self):
        self._closing = True
        self._wake.set()
        self._thread.join()

    def state(self):
        """The board's corners or None, the cells reached, the fraction of the picture reached as calibrate
        measures it, and the number of places."""
        with self._lock:
            reach = 0.0 if self._low is None else float(np.prod(self._high - self._low) / np.prod(self.size))
            return self._corners, self._cells.copy(), reach, len(self._places)

    def _search(self):
        while True:
            self._wake.wait()
            self._wake.clear()
            if self._closing:
                return
            with self._lock:
                frame, self._frame = self._frame, None
            if frame is None:
                continue
            scale = min(1.0, LIVE_SEARCH_WIDTH / frame.shape[1])
            small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            found, corners = cv2.findChessboardCornersSB(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), BOARD_INNER,
                                                         flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            corners = corners.reshape(-1, 2) / scale if found else None
            with self._lock:
                self._corners = corners
                if corners is not None:
                    self._remember(corners)

    def _remember(self, corners):
        columns = np.clip((corners[:, 0] * LIVE_GRID[0] / self.size[0]).astype(int), 0, LIVE_GRID[0] - 1)
        rows = np.clip((corners[:, 1] * LIVE_GRID[1] / self.size[1]).astype(int), 0, LIVE_GRID[1] - 1)
        self._cells[rows, columns] = True
        low, high = corners.min(axis=0), corners.max(axis=0)
        self._low = low if self._low is None else np.minimum(self._low, low)
        self._high = high if self._high is None else np.maximum(self._high, high)
        centre = corners.mean(axis=0)
        if all(np.linalg.norm(centre - c) >= CALIB_MIN_SHIFT_PX for c in self._places):
            self._places.append(centre)


def live_view(frame, watch, seconds):
    """The window's picture: the frame, the board where it was last found, the cells it hasn't reached in red,
    and a status bar below."""
    corners, cells, reach, places = watch.state()
    scale = min(1.0, LIVE_SHOW_WIDTH / frame.shape[1])
    view = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    h, w = view.shape[:2]
    red = view.copy()
    for row, column in zip(*np.nonzero(~cells)):
        cv2.rectangle(red, (column * w // LIVE_GRID[0], row * h // LIVE_GRID[1]),
                      ((column + 1) * w // LIVE_GRID[0], (row + 1) * h // LIVE_GRID[1]), (0, 0, 255), -1)
    view = cv2.addWeighted(red, 0.3, view, 0.7, 0)
    for column in range(1, LIVE_GRID[0]):
        cv2.line(view, (column * w // LIVE_GRID[0], 0), (column * w // LIVE_GRID[0], h), (220, 220, 220), 1)
    for row in range(1, LIVE_GRID[1]):
        cv2.line(view, (0, row * h // LIVE_GRID[1]), (w, row * h // LIVE_GRID[1]), (220, 220, 220), 1)
    if corners is not None:
        cv2.drawChessboardCorners(view, BOARD_INNER, (corners * scale).astype(np.float32).reshape(-1, 1, 2), True)
    lines = [f"{seconds:.0f} s   board {'found' if corners is not None else 'not found'}   reached {reach:.0%} of the "
             f"picture (needs {CALIB_MIN_COVERAGE:.0%})   {places} places (needs {CALIB_MIN_VIEWS})",
             "Move the board slowly into every red cell, corners included, tilted different ways.",
             "Press q to stop and calibrate."]
    bar = np.full((14 + 26 * len(lines), w, 3), 40, np.uint8)
    for i, text in enumerate(lines):
        cv2.putText(bar, text, (12, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([view, bar])


def record_calibration(source, video, max_width=None):
    """Record source to video as phone_camera.py record does, while a window shows where the board has been.

    Returns once q or Esc is pressed in the window, the window is closed, or Ctrl-C.
    """
    from phone_camera import FPS, MAX_WIDTH, CameraError, Stream, fitted, output_size

    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise TrackError("--live needs a screen for its window. Record with phone_camera.py record, then calibrate "
                         "the recording.")
    try:
        stream = Stream(source)
    except CameraError as error:
        raise TrackError(str(error)) from None
    stop = threading.Event()
    previous = signal.signal(signal.SIGINT, lambda signum, frame: stop.set())
    writer = watch = None
    try:
        stream.wait_first(stop)
        size = output_size(stream.latest()[0], max_width or MAX_WIDTH)
        video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), FPS, size)
        if not writer.isOpened():
            raise TrackError(f"can't write {video}")
        watch = BoardWatch(size)
        print(f"Recording {source} to {video}, {size[0]}x{size[1]}. Press q in the window to stop and calibrate.",
              flush=True)
        written, start, opened = 0, time.perf_counter(), False
        while not stop.is_set():
            frame = fitted(stream.latest()[0], size)
            writer.write(frame)
            written += 1
            watch.offer(frame)
            view = live_view(frame, watch, time.perf_counter() - start)
            if written == 1:
                cv2.namedWindow(LIVE_WINDOW, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(LIVE_WINDOW, view.shape[1], view.shape[0])
            cv2.imshow(LIVE_WINDOW, view)
            key = cv2.waitKey(max(1, round((start + written / FPS - time.perf_counter()) * 1000))) & 0xFF
            visible = cv2.getWindowProperty(LIVE_WINDOW, cv2.WND_PROP_VISIBLE) >= 1
            # Closed only once it has been seen open, in case this backend never reports it visible.
            if key in (ord("q"), ord("Q"), 27) or (opened and not visible):
                break
            opened = opened or visible
    except CameraError as error:
        raise TrackError(str(error)) from None
    finally:
        signal.signal(signal.SIGINT, previous)
        if writer is not None:
            writer.release()
        if watch is not None:
            watch.close()
        stream.close()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
    print(f"Recorded {time.perf_counter() - start:.0f} s.", flush=True)


# Calibration ---------------------------------------------------------------------
def calibrate(args):
    if args.live:
        record_calibration(args.live, args.video, args.max_width)
    total = frame_count(args.video)
    every = max(1, total // CALIB_SAMPLES)
    board = np.zeros((BOARD_INNER[0] * BOARD_INNER[1], 3), np.float32)
    board[:, :2] = np.mgrid[0:BOARD_INNER[0], 0:BOARD_INNER[1]].T.reshape(-1, 2) * args.square_mm / 1000.0
    views, centres, size, looked = [], [], None, 0
    print(f"Looking for the board in every {every}th of {total} frames ...", flush=True)
    for _, frame in read_frames(args.video, every):
        looked += 1
        if looked % 20 == 0:
            print(f"  {looked} of {total // every + 1} frames looked at, board in {len(views)} distinct views", flush=True)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCornersSB(gray, BOARD_INNER, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            continue
        centre = corners.reshape(-1, 2).mean(axis=0)
        if any(np.linalg.norm(centre - c) < CALIB_MIN_SHIFT_PX for c in centres):
            continue
        views.append(corners)
        centres.append(centre)
    print(f"Board found in {len(views)} distinct views of {looked} frames looked at.")
    if len(views) < CALIB_MIN_VIEWS:
        raise TrackError(f"need at least {CALIB_MIN_VIEWS} views. Move the board slowly over the whole "
                         "picture, at different tilts, and keep all of it in view.")
    rms, matrix, dist, _, _ = cv2.calibrateCamera([board] * len(views), views, size, None, None)
    reach = np.concatenate([v.reshape(-1, 2) for v in views])
    coverage = float((reach[:, 0].max() - reach[:, 0].min()) * (reach[:, 1].max() - reach[:, 1].min())
                     / (size[0] * size[1]))
    camera = {"size": list(size), "matrix": matrix.tolist(), "dist": dist.ravel().tolist(), "rms_px": rms,
              "views": len(views), "coverage": coverage, "square_mm": args.square_mm, "video": str(args.video)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(camera, indent=1))
    fov = 2 * math.degrees(math.atan(size[0] / 2 / matrix[0, 0]))
    print(f"Picture {size[0]}x{size[1]}, focal length {matrix[0, 0]:.0f} px ({fov:.0f} deg wide), "
          f"reprojection error {rms:.2f} px, corners reach {coverage:.0%} of the picture.")
    print(f"Wrote {args.out}")
    problems = []
    if rms > CALIB_MAX_RMS_PX:
        problems.append(f"reprojection error {rms:.2f} px is above {CALIB_MAX_RMS_PX:g}: blurred frames, "
                        "a board that isn't flat, or the wrong --square-mm")
    if coverage < CALIB_MIN_COVERAGE:
        problems.append(f"the board only reached {coverage:.0%} of the picture; take it closer to the edges")
    if problems:
        print("FAIL: " + "; ".join(problems))
        return 1
    print("PASS")
    return 0


def load_camera(path, size):
    """Camera matrix and distortion for this picture size, scaled if only the resolution differs."""
    if not path.exists():
        raise TrackError(f"no camera calibration at {path}. Run: python go2/tag_tracker.py calibrate CALIB.mp4")
    camera = json.loads(path.read_text())
    matrix, dist = np.array(camera["matrix"]), np.array(camera["dist"])
    width, height = camera["size"]
    if (width, height) != tuple(size):
        sx, sy = size[0] / width, size[1] / height
        if abs(sx - sy) > 0.01:
            raise TrackError(f"{path} is for {width}x{height} pictures, but the video is {size[0]}x{size[1]}, "
                             "a different shape. Calibrate again with the same --max-width.")
        matrix = matrix * np.array([[sx], [sy], [1.0]])
        print(f"Scaled the {width}x{height} calibration to {size[0]}x{size[1]}.")
    return matrix, dist


# Tracking ------------------------------------------------------------------------
def make_detector():
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.ArucoDetector(dictionary, params)


def detect_all(path, tag_ids, annotate=None):
    """Corners of each of tag_ids in each frame, (N, 4, 2) per ID (NaN where unseen), and the picture size."""
    detector = make_detector()
    found, size = {tag_id: [] for tag_id in tag_ids}, None
    for _, frame in read_frames(path):
        size = frame.shape[1::-1]
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        seen = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}
        for tag_id, track in found.items():
            track.append(seen.get(tag_id, np.full((4, 2), np.nan)))
        if annotate is not None:
            annotate.append(frame)
    if size is None:
        raise TrackError(f"{path} has no frames")
    return {tag_id: np.array(track) for tag_id, track in found.items()}, size


def solve_tag(corners, side, matrix, dist):
    """Both IPPE solutions as (R, t, reprojection error), best first."""
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(tag_object_points(side), corners.astype(np.float64),
                                                    matrix, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return []
    return [(cv2.Rodrigues(r)[0], t.ravel(), float(e)) for r, t, e in zip(rvecs, tvecs, np.ravel(errors))]


def floor_pose(floor, matrix, dist, side, width):
    """The floor tag's pose from its median corners, which assumes the camera stayed put."""
    seen = ~np.isnan(floor[:, 0, 0])
    if not seen.any():
        raise TrackError(f"the floor tag (ID {FLOOR_TAG_ID}) was never seen")
    median = np.median(floor[seen], axis=0)
    deviation = np.abs(floor[seen] - median).max(axis=(1, 2))
    moved_px = FLOOR_MOVED_PX * width / FLOOR_MOVED_WIDTH
    moved = float((deviation > moved_px).mean())
    solutions = solve_tag(median, side, matrix, dist)
    if not solutions:
        raise TrackError("couldn't solve the floor tag's pose")
    rotation, translation, error = solutions[0]
    info = {"seen_fraction": float(seen.mean()), "moved_fraction": moved, "moved_px": moved_px,
            "jitter_px": float(np.median(deviation)), "reprojection_px": error,
            "distance_m": float(np.linalg.norm(translation)),
            "view_angle_deg": math.degrees(math.acos(abs(rotation[2, 2])))}
    if len(solutions) > 1:
        info["ambiguity"] = solutions[1][2] / max(error, 1e-9)
    return rotation, translation, info


def tag_poses(corners, floor_rotation, floor_translation, matrix, dist, side):
    """Positions and angles of a tag lying nearly flat, in the floor tag's frame, NaN where unseen."""
    n = len(corners)
    out = {k: np.full(n, np.nan) for k in ("forward", "left", "up", "yaw", "tilt", "reprojection_px")}
    floor_up = floor_rotation[:, 2]
    for i in np.flatnonzero(~np.isnan(corners[:, 0, 0])):
        solutions = solve_tag(corners[i], side, matrix, dist)
        if not solutions:
            continue
        # The tag lies nearly flat, on the robot's back or the crate's top, so of IPPE's
        # two solutions, take the one whose face points most nearly the same way as the floor's.
        rotation, translation, error = max(solutions, key=lambda s: s[0][:, 2] @ floor_up)
        position = floor_rotation.T @ (translation - floor_translation)
        in_floor = floor_rotation.T @ rotation
        heading = in_floor[:, 1]
        out["forward"][i], out["left"][i], out["up"][i] = position[1], -position[0], position[2]
        out["yaw"][i] = math.atan2(-heading[0], heading[1])
        out["tilt"][i] = math.acos(np.clip(in_floor[2, 2], -1.0, 1.0))
        out["reprojection_px"][i] = error
    return out


def frame_times(video, count, latency):
    timeline_path = video.with_suffix(".json")
    if not timeline_path.exists():
        print(f"No {timeline_path.name}, so times count from the first frame at the video's frame rate, "
              "not on run_policy.py's clock.")
        capture = cv2.VideoCapture(str(video))
        fps = capture.get(cv2.CAP_PROP_FPS) or 15.0
        capture.release()
        return np.arange(count) / fps, False
    received = np.array(json.loads(timeline_path.read_text())["received_at"], float)
    if len(received) < count:
        raise TrackError(f"{timeline_path.name} has {len(received)} times for {count} frames")
    return received[:count] - latency, True


def load_walk(path):
    """Start and end of the walking stage on time.perf_counter(), and the run's outcome."""
    log = np.load(path, allow_pickle=True)
    meta = json.loads(str(log["meta"]))
    stages = np.array([str(s) for s in log["stage"]])
    walking = np.flatnonzero(stages == "walking")
    if len(walking) == 0:
        raise TrackError(f"{path} has no walking stage (outcome: {meta.get('outcome')})")
    t = log["t"] + meta["clock_start"]
    return t[walking[0]], t[walking[-1]], meta.get("outcome", "?")


def value_at(t, values, at, seen):
    """A straight line fitted to the frames within ENDPOINT_S of at, evaluated at at; None if none."""
    window = seen & (np.abs(t - at) <= ENDPOINT_S)
    if not window.any():
        return None
    if window.sum() == 1:
        return values[window][0]
    slope, offset = np.polyfit(t[window] - at, values[window], 1)
    return offset


def report_walk(t, track, seen, start, end, outcome):
    print(f"\nWalk {end - start:.1f} s ({outcome})")
    inside = (t >= start) & (t <= end)
    if not inside.any():
        raise TrackError("the video doesn't cover the walk. Check that both ran on this computer, "
                         "since perf_counter() times only line up on one machine.")
    print(f"  robot tag seen in {seen[inside].mean():.0%} of the walk's frames")
    xy = np.column_stack([track["forward"], track["left"]])
    first, last = value_at(t, xy, start, seen), value_at(t, xy, end, seen)
    yaw = np.full(len(t), np.nan)
    yaw[seen] = np.unwrap(track["yaw"][seen])
    yaw_first, yaw_last = value_at(t, yaw, start, seen), value_at(t, yaw, end, seen)
    if first is None or last is None:
        print(f"  robot tag not seen within {ENDPOINT_S:g} s of the start or the end, so no distance")
        return
    c, s = math.cos(yaw_first), math.sin(yaw_first)
    d = last - first
    forward, left = d[0] * c + d[1] * s, -d[0] * s + d[1] * c
    turned = math.degrees(yaw_last - yaw_first)
    print(f"  along the starting heading: {forward:+.3f} m forward, {left:+.3f} m left, turned {turned:+.1f} deg")
    print(f"  (in the floor tag's frame: {d[0]:+.3f} m forward, {d[1]:+.3f} m left)")
    use = inside & seen
    if use.sum() < 2:
        return
    from check_odom import EPISODE_S, noise_reward
    tw = t[use]
    rel = xy[use] - first
    f, l = rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c
    rewards = "  ".join(f"{w:g} s filter {noise_reward(tw, f, l, w):+.2f}" for w in REWARD_FILTERS)
    print(f"  walk reward distance terms per {EPISODE_S:g} s, stability gate assumed open: {rewards}")


def save_plot(path, t, track, seen, walk):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    t0 = t[0]
    rows = [("forward", "forward (m)", 1.0), ("left", "left (m)", 1.0), ("yaw", "yaw (deg)", 180 / math.pi),
            ("up", "tag height (m)", 1.0)]
    for axis, (key, label, scale) in zip(axes, rows):
        axis.plot(t[seen] - t0, track[key][seen] * scale, ".", ms=2)
        axis.set_ylabel(label)
        axis.grid(alpha=0.3)
        if walk is not None:
            axis.axvspan(walk[0] - t0, walk[1] - t0, color="tab:blue", alpha=0.15)
    axes[-1].set_xlabel("time since the first frame (s); blue = walking stage")
    figure.tight_layout()
    figure.savefig(path, dpi=110)
    plt.close(figure)
    print(f"Plot: {path}")


def write_annotated(path, frames, robot, floor, crate, matrix, dist, floor_rotation, floor_translation, fps):
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    rvec = cv2.Rodrigues(floor_rotation)[0]
    for frame, r, f, c in zip(frames, robot, floor, crate):
        for corners, colour in ((r, (0, 0, 255)), (f, (0, 255, 0)), (c, (255, 0, 0))):
            if not np.isnan(corners[0, 0]):
                cv2.polylines(frame, [corners.astype(np.int32)], True, colour, 2)
        cv2.drawFrameAxes(frame, matrix, dist, rvec, floor_translation, 0.3, 2)
        writer.write(frame)
    writer.release()
    print(f"Annotated video: {path} (red = robot tag, green = floor tag, blue = crate tag; "
          f"floor axes x red, y green, z blue)")


def track(args):
    frames = [] if args.annotate else None
    corners, size = detect_all(args.video, (args.robot_id, args.floor_id, args.crate_id), frames)
    robot, floor, crate = corners[args.robot_id], corners[args.floor_id], corners[args.crate_id]
    matrix, dist = load_camera(args.camera, size)
    t, on_run_clock = frame_times(args.video, len(robot), args.latency)
    floor_rotation, floor_translation, floor_info = floor_pose(floor, matrix, dist, args.floor_size_m, size[0])
    track = tag_poses(robot, floor_rotation, floor_translation, matrix, dist, args.robot_size_m)
    seen = ~np.isnan(track["forward"])
    crate_track = tag_poses(crate, floor_rotation, floor_translation, matrix, dist, args.crate_size_m)
    crate_seen = ~np.isnan(crate_track["forward"])

    print(f"{len(t)} frames over {t[-1] - t[0]:.1f} s. Robot tag seen in {seen.mean():.0%}, "
          f"floor tag in {floor_info['seen_fraction']:.0%}.")
    print(f"Floor tag {floor_info['distance_m']:.2f} m from the camera, seen {floor_info['view_angle_deg']:.0f} deg "
          f"off straight-on, corner jitter {floor_info['jitter_px']:.1f} px.")
    if floor_info["moved_fraction"] > FLOOR_MOVED_FRACTION:
        print(f"WARNING: in {floor_info['moved_fraction']:.0%} of the frames the floor tag was over "
              f"{floor_info['moved_px']:g} px from where it usually is. The camera or the tag moved, or the robot "
              "covered part of it, and positions will be off.")
    if seen.any():
        reproj = np.nanmedian(track["reprojection_px"])
        print(f"Robot tag reprojection error median {reproj:.2f} px, tilt median "
              f"{math.degrees(np.nanmedian(track['tilt'])):.1f} deg, height median {np.nanmedian(track['up']):.3f} m.")
        i, j = np.flatnonzero(seen)[[0, -1]]
        print(f"First seen at {track['forward'][i]:+.3f} m forward, {track['left'][i]:+.3f} m left; "
              f"last at {track['forward'][j]:+.3f} m forward, {track['left'][j]:+.3f} m left.")
    if crate_seen.any():
        xy = np.column_stack([crate_track["forward"], crate_track["left"]])[crate_seen]
        middle = np.median(xy, axis=0)
        print(f"Crate tag seen in {crate_seen.mean():.0%} of frames, at {middle[0]:+.3f} m forward, "
              f"{middle[1]:+.3f} m left (median), {np.median(crate_track['up'][crate_seen]):.3f} m up; "
              f"furthest frame {np.linalg.norm(xy - middle, axis=1).max() * 100:.1f} cm from that.")

    walk = None
    if args.run:
        if not on_run_clock:
            raise TrackError("--run needs the video's .json timeline from phone_camera.py record")
        start, end, outcome = load_walk(args.run)
        walk = (start, end)
        report_walk(t, track, seen, start, end, outcome)

    out = args.video.with_name(args.video.stem + "_track.npz")
    meta = {"video": str(args.video), "camera": str(args.camera), "latency_s": args.latency,
            "robot_tag": [args.robot_id, args.robot_size_m], "floor_tag": [args.floor_id, args.floor_size_m],
            "crate_tag": [args.crate_id, args.crate_size_m],
            "floor": floor_info, "clock": "time.perf_counter" if on_run_clock else "video",
            "run": str(args.run) if args.run else None}
    np.savez_compressed(out, meta=json.dumps(meta), t=t, seen=seen, **track, crate_seen=crate_seen,
                        **{f"crate_{key}": values for key, values in crate_track.items()})
    print(f"Wrote {out}")
    if not args.no_plot:
        save_plot(out.with_suffix(".png"), t, track, seen, walk)
    if args.annotate:
        fps = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 15.0
        write_annotated(args.annotate, frames, robot, floor, crate, matrix, dist, floor_rotation, floor_translation,
                        fps)
    if seen.mean() < MIN_ROBOT_DETECTION:
        print(f"FAIL: the robot tag was seen in only {seen.mean():.0%} of frames. Check the light, the focus "
              "and that the tag faces the camera.")
        return 1
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    cal = commands.add_parser("calibrate", help="calibrate the camera from a video of the checkerboard")
    cal.add_argument("video", type=Path)
    cal.add_argument("--square-mm", type=float, default=BOARD_SQUARE_MM,
                     help="measured size of one printed square (default: %(default)s)")
    cal.add_argument("--out", type=Path, default=DEFAULT_CAMERA, help="(default: go2/logs/camera.json)")
    cal.add_argument("--live", metavar="URL", help="first record VIDEO from this stream, as phone_camera.py record does, "
                                                   "with a window showing the board, until q is pressed in it")
    cal.add_argument("--max-width", type=int, help="with --live: shrink wider frames (default: phone_camera.py record's)")
    trk = commands.add_parser("track", help="track the robot tag in a walk video")
    trk.add_argument("video", type=Path)
    trk.add_argument("--camera", type=Path, default=DEFAULT_CAMERA, help="(default: go2/logs/camera.json)")
    trk.add_argument("--run", type=Path, help="run_policy.py log whose walking stage to report")
    trk.add_argument("--latency", type=float, default=0.0,
                     help="seconds the stream lags the robot, taken off the frame times (default: %(default)s)")
    trk.add_argument("--robot-id", type=int, default=ROBOT_TAG_ID)
    trk.add_argument("--floor-id", type=int, default=FLOOR_TAG_ID)
    trk.add_argument("--crate-id", type=int, default=CRATE_TAG_ID)
    trk.add_argument("--robot-size-m", type=float, default=ROBOT_TAG_M,
                     help="measured black square of the robot tag (default: %(default)s)")
    trk.add_argument("--floor-size-m", type=float, default=FLOOR_TAG_M,
                     help="measured black square of the floor tag (default: %(default)s)")
    trk.add_argument("--crate-size-m", type=float, default=CRATE_TAG_M,
                     help="measured black square of the crate tag (default: %(default)s)")
    trk.add_argument("--annotate", type=Path, help="also write a video with the tags outlined, to check by eye")
    trk.add_argument("--no-plot", action="store_true", help="don't save a plot")
    args = parser.parse_args()
    try:
        return calibrate(args) if args.command == "calibrate" else track(args)
    except TrackError as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    status = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # Skips interpreter shutdown, which aborts the process if --live's stream reader is still inside OpenCV.
    os._exit(status)
