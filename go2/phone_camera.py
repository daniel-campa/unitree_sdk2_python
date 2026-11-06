#!/usr/bin/env python
"""Film the Go2 with a phone, for the LLM in S-M-ICRL's real-robot loop.

The phone streams video with an IP-camera app, and this script reads the stream
with OpenCV. It never talks to the robot.

    python go2/phone_camera.py check URL            # is the stream usable? saves camera_check.jpg
    python go2/phone_camera.py record URL OUT.mp4   # record until Ctrl-C or SIGTERM

URL is the stream address the app shows, for example http://192.168.1.23:8080/video.
A number such as 0 is a camera attached to this computer (/dev/video0), such as an
Android phone in USB webcam mode.

record writes OUT.mp4 at a steady --fps, each frame being the newest one received
at that moment, and OUT.json with the time each of those frames arrived. The times
are on time.perf_counter(), the clock of run_policy.py's log, so S-M-ICRL can pick
out the frames from the walk. They are arrival times, so they lag the robot by the
stream's delay, usually a fraction of a second. Nothing corrects for that.
"""
import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

import cv2

OPEN_TIMEOUT_S = 10.0
READ_TIMEOUT_S = 2.0  # a stalled stream is reopened after this
RETRY_S = 1.0
MIN_RATE = 10.0  # frames per second; S-M-ICRL picks 30 pictures from a 5 s walk
MAX_GAP_S = 0.5


class CameraError(Exception):
    pass


def open_capture(source):
    if source.isdigit():
        capture = cv2.VideoCapture(int(source))
    elif source.startswith("/dev/"):
        capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    else:
        capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG, [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, int(OPEN_TIMEOUT_S * 1000),
            cv2.CAP_PROP_READ_TIMEOUT_MSEC, int(READ_TIMEOUT_S * 1000),
        ])
    if not capture.isOpened():
        capture.release()
        return None
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


class Stream:
    """Read a stream in the background and keep its newest frame. Reopens it when it drops."""

    def __init__(self, source):
        self.source = source
        self._lock = threading.Lock()
        self._frame = None
        self.received_at = None  # time.perf_counter() of the newest frame
        self.count = 0
        self.reconnects = 0
        self._longest_gap = 0.0
        self._closing = threading.Event()
        capture = open_capture(source)
        if capture is None:
            raise CameraError(f"can't open {source}. Check the address, and that the app is streaming.")
        self._reader = threading.Thread(target=self._read, args=(capture,), daemon=True)
        self._reader.start()

    def _read(self, capture):
        while not self._closing.is_set():
            ok, frame = capture.read() if capture is not None else (False, None)
            if ok:
                now = time.perf_counter()
                with self._lock:
                    if self.received_at is not None:
                        self._longest_gap = max(self._longest_gap, now - self.received_at)
                    self._frame, self.received_at = frame, now
                    self.count += 1
                continue
            if capture is not None:
                capture.release()
            if self._closing.wait(RETRY_S):
                return
            capture = open_capture(self.source)
            self.reconnects += 1
        if capture is not None:
            capture.release()

    def close(self):
        """Stop reading. Returns False if the reader is still stuck in OpenCV."""
        self._closing.set()
        self._reader.join(READ_TIMEOUT_S + OPEN_TIMEOUT_S)
        return not self._reader.is_alive()

    def wait_first(self, stop=None):
        deadline = time.perf_counter() + OPEN_TIMEOUT_S
        while self.received_at is None:
            if time.perf_counter() > deadline:
                raise CameraError(f"{self.source} opened but sent no frames in {OPEN_TIMEOUT_S:g} s")
            if stop is None:
                time.sleep(0.05)
            elif stop.wait(0.05):
                raise CameraError("stopped before the first frame")

    def latest(self):
        with self._lock:
            return self._frame, self.received_at

    def longest_gap(self):
        """The longest wait between frames so far, counting the current one."""
        with self._lock:
            if self.received_at is None:
                return 0.0
            return max(self._longest_gap, time.perf_counter() - self.received_at)

    def reset_gap(self):
        with self._lock:
            self._longest_gap = 0.0


def fitted(frame, size):
    return frame if (frame.shape[1], frame.shape[0]) == size else cv2.resize(frame, size)


def output_size(frame, max_width):
    height, width = frame.shape[:2]
    if width > max_width:
        width, height = max_width, round(height * max_width / width)
    return width - width % 2, height - height % 2  # even sizes for the encoder


def check(args):
    print(f"Opening {args.source} ...", flush=True)
    stream = Stream(args.source)
    stream.wait_first()
    stream.reset_gap()
    first, start = stream.count, time.perf_counter()
    time.sleep(args.seconds)
    elapsed = time.perf_counter() - start
    rate, gap = (stream.count - first) / elapsed, stream.longest_gap()
    frame, _ = stream.latest()
    stream.close()
    print(f"Stream: {frame.shape[1]}x{frame.shape[0]}, {rate:.1f} frames/s over {elapsed:.0f} s, "
          f"longest gap {gap * 1000:.0f} ms, {stream.reconnects} reconnects")
    if args.snapshot:
        cv2.imwrite(args.snapshot, frame)
        print(f"Snapshot: {args.snapshot}. Check that it shows the whole lane.")
    problems = []
    if rate < MIN_RATE:
        problems.append(f"{rate:.1f} frames/s is below {MIN_RATE:g}")
    if gap > MAX_GAP_S:
        problems.append(f"a {gap:.1f} s gap between frames is longer than {MAX_GAP_S:g} s")
    if problems:
        print("FAIL: " + "; ".join(problems))
        return 1
    print("PASS")
    return 0


def record(args):
    os.nice(10)  # leave the CPU to run_policy.py first
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda signum, frame: stop.set())
    parent = os.getppid()

    stream = Stream(args.source)
    stream.wait_first(stop)
    frame, _ = stream.latest()
    size = output_size(frame, args.max_width)
    args.video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.video), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, size)
    if not writer.isOpened():
        raise CameraError(f"can't write {args.video}")
    print(f"Recording {args.source} to {args.video}, {size[0]}x{size[1]} at {args.fps:g} fps", flush=True)

    received_at = []
    start = time.perf_counter()
    try:
        while not stop.is_set():
            if os.getppid() != parent:
                print("The program that started this recording has gone, so it stops.", flush=True)
                break
            if time.perf_counter() - start > args.max_seconds:
                print(f"Stopping after --max-seconds {args.max_seconds:g}.", flush=True)
                break
            delay = start + len(received_at) / args.fps - time.perf_counter()
            if delay > 0:
                stop.wait(delay)
                continue
            frame, stamp = stream.latest()
            writer.write(fitted(frame, size))
            received_at.append(stamp)
    finally:
        writer.release()
        stream.close()
        timeline = {
            "source": args.source, "fps": args.fps, "size": list(size), "clock": "time.perf_counter",
            "received_at": received_at, "frames_received": stream.count,
            "longest_gap_s": stream.longest_gap(), "reconnects": stream.reconnects,
        }
        args.video.with_suffix(".json").write_text(json.dumps(timeline))
    seconds = len(received_at) / args.fps
    print(f"Recorded {seconds:.1f} s. The stream sent {stream.count / max(seconds, 1e-9):.1f} frames/s, "
          f"longest gap {timeline['longest_gap_s']:.2f} s, {stream.reconnects} reconnects.")
    return 0


def positive(text):
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be above 0")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    check_parser = commands.add_parser("check", help="test the stream and save one frame")
    check_parser.add_argument("source", help="the stream's URL, or the number of a camera on this computer")
    check_parser.add_argument("--seconds", type=positive, default=5.0, help="how long to test (default: %(default)s)")
    check_parser.add_argument("--snapshot", default="camera_check.jpg",
                              help="where to save a frame, '' for nowhere (default: %(default)s)")
    record_parser = commands.add_parser("record", help="record until Ctrl-C or SIGTERM")
    record_parser.add_argument("source", help="the stream's URL, or the number of a camera on this computer")
    record_parser.add_argument("video", type=Path, help="the .mp4 to write; the frame times go next to it as .json")
    record_parser.add_argument("--fps", type=positive, default=15.0, help="frames written per second (default: %(default)s)")
    record_parser.add_argument("--max-width", type=int, default=960, help="shrink wider frames (default: %(default)s)")
    record_parser.add_argument("--max-seconds", type=positive, default=900.0,
                               help="stop by itself after this long (default: %(default)s)")
    args = parser.parse_args()
    try:
        return check(args) if args.command == "check" else record(args)
    except CameraError as error:
        print(f"FAIL: {error}", flush=True)
        return 1


if __name__ == "__main__":
    status = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # Skips interpreter shutdown, which aborts the process if a reader thread is
    # still inside OpenCV. Everything has been written by now.
    os._exit(status)
