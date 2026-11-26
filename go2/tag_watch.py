#!/usr/bin/env python
"""Show which AprilTags the phone camera sees: a live window with the tags outlined, and a line printed once a
second. Never touches the robot.

    python go2/tag_watch.py SOURCE              # same SOURCE as phone_camera.py: stream URL or camera number
    python go2/tag_watch.py SOURCE --no-window  # print only (also the default with no screen)

q or Esc in the window, closing it, or Ctrl-C stops.
"""
import argparse
import os
import time

import cv2
import numpy as np

from phone_camera import Stream
from tag_tracker import CRATE_TAG_ID, FLOOR_TAG_ID, ROBOT_TAG_ID, make_detector

NAMES = {ROBOT_TAG_ID: "robot", FLOOR_TAG_ID: "floor", CRATE_TAG_ID: "crate"}
COLOURS = {ROBOT_TAG_ID: (0, 0, 255), FLOOR_TAG_ID: (0, 255, 0), CRATE_TAG_ID: (255, 0, 0)}  # as tag_tracker.py --annotate
OTHER_COLOUR = (0, 220, 255)
WINDOW = "tag_watch.py (q stops)"
SHOW_WIDTH = 1280


def status_line(seen, age):
    status = "  ".join(f"{name}:{'YES' if i in seen else '---'}" for i, name in NAMES.items())
    others = sorted(seen.keys() - NAMES.keys())
    return f"{status}  other:{others or '-'}  frame age {age:.1f}s"


def view(frame, seen, line):
    scale = min(1.0, SHOW_WIDTH / frame.shape[1])
    picture = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    for tag_id, corners in seen.items():
        points = (corners * scale).astype(np.int32)
        colour = COLOURS.get(tag_id, OTHER_COLOUR)
        cv2.polylines(picture, [points], True, colour, 3)
        cv2.putText(picture, NAMES.get(tag_id, str(tag_id)), tuple(points.min(axis=0) - [0, 8]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2, cv2.LINE_AA)
    bar = np.full((40, picture.shape[1], 3), 40, np.uint8)
    cv2.putText(bar, line, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([picture, bar])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source")
    parser.add_argument("--no-window", action="store_true", help="only print")
    args = parser.parse_args()
    window = not args.no_window and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not window and not args.no_window:
        print("No screen, so no window; printing only.", flush=True)

    stream = Stream(args.source)
    detector = make_detector()
    last_frame_at = last_print = None
    created = opened = False
    try:
        stream.wait_first()
        while True:
            frame, received_at = stream.latest()
            if received_at == last_frame_at:
                if not window:
                    time.sleep(0.02)
                elif (cv2.waitKey(10) & 0xFF) in (ord("q"), ord("Q"), 27):
                    break
                continue
            last_frame_at = received_at
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            seen = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}
            now = time.perf_counter()
            line = status_line(seen, now - received_at)
            if last_print is None or now - last_print >= 1:
                print(f"{time.strftime('%H:%M:%S')}  {line}", flush=True)
                last_print = now
            if not window:
                continue
            picture = view(frame, seen, line)
            if not created:
                created = True
                cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(WINDOW, picture.shape[1], picture.shape[0])
            cv2.imshow(WINDOW, picture)
            key = cv2.waitKey(1) & 0xFF
            visible = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
            # Closed only once it has been seen open, in case this backend never reports it visible.
            if key in (ord("q"), ord("Q"), 27) or (opened and not visible):
                break
            opened = opened or visible
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()
        if window:
            cv2.destroyAllWindows()
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
