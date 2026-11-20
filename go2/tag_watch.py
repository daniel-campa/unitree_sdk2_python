#!/usr/bin/env python
"""Print which AprilTags the phone camera sees, once a second. Never touches the robot.

    python go2/tag_watch.py SOURCE      # same SOURCE as phone_camera.py: stream URL or camera number
"""
import sys
import time

import cv2

from phone_camera import Stream
from tag_tracker import FLOOR_TAG_ID, ROBOT_TAG_ID, make_detector

NAMES = {ROBOT_TAG_ID: "robot", FLOOR_TAG_ID: "floor"}

stream = Stream(sys.argv[1])
stream.wait_first()
detector = make_detector()
try:
    while True:
        frame, received_at = stream.latest()
        _, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        seen = set() if ids is None else {int(i) for i in ids.ravel()}
        status = "  ".join(f"{name}:{'YES' if i in seen else '---'}" for i, name in NAMES.items())
        others = sorted(seen - NAMES.keys())
        age = time.perf_counter() - received_at
        print(f"{time.strftime('%H:%M:%S')}  {status}  other:{others or '-'}  frame age {age:.1f}s", flush=True)
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    stream.close()
