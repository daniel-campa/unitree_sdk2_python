"""The pendulum rig's hinge encoder, turned into S-M-ICRL's pole state.

The encoder measures the pole's angle on its hinge, relative to the trunk. With the IMU,
pole_state() turns that into what S-M-ICRL's side-locked inverted_pendulum tasks give the
policy as robot_state[37:41]: the pole's forward angle from straight up, its rate, and the
angle's cosine and sine. PoleReader reads the encoder's microcontroller over USB serial, or
go2/sim_headless.py --pendulum over UDP. Nothing here touches DDS.

The microcontroller sends one text line per reading:

    <its clock in microseconds> <hinge angle in radians>

The angle may have any zero, as an incremental encoder's is wherever it was at power-up:
HangingWindow takes the zero from the pole hanging still, each time. Its direction is the
mount file's, from go2/check_pole.py --calibrate. go2/pole_encoder/pole_encoder.ino sends
these lines from a quadrature encoder on an ESP32-S3.
"""
import json
import math
import os
import socket
import termios
import time
import tty
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

MOUNT_FILE = Path(__file__).resolve().parent / "logs" / "pole_mount.json"
SIM_PORT = 9870  # go2/sim_headless.py --pendulum's encoder
DEFAULT_BAUD = 500000  # 500 lines of ~20 bytes a second need more than 115200; native USB ignores it
# The rate is the slope over this much of the encoder's own clock. Longer is smoother but later.
RATE_WINDOW_S = 0.02
MAX_POLE_AGE = 0.1  # s; an older reading stops a walk
KEEPALIVE_S = 0.5  # the simulator stops streaming to a client it hasn't heard from for 2 s
HISTORY = 64  # readings kept for the rate
# The zero is the mean over this long, which covers a whole swing of a pole swaying slightly.
ZERO_WINDOW_S = 1.5
HANGING_STILL_DEG = 3.0  # the most the pole may stray from its mean while the zero is taken


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class PoleMount:
    """The encoder's direction. hinge(), the pole's angle from the trunk's up axis, is + with the tip toward the head."""
    sign: float = 1.0
    measured: str = "none: the simulator's encoder turns with the hinge"

    def hinge(self, raw, zero):
        return wrap(self.sign * raw + zero)

    def as_dict(self):
        return {"sign": self.sign, "measured": self.measured}


def load_mount(path):
    data = json.loads(Path(path).read_text())
    sign = float(data["sign"])
    if sign not in (1.0, -1.0):
        raise ValueError(f"sign must be 1 or -1, not {sign:g}")
    return PoleMount(sign=sign, measured=str(data.get("measured", path)))


def save_mount(path, mount, **extra):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({**mount.as_dict(), **extra}, indent=2) + "\n")


def body_rotation(quaternion):
    """The body-to-world rotation matrix of a (w, x, y, z) IMU quaternion."""
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def pole_state(hinge, hinge_rate, quaternion, gyro):
    """(theta, rate, cos theta, sin theta, up): S-M-ICRL's swingup pole state, and the pole's world up component.

    As S-M-ICRL's _link_pitch_state: theta is the forward angle from straight up in the
    heading frame (level, turned with the yaw), 0 upright and +-pi hanging; rate is the
    pole's world angular velocity about the heading frame's left axis. up is what the
    swing-up reward reads: the angle from straight up is acos(up).
    """
    rotation = body_rotation(quaternion)
    u = rotation @ np.array([math.sin(hinge), 0.0, math.cos(hinge)])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    theta = math.atan2(u[0] * c + u[1] * s, u[2])
    w = rotation @ (np.asarray(gyro, dtype=float) + np.array([0.0, hinge_rate, 0.0]))
    return theta, -w[0] * s + w[1] * c, math.cos(theta), math.sin(theta), float(np.clip(u[2], -1.0, 1.0))


def hanging_hinge(quaternion):
    """The hinge angle at which the pole hangs straight down, with the trunk at this orientation.

    quaternion may also be (N, 4), for N angles.
    """
    w, x, y, z = np.moveaxis(np.asarray(quaternion, dtype=float), -1, 0)
    # Straight down in the body frame is minus the rotation matrix's bottom row.
    return np.arctan2(-2.0 * (x * z - w * y), -(1.0 - 2.0 * (x * x + y * y)))


class HangingWindow:
    """Encoder readings and body orientations over the last ZERO_WINDOW_S, to zero the encoder on a pole hanging still."""

    def __init__(self, seconds=ZERO_WINDOW_S):
        self.seconds = seconds
        self._rows = deque()

    def add(self, t, raw, quaternion):
        self._rows.append((t, raw, *quaternion))
        while self._rows[0][0] < t - self.seconds:
            self._rows.popleft()

    def clear(self):
        self._rows.clear()

    def zero(self, sign):
        """(zero, spread), or None until it spans nearly ZERO_WINDOW_S.

        zero makes the mean reading hang straight down, and spread is the most a reading
        strayed from it (rad). A pole that swung, or a trunk that moved under a pole that
        didn't, shows as spread.
        """
        if not self._rows or self._rows[-1][0] - self._rows[0][0] < 0.9 * self.seconds:
            return None
        rows = np.array(self._rows)
        offsets = hanging_hinge(rows[:, 2:6]) - sign * rows[:, 1]
        zero = math.atan2(np.sin(offsets).mean(), np.cos(offsets).mean())
        spread = float(np.abs((offsets - zero + math.pi) % (2.0 * math.pi) - math.pi).max())
        return zero, spread


class PoleReader:
    """The newest encoder readings, read by poll() without blocking.

    source is a serial device, optionally with its baud rate (/dev/ttyACM0@115200), or
    udp:PORT for go2/sim_headless.py --pendulum.
    """

    def __init__(self, source):
        self.source = source
        self.simulated = source.startswith("udp:")
        self._buffer = b""
        self._recent = deque(maxlen=HISTORY)  # (device seconds, unwrapped raw angle)
        self._last_us = None
        self._wraps = 0
        self._unwrapped = None
        self.newest_raw = None
        self.newest_arrival = None
        self.readings = []  # (arrival, device seconds, raw) for the log
        self.bad_lines = 0
        self.restarts = 0
        self._sock = self._fd = None
        if self.simulated:
            self._address = ("127.0.0.1", int(source[4:]))
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(("127.0.0.1", 0))
            self._sock.setblocking(False)
            self._last_sent = -math.inf
            self._send("hello")
        else:
            device, _, baud = source.partition("@")
            self._fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                tty.setraw(self._fd)
                attrs = termios.tcgetattr(self._fd)
                speed = getattr(termios, f"B{int(baud or DEFAULT_BAUD)}")
                attrs[2] |= termios.CLOCAL | termios.CREAD
                attrs[4] = attrs[5] = speed
                termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
                termios.tcflush(self._fd, termios.TCIFLUSH)
            except termios.error as error:
                os.close(self._fd)
                raise OSError(f"{device} isn't a serial port: {error}") from error
            except (AttributeError, ValueError) as error:
                os.close(self._fd)
                raise ValueError(f"baud rate {baud!r} isn't one this computer has") from error

    def _send(self, text):
        try:
            self._sock.sendto(text.encode(), self._address)
        except OSError:
            pass  # the simulator isn't up yet; the next keepalive tries again
        self._last_sent = time.perf_counter()

    def steady(self):
        """In the simulator, damp the hinge as an operator's hand steadies the pole. Nothing on the robot."""
        if self.simulated:
            self._send("steady")

    def release(self):
        """In the simulator, let the pole go: the hinge turns freely again. Nothing on the robot."""
        if self.simulated:
            self._send("release")

    def flush(self):
        """Drop what queued up unread, so the next reading is a fresh one."""
        if self.simulated:
            for _ in self._chunks():
                pass
        else:
            termios.tcflush(self._fd, termios.TCIFLUSH)
        self._buffer = b""

    def _chunks(self):
        while True:
            try:
                chunk = self._sock.recv(65536) if self.simulated else os.read(self._fd, 4096)
            except (BlockingIOError, InterruptedError):
                return
            if not chunk:
                return
            yield chunk

    def poll(self, now=None):
        """Take every reading that has arrived. Returns how many."""
        now = time.perf_counter() if now is None else now
        if self.simulated and now - self._last_sent > KEEPALIVE_S:
            self._send("hello")
        count = 0
        for chunk in self._chunks():
            self._buffer += chunk
            *lines, self._buffer = self._buffer.split(b"\n")
            for line in lines:
                count += self._take(line, now)
        if len(self._buffer) > 256:  # no newline in that much: not this protocol
            self._buffer = b""
            self.bad_lines += 1
        return count

    def _take(self, line, now):
        try:
            us_text, angle_text = line.split()
            us, raw = int(us_text), float(angle_text)
        except ValueError:
            self.bad_lines += line.strip() != b""
            return 0
        if not math.isfinite(raw) or us < 0:
            self.bad_lines += 1
            return 0
        if self._last_us is not None and us < self._last_us:
            if self._last_us - us > 2 ** 31:
                self._wraps += 1  # micros() wraps after 71.6 min
            else:
                self._recent.clear()  # the microcontroller restarted
                self._unwrapped = None
                self._wraps = 0
                self.restarts += 1
        self._last_us = us
        device_t = (us + self._wraps * 2 ** 32) * 1e-6
        self._unwrapped = raw if self._unwrapped is None else self._unwrapped + wrap(raw - self.newest_raw)
        self._recent.append((device_t, self._unwrapped))
        self.newest_raw, self.newest_arrival = raw, now
        self.readings.append((now, device_t, raw))
        return 1

    def latest(self, now=None):
        """(raw angle, raw rate in rad/s, age in s) of the newest reading, or None before there is one."""
        if self.newest_arrival is None:
            return None
        now = time.perf_counter() if now is None else now
        newest_t = self._recent[-1][0]
        window = [(t, a) for t, a in self._recent if t >= newest_t - RATE_WINDOW_S]
        rate = 0.0
        if len(window) >= 2:
            t, a = np.array(window).T
            t = t - t.mean()
            rate = float((t * (a - a.mean())).sum() / max((t * t).sum(), 1e-12))
        return self.newest_raw, rate, now - self.newest_arrival

    def wait(self, timeout):
        """Poll until a reading arrives or timeout passes. True if one did."""
        deadline = time.perf_counter() + timeout
        while self.newest_arrival is None and time.perf_counter() < deadline:
            self.poll()
            time.sleep(0.005)
        return self.newest_arrival is not None

    def rate_hz(self, since):
        """Readings per second that arrived from since (time.perf_counter()) on."""
        times = [r[0] for r in self.readings if r[0] >= since]
        if len(times) < 2:
            return 0.0
        return (len(times) - 1) / max(times[-1] - times[0], 1e-9)

    def worst_gap(self, since):
        """The longest time between readings that arrived from since on, in s."""
        times = np.array([r[0] for r in self.readings if r[0] >= since])
        return float(np.diff(times).max()) if len(times) >= 2 else math.inf

    def describe(self):
        return f"{self.source} ({'go2/sim_headless.py --pendulum' if self.simulated else 'serial'})"

    def close(self):
        if self._sock is not None:
            self._sock.close()
        if self._fd is not None:
            os.close(self._fd)


def wait_still(reader, quaternion, sign, timeout):
    """Watch the pole until it has hung still for ZERO_WINDOW_S: (zero, spread), or None after timeout.

    quaternion() is the body's orientation now, from the IMU.
    """
    window = HangingWindow()
    deadline = time.perf_counter() + timeout
    next_check = 0.0
    while time.perf_counter() < deadline:
        reader.poll()
        reading = reader.latest()
        now = time.perf_counter()
        if reading is not None and reading[2] <= MAX_POLE_AGE:
            window.add(now, reading[0], quaternion())
        if now >= next_check:
            next_check = now + 0.1
            found = window.zero(sign)
            if found is not None and math.degrees(found[1]) <= HANGING_STILL_DEG:
                return found
        time.sleep(0.005)
    return None


def default_mount(source, path=None):
    """The mount for source: path's, or the simulator's identity, or MOUNT_FILE's for a real encoder."""
    if path is not None:
        return load_mount(path), Path(path)
    if source.startswith("udp:"):
        return PoleMount(), None
    return load_mount(MOUNT_FILE), MOUNT_FILE


def mount_stamp():
    return f"go2/check_pole.py --calibrate, {datetime.now():%Y-%m-%d %H:%M:%S}"
