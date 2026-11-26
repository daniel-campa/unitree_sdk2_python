"""go2/sim_headless.py --pendulum: S-M-ICRL's side-locked pendulum rigs on Unitree's Go2, and their encoder.

The rigs copy S-M-ICRL's Robots/go2/go2_inverted_pendulum_sidelocked.xml (top) and
go2_inverted_pendulum_sidelocked_sidearm.xml (sidearm): keep them the same. The hinge is
frictionless there, so here too, except while a client asks for the pole to be steadied.

SimEncoder streams the hinge angle over UDP in go2/pole_encoder.py's line protocol, as the
rig's microcontroller does over serial, to every client that has sent a datagram in the
last 2 s. Like the rig's incremental encoder, it counts from where the pole was when it
started. A client's datagram can also say `steady` or `release`.
"""
import math
import socket
import time

import mujoco

POLE_LENGTH = 0.40  # m, hinge to the tip's centre
RIGS = {
    # (name, fromto, mass) of each fixed support on the trunk, the hinge's position, and its axle
    "top": {
        "supports": [("pendulum_mast", [0, 0, 0.057, 0, 0, 0.56], 0.20)],
        "hinge": [0, 0.04, 0.56],
        "axle": [0, -0.045, 0, 0, 0.01, 0],
    },
    "sidearm": {
        "supports": [("pendulum_post", [0, 0, 0.057, 0, 0, 0.25], 0.10),
                     ("pendulum_arm", [0, 0, 0.25, 0, 0.185, 0.25], 0.10)],
        "hinge": [0, 0.20, 0.25],
        "axle": [0, -0.015, 0, 0, 0.01, 0],
    },
}
JOINT = "pole_pitch"
POLE_BODY = "pendulum_pole"
# About critical for the hanging pole (gravity stiffness 2.2 N·m/rad, inertia 0.085 kg·m²).
STEADY_DAMPING = 1.0  # N·m·s/rad
ENCODER_HZ = 500.0
CLIENT_TIMEOUT_S = 2.0


def add_rig(spec, rig):
    """Put the rig on the trunk, with the pole hanging straight down in the "home" keyframe."""
    geometry = RIGS[rig]
    base = spec.body("base_link")
    hidden = {"contype": 0, "conaffinity": 0, "group": 1}
    for name, fromto, mass in geometry["supports"]:
        base.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=fromto, size=[0.012, 0, 0], mass=mass,
                      rgba=[0.55, 0.55, 0.60, 1], **hidden)
    hinge = base.add_body(name="pendulum_base", pos=geometry["hinge"])
    # The trunk's childclass would give the hinge the legs' friction, which stalls the swing.
    hinge.add_joint(name=JOINT, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0], damping=0, armature=0,
                    frictionloss=0)
    hinge.add_geom(name="pendulum_axle", type=mujoco.mjtGeom.mjGEOM_CYLINDER, fromto=geometry["axle"],
                   size=[0.012, 0, 0], mass=0.01, rgba=[0.30, 0.30, 0.30, 1], **hidden)
    pole = hinge.add_body(name=POLE_BODY)
    pole.add_geom(name="pole_rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0, 0, 0, POLE_LENGTH],
                  size=[0.008, 0, 0], mass=0.10, rgba=[0.85, 0.85, 0.20, 1], **hidden)
    pole.add_geom(name="pole_tip", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0, 0, POLE_LENGTH], size=[0.035, 0, 0],
                  mass=0.50, rgba=[0.90, 0.20, 0.20, 1], **hidden)
    for key in spec.keys:
        key.qpos = list(key.qpos) + [math.pi]  # the new joint's qpos comes last


def pole_up_deg(model, data):
    """The pole's angle from straight up, in degrees: 0 up, 180 hanging."""
    z = data.xmat[model.body(POLE_BODY).id][8]
    return math.degrees(math.acos(max(-1.0, min(1.0, z))))


class SimEncoder:
    """The rig's encoder: the hinge's turn since start-up in [0, 2 pi), in counts per turn, ENCODER_HZ times a simulated second."""

    def __init__(self, model, data, port, counts):
        self.model, self.data = model, data
        joint = model.joint(JOINT)
        self.qpos, self.dof = joint.qposadr[0], joint.dofadr[0]
        self.start = float(data.qpos[self.qpos])
        self.counts = counts
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.clients = {}
        self.steady = True  # until a client releases it, as the operator holds the pole while the robot stands up
        self.next_send = 0.0
        self._apply()

    def _apply(self):
        self.model.dof_damping[self.dof] = STEADY_DAMPING if self.steady else 0.0

    def _listen(self, now):
        while True:
            try:
                message, address = self.sock.recvfrom(256)
            except (BlockingIOError, InterruptedError):
                break
            self.clients[address] = now
            word = message.strip().decode(errors="replace")
            if word in ("steady", "release") and self.steady != (word == "steady"):
                self.steady = word == "steady"
                self._apply()
                print(f"pole: {'steadied' if self.steady else 'released'} at t={self.data.time:.2f}", flush=True)
        for address in [a for a, heard in self.clients.items() if now - heard > CLIENT_TIMEOUT_S]:
            del self.clients[address]

    def step(self):
        """Call after every mj_step."""
        if self.data.time < self.next_send:
            return
        self.next_send += 1.0 / ENCODER_HZ
        if self.next_send < self.data.time:  # after a reset, or a dt longer than the encoder's period
            self.next_send = self.data.time + 1.0 / ENCODER_HZ
        self._listen(time.perf_counter())
        angle = (self.data.qpos[self.qpos] - self.start) % (2.0 * math.pi)
        if self.counts:
            angle = round(angle * self.counts / (2.0 * math.pi)) % self.counts * 2.0 * math.pi / self.counts
        line = f"{int(self.data.time * 1e6)} {angle:.6f}\n".encode()
        for address in list(self.clients):
            try:
                self.sock.sendto(line, address)
            except OSError:
                del self.clients[address]
