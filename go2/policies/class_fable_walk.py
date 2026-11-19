# Trot for the Go2 that steps on a count of policy calls, not on what the legs report.
#
# Joint order: FR(0,1,2), FL(3,4,5), RR(6,7,8), RL(9,10,11); per leg hip, thigh, calf.
# Diagonal A is FR + RL, diagonal B is FL + RR.
# Run it at 10 policy calls a second (--policy-hz 10): each pose is held for one call.
#
# Why it is built this way (from the robot runs of 2026-11-19):
#   - The state a policy was given on the robot was at times 0.3-0.7 s old. Here the gait's
#     timing comes only from counting calls, so old joint readings can't stall or scramble it.
#     The one thing read from the state is the heading (robot_state[26]), for slow steering.
#   - Loaded knees on the robot sag and never reach a straight target, so nothing waits for a
#     leg to arrive anywhere, and no pose depends on a loaded knee straightening.
#   - The feet are set further back under the body than in the home pose (thigh 0.96, not
#     0.90, with a straighter knee). That moves weight off the rear legs, which were the ones
#     that could not lift: in simulation with weakened joints the rear feet cleared 2-3 cm
#     with the feet under the hips and 6-8 cm with them here.
#   - It stands taller than the home pose (knee -1.50, not -1.80) and tucks the swing knee
#     0.80 rad, because the rear-right foot dragged when the walk was crouched.

STAND_THIGH = 0.96
STAND_CALF = -1.50
TUCK = 0.8
STRIDE = 0.16          # rad of thigh travel each side of the stand pose
REAR_SHIFT = 0.00      # rad added to the rear thighs, which moves weight onto the front legs
RISE_CALLS = 5         # calls spent moving to the tall stand, all feet down, before the first step
STEER = 0.40           # share of the stride taken off the inside legs when steering
YAW_BAND = 0.08        # rad of heading error before it steers

# One leg's cycle, one entry per call. THIGH is in units of STRIDE (negative = forward),
# LIFT is the share of TUCK. The other diagonal runs the same cycle half a cycle later.
THIGH = (-0.35, -1.0, 0.35, 1.0)
LIFT = (1.0, 0.0, 0.0, 0.0)
N = len(THIGH)

STEERS = ((1.0, 1.0), (1.0, 1.0 - STEER), (1.0 - STEER, 1.0))   # (right, left) stride: straight, turn left, turn right


def make_matrix(pose):
    K = np.zeros((12, 6), dtype=float)
    K[:, 5] = pose
    return K


def pose(k, stride):
    """The pose for call k of the cycle; k < 0 is the tall stand with all feet down."""
    out = []
    for leg in ("FR", "FL", "RR", "RL"):
        side = stride[0] if leg in ("FR", "RR") else stride[1]
        base = STAND_THIGH + (REAR_SHIFT if leg in ("RR", "RL") else 0.0)
        if k < 0:
            out += [0.0, base, STAND_CALF]
            continue
        p = k if leg in ("FR", "RL") else (k + N // 2) % N
        out += [0.0, base + THIGH[p] * STRIDE * side, STAND_CALF - LIFT[p] * TUCK]
    return out


matrices = [make_matrix(pose(-1, (1.0, 1.0)))]
matrices += [make_matrix(pose(k, stride)) for stride in STEERS for k in range(N)]

calls = [0]


def selector(robot_state):
    n = calls[0]
    calls[0] = n + 1
    if n < RISE_CALLS:
        return 0
    yaw = robot_state[26]
    steer = 1 if yaw < -YAW_BAND else (2 if yaw > YAW_BAND else 0)   # drifted right: turn left
    return 1 + steer * N + (n - RISE_CALLS) % N


robot.simulate_classifier(matrices, selector)
