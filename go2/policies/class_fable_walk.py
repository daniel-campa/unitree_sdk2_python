# Trot for the Go2, written for the real robot on carpet.
#
# Joint order: FR(0,1,2), FL(3,4,5), RR(6,7,8), RL(9,10,11); per leg hip, thigh, calf.
# Diagonal A is FR + RL, diagonal B is FL + RR.
#
# What it does differently from class_argon_walk, and why:
#   - Stands tall. The stance knee is commanded to -1.50 rather than -1.80, because the
#     robot's knees sag 0.1-0.2 rad under load and argon ended up walking in a crouch.
#   - Lifts high. The swing knee tucks 0.70 rad from the stance pose, so a foot clears the
#     floor even when its leg has sagged. On the robot the rear-right foot dragged.
#   - Takes shorter steps, so the jumps between poses stay inside the torque cap.
#   - Picks the next pose from where the legs are now (nearest pose, then the one after it),
#     so it needs no joint velocities and has no thresholds a sagging leg can fail to reach.
#   - Levels the body. When the IMU shows one side or the rear riding low, the legs there
#     are commanded straighter, which is what a sagging leg needs to carry its share.
#   - Steers. When the heading drifts, the legs on the inside of the correction take a
#     shorter stride.
#
# Every combination of (step, steer, roll trim, pitch trim) is its own matrix; the selector
# only picks one. It reads joint angles (1, 2, 4, 5, 7, 8, 10, 11), roll, pitch and yaw (24-26).

STAND_THIGH = 0.78
STAND_CALF = -1.50
TUCK = 0.70          # how far the swing knee folds
STRIDE = 0.16        # thigh travel each side of the stand pose at full stride
HALF = 0.35          # share of the stride used in the lift pose
STEER = 0.45         # the inside legs' stride is cut by this share when steering
YAW_BAND = 0.06      # rad of heading error before it steers
EXTEND = 0.20        # rad a low side's knees are straightened by
ROLL_BAND = 0.04     # rad of roll before a side counts as low
PITCH_BAND = 0.05    # rad of nose-up pitch before the rear counts as low

STEERS = ((1.0, 1.0), (1.0, 1.0 - STEER), (1.0 - STEER, 1.0))   # (right, left) stride: straight, turn left, turn right
ROLLS = ((0.0, 0.0), (EXTEND, 0.0), (0.0, EXTEND))               # (right, left) extension: level, right low, left low
PITCHES = (0.0, EXTEND)                                          # rear extension: level, rear low


def make_matrix(pose):
    K = np.zeros((12, 6), dtype=float)
    K[:, 5] = pose
    return K


def pose(step, stride, side_extend, rear_extend):
    """One pose of the cycle.

    step 0: A lifted and swinging forward, B on the ground pushing back
    step 1: A planted ahead, B pushed fully back
    step 2: B lifted and swinging forward, A on the ground pushing back
    step 3: B planted ahead, A pushed fully back
    """
    lifted = step in (0, 2)
    a_swings = step in (0, 1)
    reach = STRIDE * (HALF if lifted else 1.0)
    out = []
    for leg in ("FR", "FL", "RR", "RL"):
        right = leg in ("FR", "RR")
        swinging = (leg in ("FR", "RL")) == a_swings
        extend = side_extend[0 if right else 1] + (rear_extend if leg in ("RR", "RL") else 0.0)
        thigh = STAND_THIGH - 0.5 * extend + (-reach if swinging else reach) * stride[0 if right else 1]
        calf = STAND_CALF + extend - (TUCK if (swinging and lifted) else 0.0)
        out += [0.0, thigh, calf]
    return out


matrices = [make_matrix(pose(step, stride, side, rear))
            for rear in PITCHES for side in ROLLS for stride in STEERS for step in range(4)]

# Where each step sits in the plane of (thigh difference, calf difference) between the diagonals.
POINTS = [(-2.0 * STRIDE * HALF, -TUCK), (-2.0 * STRIDE, 0.0), (2.0 * STRIDE * HALF, TUCK), (2.0 * STRIDE, 0.0)]


def selector(robot_state):
    d_thigh = 0.5 * ((robot_state[1] + robot_state[10]) - (robot_state[4] + robot_state[7]))
    d_calf = 0.5 * ((robot_state[2] + robot_state[11]) - (robot_state[5] + robot_state[8]))

    # The step the legs are nearest to now. The next one in the cycle is the target.
    best, nearest = None, 0
    for i, (pt, pc) in enumerate(POINTS):
        dist = ((d_thigh - pt) / (2.0 * STRIDE)) ** 2 + ((d_calf - pc) / TUCK) ** 2
        if best is None or dist < best:
            best, nearest = dist, i
    step = (nearest + 1) % 4

    roll, pitch, yaw = robot_state[24], robot_state[25], robot_state[26]
    steer = 1 if yaw < -YAW_BAND else (2 if yaw > YAW_BAND else 0)        # drifted right: turn left
    side = 1 if roll > ROLL_BAND else (2 if roll < -ROLL_BAND else 0)     # positive roll: right side low
    rear = 1 if pitch < -PITCH_BAND else 0                                # negative pitch: rear low
    return ((rear * len(ROLLS) + side) * len(STEERS) + steer) * 4 + step


robot.simulate_classifier(matrices, selector)
