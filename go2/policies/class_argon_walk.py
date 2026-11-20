k0 = np.zeros((12, 6))
k1 = np.zeros((12, 6))
k2 = np.zeros((12, 6))
k3 = np.zeros((12, 6))
k4 = np.zeros((12, 6))
k5 = np.zeros((12, 6))

# Nominal swing: thigh 0.52, calf -2.13
# Nominal stance: thigh 1.24, calf -1.51

# --- Straight pair (|heading_err| <= 0.015) ---
# k0: FR+RL swing, FL+RR stance
k0[:, 5] = [
    0.0, 0.52, -2.13,  # FR (swing)
    0.0, 1.24, -1.51,  # FL (stance)
    0.0, 1.24, -1.51,  # RR (stance)
    0.0, 0.52, -2.13,  # RL (swing)
]

# k1: FL+RR swing, FR+RL stance
k1[:, 5] = [
    0.0, 1.24, -1.51,  # FR (stance)
    0.0, 0.52, -2.13,  # FL (swing)
    0.0, 0.52, -2.13,  # RR (swing)
    0.0, 1.24, -1.51,  # RL (stance)
]

# --- Steer right pair (heading_err > 0.015: robot angled left, push harder with left legs) ---
# k2: FR+RL swing, FL+RR stance
k2[:, 5] = [
    0.0, 0.545, -2.13, # FR (swing slightly shorter)
    0.0, 1.29, -1.51,  # FL (stance stronger push)
    0.0, 1.19, -1.51,  # RR (stance lighter push)
    0.0, 0.495, -2.13, # RL (swing slightly further)
]

# k3: FL+RR swing, FR+RL stance
k3[:, 5] = [
    0.0, 1.19, -1.51,  # FR (stance lighter push)
    0.0, 0.495, -2.13, # FL (swing slightly further)
    0.0, 0.545, -2.13, # RR (swing slightly shorter)
    0.0, 1.29, -1.51,  # RL (stance stronger push)
]

# --- Steer left pair (heading_err < -0.015: robot angled right, push harder with right legs) ---
# k4: FR+RL swing, FL+RR stance
k4[:, 5] = [
    0.0, 0.495, -2.13, # FR (swing slightly further)
    0.0, 1.19, -1.51,  # FL (stance lighter push)
    0.0, 1.29, -1.51,  # RR (stance stronger push)
    0.0, 0.545, -2.13, # RL (swing slightly shorter)
]

# k5: FL+RR swing, FR+RL stance
k5[:, 5] = [
    0.0, 1.29, -1.51,  # FR (stance stronger push)
    0.0, 0.545, -2.13, # FL (swing slightly shorter)
    0.0, 0.495, -2.13, # RR (swing slightly further)
    0.0, 1.19, -1.51,  # RL (stance lighter push)
]

matrices = [k0, k1, k2, k3, k4, k5]

def selector(robot_state):
    # q > 0 when FR+RL are swept back relative to FL+RR
    q = (robot_state[1] + robot_state[10]) - (robot_state[4] + robot_state[7])
    dq = (robot_state[13] + robot_state[22]) - (robot_state[16] + robot_state[19])

    thresh = 0.38
    if q <= -thresh:
        phase = 1  # FR+RL reached forward -> switch to FR+RL stance (FL+RR swing)
    elif q >= thresh:
        phase = 0  # FL+RR reached forward -> switch to FR+RL swing (FL+RR stance)
    else:
        phase = 1 if dq > 0.0 else 0

    heading_err = robot_state[26] + 0.04 * robot_state[29]
    if heading_err > 0.015:
        return phase + 2
    elif heading_err < -0.015:
        return phase + 4
    else:
        return phase

robot.simulate_classifier(matrices, selector)