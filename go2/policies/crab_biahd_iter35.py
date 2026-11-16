def policy(robot_state):
    if not hasattr(policy, "t"):
        policy.t = 0
        policy.yaw_i = 0.0
        policy.x_i = 0.0
        policy.roll_i = 0.0
        policy.pitch_i = 0.0
        policy.yaw_f = 0.0
    t = policy.t
    policy.t += 1

    yaw = robot_state[26]
    wz = robot_state[28]
    roll = robot_state[24]
    pitch = robot_state[25]
    vx = robot_state[30]
    h = robot_state[33]

    def cl(v, lo, hi):
        return max(lo, min(hi, v))

    SETTLE = 4                 # start walking early (proven safe)
    if t >= SETTLE:
        # low-pass filtered yaw: tracks drift, rejects gait-frequency wobble
        policy.yaw_f = 0.75 * policy.yaw_f + 0.25 * yaw
        policy.yaw_i = cl(policy.yaw_i + 0.12 * policy.yaw_f, -0.3, 0.3)
        policy.x_i = cl(policy.x_i + 0.15 * vx, -0.25, 0.25)
        policy.roll_i = cl(policy.roll_i + 0.05 * roll, -0.2, 0.2)
        policy.pitch_i = cl(policy.pitch_i + 0.05 * pitch, -0.2, 0.2)

    N = 14                     # 1.4 s per trot cycle (proven best)
    A = 0.65                   # push stroke amplitude past iter-34's 0.63
    SW = 0.36                  # swing fraction
    END = 158                  # walk to the end before the held final second

    # yaw feedback: filtered yaw + raw rate + integral -> front strokes shrink,
    # rear strokes grow when yawing left (proven best gains)
    k = cl(1.0 * policy.yaw_f + 0.70 * wz + 0.7 * policy.yaw_i, -0.45, 0.45)
    A_f = A * (1.0 - k)
    A_r = A * (1.0 + k)

    # x-drift trim: thigh bias to push body back toward x=0
    dth = cl(-1.1 * policy.x_i, -0.15, 0.15)
    # roll trim: abduction toward the low side (slightly stronger for larger stroke)
    dr = cl(0.30 * roll + 0.35 * policy.roll_i, -0.12, 0.12)
    # pitch trim: calf differential front vs rear (proven gains)
    dp = cl(0.45 * pitch + 0.4 * policy.pitch_i, -0.14, 0.14)

    # stance splay to widen the base for stability
    SPLAY = 0.06

    pose = np.array([0.0, 0.9, -1.8] * 4)

    if SETTLE <= t < END:
        tg = t - SETTLE
        ramp = min(1.0, (tg + 1) / float(N))
        frac = (tg % N) / float(N)
        h_err = 0.27 - h
        # diagonal trot: FR+RL swing first, then FL+RR (linear profiles)
        legs = [(0, 0.0, True), (3, 0.5, True), (6, 0.5, False), (9, 0.0, False)]
        for start, ph, front in legs:
            phi = (frac + ph) % 1.0
            Al = (A_f if front else A_r) * ramp
            if phi < SW:
                # swing: foot lifted (deeper tuck for clearance at large stroke)
                u = phi / SW
                hip = -Al + 2.0 * Al * u
                calf = -2.10
            else:
                # stance: planted foot sweeps back rightward, body moves left
                u = (phi - SW) / (1.0 - SW)
                hip = Al - 2.0 * Al * u
                # height hold: extend knees when body is low
                calf = cl(-1.78 + 0.6 * h_err, -2.35, -0.95)
            pose[start] = hip
            pose[start + 1] = 0.9 + dth
            pose[start + 2] = calf

    # stance splay: left legs out (+), right legs out (-)
    pose[3] += SPLAY
    pose[9] += SPLAY
    pose[0] -= SPLAY
    pose[6] -= SPLAY
    # roll: abduct toward the low side to catch the tilt
    pose[3] += dr
    pose[9] += dr
    pose[0] -= dr
    pose[6] -= dr
    # pitch: calf differential front vs rear
    for i in (0, 3):
        pose[i + 2] = cl(pose[i + 2] + dp, -2.35, -0.95)
    for i in (6, 9):
        pose[i + 2] = cl(pose[i + 2] - dp, -2.35, -0.95)

    K = np.zeros((12, 6))
    for j in range(12):
        K[j, 5] = pose[j]
    return K

robot.simulate_rollout(policy)