def policy(robot_state):
    K = np.zeros((12, 6))

    # Joint positions and velocities
    q = robot_state[0:12]
    dq = robot_state[12:24]

    # Body orientation deviation [roll, pitch, yaw], angular velocity [wx, wy, wz], lateral velocity vy
    roll, pitch, yaw = robot_state[24], robot_state[25], robot_state[26]
    wx, wy, wz = robot_state[27], robot_state[28], robot_state[29]
    vy = robot_state[31]

    # Antisymmetric coordinates between Diagonal 1 (FR, RL) and Diagonal 2 (FL, RR)
    # x: thigh difference; y: calf difference (calf leads thigh by 90 deg in the gait cycle)
    x = 0.25 * ((q[1] + q[10]) - (q[4] + q[7]))
    y = 0.25 * ((q[2] + q[11]) - (q[5] + q[8]))

    # Normalize into dimensionless cycle coordinates (u = sin(phi), w = cos(phi))
    ax_norm = 0.219
    ay_norm = 0.242
    u = -x / ax_norm
    w = -y / ay_norm

    # Smooth symmetry-breaking bias at the origin (home pose) to initiate Diag 1 calf tuck first
    r2 = u * u + w * w
    w_eff = w + 0.25 * np.exp(-19.8 * r2)

    # Unit phase vector from current measured leg state
    r = np.sqrt(u * u + w_eff * w_eff + 1e-6)
    s_phi = u / r
    c_phi = w_eff / r

    # Rotate phase ahead by delta_phi (~66 deg) so target always leads current leg state
    delta_phi = 1.15
    cos_d = np.cos(delta_phi)
    sin_d = np.sin(delta_phi)

    sin_psi = s_phi * cos_d + c_phi * sin_d
    cos_psi = c_phi * cos_d - s_phi * sin_d

    # Stride amplitudes and nominal biases (from best-scoring Iteration 50)
    thigh_base = 0.926
    thigh_amp = 0.22

    calf_base = -1.810
    calf_sym_amp = 0.10
    calf_tuck_amp = 0.28

    # Thigh targets: swing leg drives below home, stance leg sweeps above home
    thigh_d1 = thigh_base - thigh_amp * sin_psi
    thigh_d2 = thigh_base + thigh_amp * sin_psi

    # Calf targets: swing leg tucks sharply below home, stance leg holds near home
    tuck_d1 = np.maximum(0.0, cos_psi)
    tuck_d2 = np.maximum(0.0, -cos_psi)

    calf_d1 = calf_base - calf_sym_amp * cos_psi - calf_tuck_amp * tuck_d1
    calf_d2 = calf_base + calf_sym_amp * cos_psi - calf_tuck_amp * tuck_d2

    # Posture, lateral velocity, and heading feedback trims
    hip_roll = np.clip(0.20 * roll + 0.04 * wx + 0.04 * vy, -0.08, 0.08)
    pitch_trim = np.clip(0.16 * pitch + 0.033 * wy + 0.0045, -0.06, 0.06)
    steer = np.clip(0.095 * yaw + 0.036 * wz, -0.06, 0.06)

    # FR (Diagonal 1, right)
    K[0, 5] = -0.02 + hip_roll
    K[1, 5] = thigh_d1 - pitch_trim + steer
    K[2, 5] = calf_d1

    # FL (Diagonal 2, left)
    K[3, 5] = 0.02 + hip_roll
    K[4, 5] = thigh_d2 - pitch_trim - steer
    K[5, 5] = calf_d2

    # RR (Diagonal 2, right)
    K[6, 5] = -0.02 + hip_roll
    K[7, 5] = thigh_d2 + pitch_trim + steer
    K[8, 5] = calf_d2

    # RL (Diagonal 1, left)
    K[9, 5] = 0.02 + hip_roll
    K[10, 5] = thigh_d1 + pitch_trim - steer
    K[11, 5] = calf_d1

    # Joint-velocity damping to soften foot impacts, reduce acceleration spikes, and lower torque penalty
    K[:, 5] -= 0.0155 * dq

    return K

robot.simulate_rollout(policy)