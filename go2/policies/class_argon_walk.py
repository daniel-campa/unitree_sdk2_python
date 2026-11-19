def make_matrix(pose):
    K = np.zeros((12, 6), dtype=float)
    K[:, 5] = pose
    return K

# Joint order: FR(0,1,2), FL(3,4,5), RR(6,7,8), RL(9,10,11)
# Home standing pose is [0.0, 0.9, -1.8] per leg.

# Posture 0: FR+RL early swing (calf tucked), FL+RR early stance sweep & support
# Drives d_pos toward 0.79 - 1.01 = -0.22 while calf tucks clear of ground
pose_0 = [
    0.0, 0.79, -2.34,  # FR: lift and swing forward
    0.0, 1.01, -1.68,  # FL: stance sweep with firm calf support
    0.0, 1.01, -1.68,  # RR: stance sweep with firm calf support
    0.0, 0.79, -2.34   # RL: lift and swing forward
]

# Posture 1: FR+RL late swing & plant, FL+RR late stance push
# Drives d_pos further toward 0.67 - 1.13 = -0.46
pose_1 = [
    0.0, 0.67, -1.78,  # FR: extend forward and plant
    0.0, 1.13, -1.72,  # FL: finish stance push with extended calf support
    0.0, 1.13, -1.72,  # RR: finish stance push with extended calf support
    0.0, 0.67, -1.78   # RL: extend forward and plant
]

# Posture 2: FL+RR early swing (calf tucked), FR+RL early stance sweep & support
# Drives d_pos toward 1.01 - 0.79 = +0.22 while calf tucks clear of ground
pose_2 = [
    0.0, 1.01, -1.68,  # FR: stance sweep with firm calf support
    0.0, 0.79, -2.34,  # FL: lift and swing forward
    0.0, 0.79, -2.34,  # RR: lift and swing forward
    0.0, 1.01, -1.68   # RL: stance sweep with firm calf support
]

# Posture 3: FL+RR late swing & plant, FR+RL late stance push
# Drives d_pos further toward 1.13 - 0.67 = +0.46
pose_3 = [
    0.0, 1.13, -1.72,  # FR: finish stance push with extended calf support
    0.0, 0.67, -1.78,  # FL: extend forward and plant
    0.0, 0.67, -1.78,  # RR: extend forward and plant
    0.0, 1.13, -1.72   # RL: finish stance push with extended calf support
]

matrices = [
    make_matrix(pose_0),
    make_matrix(pose_1),
    make_matrix(pose_2),
    make_matrix(pose_3),
]

def selector(robot_state):
    # Diagonal average thigh positions and velocities
    q_diag1 = 0.5 * (robot_state[1] + robot_state[10])   # FR, RL thighs
    q_diag2 = 0.5 * (robot_state[4] + robot_state[7])    # FL, RR thighs
    dq_diag1 = 0.5 * (robot_state[13] + robot_state[22])
    dq_diag2 = 0.5 * (robot_state[16] + robot_state[19])

    # Diagonal average calf positions (tucked calf is more negative, leading thigh velocity)
    c_diag1 = 0.5 * (robot_state[2] + robot_state[11])   # FR, RL calves
    c_diag2 = 0.5 * (robot_state[5] + robot_state[8])    # FL, RR calves

    d_pos = q_diag1 - q_diag2
    d_vel = dq_diag1 - dq_diag2
    d_calf = c_diag1 - c_diag2

    # Outer limits trigger swing reversal
    if d_pos > 0.19:
        return 0
    elif d_pos < -0.19:
        return 2

    # Light calf damping prevents impact velocity chatter and triggers smooth end-of-stride reversal
    phase_rate = d_vel + 1.2 * d_calf

    # Inside hysteresis band [-0.19, 0.19], advance monotonically from lift (0/2) to plant (1/3)
    if phase_rate <= 0.0:
        return 0 if d_pos > -0.05 else 1
    else:
        return 2 if d_pos < 0.05 else 3

robot.simulate_classifier(matrices, selector)