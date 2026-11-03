def policy(robot_state):
    # Joint Indices:
    # FR: 0,1,2 | FL: 3,4,5 | RR: 6,7,8 | RL: 9,10,11
    # Thighs: 1, 4, 7, 10
    
    # Read current thigh positions for the diagonal trot (FR and RL)
    fr_thigh = robot_state[1]
    rl_thigh = robot_state[10]
    avg_a = (fr_thigh + rl_thigh) / 2.0
    
    # Hysteresis for stable, long strides
    if avg_a > 0.98:
        phase_a = "swing"
        phase_b = "stance"
    elif avg_a < 0.82:
        phase_a = "stance"
        phase_b = "swing"
    else:
        # Mid-band: maintain current state based on threshold
        phase_a = "stance" if avg_a < 0.9 else "swing"
        phase_b = "swing" if avg_a < 0.9 else "stance"
        
    # Target Constants
    SWING_THIGH = 0.65
    STANCE_THIGH = 1.20
    SWING_CALF = -2.2
    STANCE_CALF = -1.7
    HOME_HIP = 0.0
    
    K = np.zeros((12, 6))
    targets = np.zeros(12)
    
    def set_leg(start_idx, phase):
        if phase == "swing":
            targets[start_idx] = HOME_HIP
            targets[start_idx+1] = SWING_THIGH
            targets[start_idx+2] = SWING_CALF
        else:
            targets[start_idx] = HOME_HIP
            targets[start_idx+1] = STANCE_THIGH
            targets[start_idx+2] = STANCE_CALF
    
    # Diagonal Trot Assignment: (FR, RL) vs (FL, RR)
    set_leg(0, phase_a) # FR
    set_leg(3, phase_b) # FL
    set_leg(6, phase_b) # RR
    set_leg(9, phase_a) # RL
    
    # Set nominal pose in Column 5 (Bias)
    K[:, 5] = targets
    
    # Roll Stability: Adjust hips based on roll (robot_state[24])
    # Positive roll = tilt right.
    roll = robot_state[24]
    roll_gain = -0.25 
    K[0, 5] += roll * roll_gain # FR
    K[3, 5] -= roll * roll_gain # FL
    K[6, 5] += roll * roll_gain # RR
    K[9, 5] -= roll * roll_gain # RL
    
    # Pitch Stability: Adjust front thighs based on pitch (robot_state[25])
    pitch = robot_state[25]
    pitch_gain = -0.12
    K[1, 5] += pitch * pitch_gain # FR thigh
    K[4, 5] += pitch * pitch_gain # FL thigh
    
    # Terrain Response: Map side probes to corresponding legs during swing
    # env[0]: ahead-left, env[1]: ahead-center, env[2]: ahead-right
    
    # FR (Right leg) - Swing phase a
    if phase_a == "swing":
        K[2, 2] = -0.6 # Right probe -> FR calf
    # FL (Left leg) - Swing phase b
    if phase_b == "swing":
        K[5, 0] = -0.6 # Left probe -> FL calf
    # RR (Right leg) - Swing phase b
    if phase_b == "swing":
        K[8, 2] = -0.6 # Right probe -> RR calf
    # RL (Left leg) - Swing phase a
    if phase_a == "swing":
        K[11, 0] = -0.6 # Left probe -> RL calf
        
    return K

robot.simulate_rollout(policy)