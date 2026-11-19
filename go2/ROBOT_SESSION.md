# Robot session: measure the Go2, run the argon walks, record everything

This session has two goals:

1. **Measure the robot** so the simulator can be made to match it: latency, friction, torque, sensor noise, standing sag. This includes checking that every policy input means on the robot what it means in simulation. Do this part even if no policy ever walks. Its data is useful on its own.
2. **Run the two walk policies with a recorder alongside**, so that if a walk fails, the logs show why. You don't need to work out the cause during the session. Record the runs and send the files back.

Total time is about 1.5 hours, most of it setup and moving the harness.

**Before you start, read README.md sections 1–4 and "Test on the robot".** This file assumes you know how to stop the robot: press **B** on the remote, or Ctrl-C. Nothing protects the robot if the script is killed or the cable comes out, so keep the robot in a harness or held until step B3.

The session has two programs:

- **`calibrate.py`** (Part A) runs once at the start: it checks the robot, measures it, checks every policy input, and writes `go2/logs/calibration/latest.json`. `run_policy.py` won't drive the robot without a calibration that passed.
- **`watch.py`** (Part B) runs next to every policy run and records everything: every message, the policy's inputs, timing, torque and energy use, and the computer's load.

---

## What you need

- SSH access to the Ubuntu computer wired to the robot, with this repo installed there (README section 1).
- That repo on the `jnana` with these tools: `calibrate.py`, `watch.py`, `diagnose.py`, `sysid.py`, `run_policy.py`, `state_estimator.py`, and the two argon policies (see Setup).
- `tmux` on the robot computer (`sudo apt install tmux` if `tmux -V` fails).
- The remote, charged. Check that **B** works.
- A harness or stand that holds the robot with all four feet clear of the ground.
- A flat lane about **8 m long and 4 m wide**, clear of cables and of anything taller than 1 cm (the feet lift only 2–3 cm).
- Two people: one at the computer, one next to the robot ready to press B.
- Optional: a phone filming each floor run from the side (README "Film with a phone"). Video helps a lot when reading the logs later.

## Setup (every session)

Everything runs on the robot computer, over SSH, inside tmux. Then a dropped connection doesn't stop a run: reconnect and reattach. Without tmux, a dropped connection stops the run and damps the motors.

```bash
ssh USER@ROBOT_PC
tmux new -s go2              # after a dropped connection: ssh in again, then: tmux attach -t go2
```

In tmux, `Ctrl-b %` splits the window into two panes and `Ctrl-b` then an arrow key moves between them. Part B needs two panes.

Get the latest code, where BRANCH is the branch Jnana pushed these tools to:

```bash
cd unitree_sdk2_python
git fetch && git checkout BRANCH && git pull
```

Then:

```bash
conda activate go2
export PYTHONNOUSERSITE=1

IFACE=enx80691a8d7c72        # replace with your port's name from: ip -br link
sudo ip addr add 192.168.123.51/24 dev "$IFACE"
sudo ip link set "$IFACE" up

mkdir -p go2/logs
python go2/check_connection.py "$IFACE" | tee go2/logs/connection_check.txt
```

It must say PASS, with sensible battery, temperatures and joint angles. **In every new pane, run the `cd`, `conda activate`, `export` and `IFACE=` lines again.**

---

## Part A: Pre-check and calibrate (about 10 minutes)

```bash
python go2/calibrate.py --robot "$IFACE"
```

It runs five steps in order. Before each one it says what to do with the robot and waits for Enter (`s` skips a step, `q` stops):

| Step | Robot | What it does | Moves the robot? |
|---|---|---|---|
| connection | standing in sport mode after start-up | checks the link and the state, and reads the robot's serial number | no |
| passive | standing still in sport mode, 10 s | state rate, timing, sensor noise, IMU bias at rest | no |
| inputs | **you walk it with the remote** in sport mode, 2 min | records everything, then checks each of the 37 policy inputs against a second source (see [the input table](#reference-what-the-policy-reads)), including the velocity and height estimate against Unitree's own | no, only you do |
| hang | **hanging in the harness, feet clear**, 2.5 min | latency, torque scale, friction, tracking at 0.5–4 Hz, each foot sensor's unloaded reading | **yes, the legs** |
| stand | **on the floor**, harness slack, 20 s | standing sag and torque, standing height, each foot sensor's loaded reading | **yes, stands up** |

During **inputs**, in the clear lane:
1. Stand still for 10 s.
2. Walk forward slowly about 2 m, then back.
3. Walk sideways about 1 m left, then right.
4. Turn on the spot, a half turn each way.
5. Stand still until it stops.

**hang** asks you to type `hanging` and press B, then releases sport mode, so it stays off until the robot is restarted. That's why the sport-mode steps come first. If anything buzzes or shakes, press B.

At the end it prints a summary:
- the result of each input check;
- whether the foot sensors work;
- the latency, and which argon policy to run first;
- **READY** or **NOT READY**, with the reasons.

| Summary says | What to do |
|---|---|
| READY | Go to Part B. |
| the input check failed for joints, roll/pitch/yaw, angular velocity or gravity | **Don't run any policy.** Send `go2/logs/calibration/` to Jnana. |
| the input check couldn't verify something | Redo that step with the robot still, walking and turning as told: `python go2/calibrate.py --robot "$IFACE" --steps inputs` (restart the robot first if sport mode is off). |
| the velocity/height estimate disagreed with sport mode | Policies can still run, but add `--fill-unmeasured` to `func_argon_walk`, and note it. |
| a step failed | Redo just that step with `--steps NAME`. |

Each step saves its result as soon as it finishes, so a step can be redone without the others. A kinematics CHECK on its own can be a false alarm, because Unitree doesn't document the frame of its foot positions, and it doesn't block. `python go2/calibrate.py --show` prints the summary again.

---

## Part B: Run the policies with the recorder

### Which policy first

| Latency from Part A | Run first | Why |
|---|---|---|
| 10 ms or less | `class_argon_walk` | Fastest in simulation (about 6.4 m in 10 s), and unaffected by latency up to 10 ms |
| more than 10 ms | `func_argon_walk` | Slower (about 3 m in 10 s), but keeps walking straight with 20–40 ms latency |

If there's time, run both.

### How every run works

Every run uses **two tmux panes**:

- **Pane 1, the recorder.** `watch.py` only listens and can't move the robot. Start it **before every run**: it saves one file when the run's commands stop, then exits.
  ```bash
  nice -n 10 python go2/watch.py --robot "$IFACE"
  ```
  Each second it prints a status line. It includes the estimated body velocity and height (`v` and `h`), and notes `no force` or `no accel` if the estimator can't use those sensors. It warns when latency is above 15 ms, when the state or commands stall, when a joint is at its torque limit, when the body tilts, and when a motor gets hot. When the run ends it prints what the run used: energy, battery, torque as a share of the limits, motor temperatures, lost packets, estimated distance, and the computer's CPU load and temperature.
- **Pane 2, the policy.** `run_policy.py` drives the robot. It checks the calibration first, and refuses if there isn't one, if it's from another robot, or if its input check failed. Then it asks you to type `yes` and press B, lies the robot down, releases sport mode, stands up, and waits for **Enter** before walking. The body velocity and height estimate is on by default.

**Always pass `--policy-hz 10`.** The default of 5 Hz is wrong for these two policies.

**Rates and gains, matched to the simulation:**
- **Control loop: 500 Hz.** This is fixed and is the same as the simulation's 500 Hz physics step.
- **Policy: 10 Hz** (`--policy-hz 10`). The policy is called every 50 control cycles, and its output is held between calls, as in the simulation.
- **Env vector: rebuilt at 25 Hz** (`--env-hz`, default 25). This matches the Go2 walk configs, so leave it out unless the policy's config used a different `env_hz`.
- **Gains: kp 80, kd 4** (`--kp`, `--kd`, the defaults). These are the training gains.

The run log records the rates and every policy call's time, and `diagnose.py report` shows the set and measured rates, so you can confirm each run held them.

Commands for pane 2:

```bash
# class_argon_walk: reads only joint angles and velocities
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds N

# func_argon_walk: also reads body velocity, which comes from the estimate
# (add --fill-unmeasured if the calibration said the estimate disagreed with sport mode)
python go2/run_policy.py go2/policies/func_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds N
```

Use these settings exactly. Don't change `--kp`, `--kd` or `--torque-fraction`; they're the values the policies were tested at.

### The stages, for each policy

Do these in order. Stop at the first stage that looks or sounds wrong, and write down what you saw.

**B1. Dry run (robot doesn't move).** Add `--dry-run --seconds 30` in place of `--seconds N`. It prints what it would command without sending it, with the estimated velocity, height and which feet it thinks are standing. The watcher isn't needed here. Put the robot in damping mode with the remote and move it by hand to check signs, as in README "Test on the robot" stage 1.

**B2. Hanging in the harness, feet clear.** Start the watcher, then run with `--seconds 0`, which only stands up and lies down. Start the watcher again, then run with `--seconds 5`. The legs should step in a steady rhythm. Buzzing or fast shaking is not expected, so press B if you see either.

**B3. On the floor, harness slack.** For each of `--seconds 2`, then `5`, then `10`: start the watcher, start the policy, press Enter, and watch. Someone stays by the robot with B.
- The run stops itself on tilt above 0.5 rad, an overheated motor, or a joint near its limit. That is a safe stop, not a crash.
- `class_argon_walk` should go about 0.6 m/s, so a 10 s run covers about 6 m. `func_argon_walk` should go about 0.3 m/s. Some sideways drift is normal.

### Write down for each run

Keep a short note per run, even if the run failed in the first second. Copy this table into a text file:

| Time | Policy | Stage / --seconds | What happened (walked / stumbled / turned / fell / stopped, distance by eye) | Watcher warnings |
|---|---|---|---|---|

The time ties the note to the log files, which are named by time.

---

## Part C: Send the files back

Copy the logs to your own computer. Run this there, not over SSH:

```bash
rsync -av USER@ROBOT_PC:unitree_sdk2_python/go2/logs/ robot_session_$(date +%Y%m%d)/
```

Add your notes and any phone videos to that folder, and send it to Jnana.

You don't need to analyze anything. Jnana compares each run with the same policy in simulation (`diagnose.py report`), and the robot's measurements with the simulator's (`sysid.py compare`).

### What gets recorded

Everything the robot sends is saved, whether or not the scripts use it.

| File | From | Contents |
|---|---|---|
| `go2/logs/connection_check.txt` | setup | connection check output |
| `go2/logs/calibration/latest.json` | `calibrate.py` | the calibration `run_policy.py` uses: robot serial, input check verdicts, foot sensor readings hanging and standing, IMU bias, latency, standing height, and every step's results |
| `go2/logs/calibration/<time>/` | `calibrate.py` | each step's full log: connection output, passive and hang and stand recordings, the input walk recording, `inputs_check.txt`, `summary.txt` |
| `go2/logs/<time>.npz` | `run_policy.py`, at 500 Hz | joint angles, velocities, accelerations, torques, temperatures and lost-packet counts; commanded and torque-capped targets; IMU quaternion, rpy, gyro and accelerometer; foot force (raw and Unitree's estimate); battery, voltage and current; the robot's tick and the state's age; the estimator's velocity, height and standing feet; **the full 37-input robot_state every cycle**. For every policy call: the exact robot_state and env vector the policy got, the matrix it returned, the target, and the state's age. The calibration it ran with. Also rt/utlidar/robot_odom if it publishes. |
| `go2/logs/watch/<time>.npz` | `watch.py`, every message | every field of rt/lowstate (all of the above plus raw encoder values, motor modes, battery cells and temperatures, remote buttons, fan speeds); every rt/lowcmd sent; every field of rt/sportmodestate (Unitree's position, velocity, body height, foot positions and speeds, gait, obstacle ranges) and rt/utlidar/robot_odom while they publish; the estimator and the full robot_state for every message; once a second, the computer's CPU, memory, temperature and run_policy.py's CPU share; a usage summary (energy, battery, torque share, temperatures, lost packets, estimated distance). |
| `<log>.diagnosis.json`, `<watch>.inputs.json` | `diagnose.py` | analysis results, if you ran them |
| phone videos, notes | you | what the robot actually did |

---

## Reference: what the policy reads

Each policy call gets a 37-input `robot_state` and multiplies its output matrix by a 6-column env vector. This is how each one is filled on the robot, and what checks it.

### robot_state

| Index | In S-M-ICRL | On the robot | Same? | Checked by |
|---|---|---|---|---|
| 0–11 joint angles (FR, FL, RR, RL × hip, thigh, calf) | joint positions, in actuator order | `motor_state[0:12].q`, same order | yes: same order, axes and limits in both models | calibration inputs step (within limits, and against Unitree's foot positions); B1 by hand |
| 12–23 joint velocities | joint velocities | `motor_state.dq` | yes (the robot's are noisier) | calibration inputs step (against the rate of the joint angles) |
| 24–25 roll, pitch | from the body quaternion | from the IMU quaternion, same formula | yes | calibration inputs step (against the IMU's own rpy); B1 by hand |
| 26 yaw | yaw minus 0; every rollout starts at yaw 0 | yaw minus the yaw when the walk started | yes: both are the heading change since the start | calibration inputs step (against the IMU's own rpy) |
| 27–29 angular velocity | body-frame angular velocity | IMU gyroscope, body frame | yes | calibration inputs step (against the quaternion's rate) |
| 30–32 linear velocity | world-frame body velocity, x = start heading | estimated from legs + IMU + foot force (`state_estimator.py`), the default. `--fill-unmeasured`: 0. `--no-estimate`: the policy is refused if it reads them | estimated | calibration inputs step (against sport mode's velocity) |
| 33 body height | base height above the floor | estimated from legs + IMU, the default. `--fill-unmeasured`: 0.27 m | estimated | calibration inputs step (against sport mode's body height) |
| 34–36 gravity direction | world up in body axes | from the IMU quaternion, same formula | yes | calibration inputs step (against the accelerometer at rest) |

### env vector

| Column | In S-M-ICRL, flat floor | On the robot | Same? |
|---|---|---|---|
| 0–2 ground probes | floor height 0.3 m ahead minus body height, rebuilt at 25 Hz: on a flat floor, minus the body height (−0.269 to −0.277 in the walk) | minus the estimated height, rebuilt at 25 Hz (`--env-hz`). `--no-estimate`/`--fill-unmeasured`: fixed at −0.262 | yes, on a flat floor |
| 3 clearance above | 2.0 (open sky) | 2.0 | yes |
| 4 obstacle distance | about 10.0 (clear path) | 10.0 | yes |
| 5 bias | 1.0 | 1.0 | yes |

As in S-M-ICRL, the policy's matrix is held between calls and multiplied by the latest env vector every control cycle, so a policy with gains on the probes adjusts its targets at 25 Hz. Neither argon policy uses columns 0–4. Obstacles and steps can't be sensed: the probes assume a flat floor, and clearance and obstacle distance are always "open". `class_argon_walk` reads inputs 1, 2, 4, 5, 7, 8, 10, 11 and 13, 16, 19, 22 (thighs, calves and thigh velocities). `func_argon_walk` also reads 24–29 and 31 (sideways velocity).

### The velocity and height estimate (30–33)

`state_estimator.py` runs every control cycle, from before the robot stands up, so it has settled by the walk. It starts from the calibration's foot sensor readings.

- **Standing feet:** picked from the foot force sensors. Their units are raw, so each foot's unloaded reading and range are learned as the robot moves. If the sensors don't vary, it uses the lowest feet instead.
- **Height:** from the standing feet's depth below the body, given the IMU's tilt. This assumes a flat floor.
- **Velocity:** a standing foot's contact point doesn't move, so the body moves opposite to it. The estimate allows for the foot sphere rolling, and the accelerometer fills in between steps. The accelerometer is used only while it reads about 1 g upward, so a bad one can't make the estimate drift.

In S-M-ICRL, with robot-like sensor noise and foot force sensors of unknown gain and offset, it read within 1.5 cm/s and 3 mm of the true values on `func_argon_walk`, which walked 2.8 m against 3.06 m when fed the true values. A foot that slips makes it read slow. The crab gait slides sideways in simulation, and its sideways speed read 0.06 m/s low. This hasn't been tried on the robot yet, which is what the calibration's inputs step checks.

---

## If something goes wrong

| Problem | What to do |
|---|---|
| Robot falls or stumbles | Press B, then write down how it fell. The logs from that run are still saved and are exactly what's needed. |
| `run_policy.py` refuses: no calibration, another robot, or a failed input check | Run `calibrate.py` (or the step it names). Only pass `--no-calibration` if Jnana says so. |
| `run_policy.py` refuses the policy for another reason | Copy the message. |
| Watcher status shows `no force` | The foot force sensors aren't varying, so the estimator uses foot height instead. Note it and carry on. |
| Watcher says `nothing recorded` | It was started after the run, or on a different interface. Start it first next time. |
| Watcher keeps warning `latency ... ms` | Note it. That is the main thing being looked for, so keep going. |
| `no robot state for ... ms` fault | The cable or the computer stalled. Check the cable, and close other heavy programs. |
| Connection problems | README "If something fails". |
| The SSH connection dropped | SSH in again and run `tmux attach -t go2`. Everything carried on. Without tmux, the run stopped and damped, and `go2/logs/hangup.log` says when. |
| Sport mode is gone after a run | Expected. Restart the robot to get it back. |
