# Go2 setup on the robot computer

Follow these steps on the Ubuntu computer wired to the robot. They end with a read-only check that sends no commands to the robot.

**Don't run other scripts yet.** Unitree's examples can move the robot as soon as you give them a network interface. `stand_test.py` only works with the simulator, so it can't reach the robot. `run_policy.py` only reaches the robot if you pass `--robot`, and [Run a policy](#run-a-policy) covers when to do that. `check_lidar.py` and `lidar_map.py` are also read-only, and [Check the lidar](#check-the-lidar) covers them. So is `check_odom.py`, which [Check the odometry](#check-the-odometry) covers. `phone_camera.py` never talks to the robot, and [Film with a phone](#film-with-a-phone) covers it.

## 1. Install

You need `git`, `conda`, and `sudo` rights for the network step. Check the first two with:

```bash
git --version
conda --version
```

Then install the SDK:

```bash
git clone -b go2-dev https://github.com/daniel-campa/unitree_sdk2_python.git
cd unitree_sdk2_python
go2/setup_env.sh
```

It should finish with `SDK imports OK`.

To get later changes, run `git pull` in this folder.

## 2. Prepare the robot

1. Put the robot on the floor or a stand, with clear space around it.
2. Turn it on and let it finish starting up. Don't change its mode.
3. Connect it to this computer with an Ethernet cable.

## 3. Set up the network

Find the Ethernet port's name, which usually starts with `en`:

```bash
ip -br link
```

The scripts default to `enx80691a8d7c72`. If yours is different, use its name below and pass it to each script. Set it here, then give the port an address on the robot's network:

```bash
IFACE=enx80691a8d7c72
sudo ip addr add 192.168.123.51/24 dev "$IFACE"
sudo ip link set "$IFACE" up
```

This adds no default route, so Wi-Fi and internet keep working. The address lasts until the computer reboots or the cable is reconnected, so repeat this step if it disappears.

Optional: test the link. Unitree's documented address for the Go2 is usually `192.168.123.161`, but this hasn't been tried on your robot, so a failed ping alone doesn't prove a problem.

```bash
ping -c 3 192.168.123.161
```

## 4. Check the connection

```bash
conda activate go2
export PYTHONNOUSERSITE=1
python go2/check_connection.py "$IFACE"
```

**A PASS only means valid messages arrived.** Also check that:

- Battery, motor temperatures and foot force are non-zero and sensible.
- Joint angles match the robot's pose.
- `rt/sportmodestate` messages are arriving, since sport mode should still be on.

Save the output to share it:

```bash
python go2/check_connection.py "$IFACE" | tee connection_check.txt
```

## If something fails

| Message | What to try |
|---|---|
| `no network interface called ...` | Use a name from the list it prints. |
| `... is down` | Check the cable and that the robot is on, then repeat step 3. |
| `has no IPv4 address` | Repeat step 3. |
| `channel factory init error` | CycloneDDS couldn't start, and the line above the error may say why. It writes a log to `/tmp/cdds-<uid>.LOG`, so if that can't be opened, delete the file. |
| `has no 192.168.123.x address` | Repeat step 3. If the address keeps disappearing, the network manager is probably removing it, so see below. |
| `no rt/lowstate received` | Check the robot's power and the cable, then run `ip addr show "$IFACE"` to confirm the address. A firewall can also block this, so check `sudo ufw status`. |
| `conda: command not found` | Install Miniconda, or run `source ~/miniconda3/etc/profile.d/conda.sh` if it's already installed. |

To make the address stay with NetworkManager, using a new connection called `go2`:

```bash
sudo nmcli con add type ethernet ifname "$IFACE" con-name go2 ipv4.method manual ipv4.addresses 192.168.123.51/24 ipv6.method disabled
sudo nmcli con up go2
```

## Optional: test without the robot

This needs a display-free run of the simulator in one terminal and the test in another. It can't reach the robot.

```bash
go2/setup_env.sh --with-sim

# Terminal 1
conda activate go2; export PYTHONNOUSERSITE=1
python go2/sim_headless.py

# Terminal 2
conda activate go2; export PYTHONNOUSERSITE=1
python go2/stand_test.py
```

The test should end with `PASS`. Press Ctrl-C in terminal 1 to stop the simulator.

## Check the lidar

`check_lidar.py` shows whether the lidar can supply the seven terrain beams that the `platform_curve` policies steer on. It only subscribes to topics, so it can't move the robot or switch the lidar on or off.

**This hasn't been tried on a real robot.** It has only run against a fake publisher. Its topic names come from third-party docs, and the frame names and how the height map marks unknown cells aren't documented, so it may need changes after the first run.

Do sections 1-4 first, so `$IFACE` is set. Keep the robot still while it runs:

```bash
python go2/check_lidar.py "$IFACE" | tee lidar_check.txt
```

**A PASS only means data reached all seven beam locations.** It doesn't show the readings are right, so do these runs, saving each to its own file. Distances are from the middle of the body.

1. **Flat floor, standing.** Every beam should read about the same height, and the three differences should be near 0. `spread` should be small compared with the 0.30 m an edge produces.
2. **Low box on the left.** Stand a box about 10 cm high in front of the robot, covering 0.35-0.55 m ahead and 0.05-0.45 m left of the centreline. The three differences should read about +0.10. A negative value means left and right are swapped.
3. **Raised edge, if you have one.** Stand the robot on a stable raised surface with the drop close on one side, and keep someone next to it. A lidar can't see the floor just past a raised edge, so those beams may show `--` instead of the -0.57 they read in simulation. A `FAIL` here is a useful result, so note which beams have data.
4. **Sport mode off.** Sport mode stays off after stage 2 of [Test on the robot](#test-on-the-robot) until you restart the robot. Run the check again then, to see which topics still publish. The robot is lying down by that point, so only the rates and `valid %` mean anything.

| Message | What it means |
|---|---|
| `no lidar topics received` | The lidar may be off, or this firmware may use other topic names. Don't try `example/go2/high_level/go2_utlidar_switch.py` as a check, because as written it switches the lidar off. |
| `frame ... isn't handled` | The script can't place that topic's points relative to the body. This doesn't matter if another source passes. Otherwise, the frame needs adding to the script. |
| `needs the robot's pose` | That topic's data needs `rt/utlidar/robot_odom`, which isn't arriving. |
| `no source covered all seven beams` | Check `valid %`. A low value on every beam suggests sparse data, and a value near 0 on some beams suggests the lidar can't see them. `--ahead` tries the beams at another distance, to find where coverage starts. |

### See the height map

`lidar_map.py` draws the height map live, with the robot and the seven beams on top. It's also read-only. Close the window to stop it, or use `--save` to get one picture without needing a display:

```bash
python go2/lidar_map.py "$IFACE"
python go2/lidar_map.py "$IFACE" --save lidar_map.png
```

- **Colours:** these show height relative to the base, on a fixed scale from -0.7 to +0.1 m. Grey cells are unknown, and anything above +0.1 m shows as the top colour.
- **Robot:** the red dot is the base, and the arrow points to the middle beam. Beam `0` should be on the robot's left.
- **Terminal:** it prints the most repeated values in the first map. A value repeated across many cells is probably the marker for unknown cells. If that marker is smaller than 100 m, the script draws it as a real height.

### Try it in the simulator

To see what the report looks like without a robot, `sim_headless.py --lidar` publishes simulated lidar topics. `--box` adds the box from run 2 above. Run each command in its own terminal:

```bash
python go2/sim_headless.py --lidar --box
python go2/stand_test.py --hold 15
python go2/check_lidar.py lo --domain 1 --seconds 5
```

Start the check about 3 s after `stand_test.py`, once the robot is standing. Without `stand_test.py` the robot lies on the floor, so the heights are smaller, but the differences stay the same.

**A PASS here says nothing about the real robot.** The simulated data has no noise, and its frames, scan pattern and lidar mount are assumptions. `rt/sportmodestate` always publishes in the simulator, so ignore the sport mode line too. The simulated lidar also slows the simulator by about 7%, so leave it off when testing policies.

## Run a policy

`run_policy.py` runs a policy from S-M-ICRL. It uses the simulator unless you pass `--robot`.

`policies/walk_tmnxt_iter17.py` is an unchanged copy of `iter_17/code.py` from the run `walk_2026-10-27_17-37-32_TMNXT`. The script's default rate and gains (5 Hz, kp 80, kd 4) come from that run. Keep them for this policy, because the rate sets its gait and lower gains make it walk much less far. A policy from another run needs the values from that run's `config.json`.

The script feeds the policy fixed flat-floor values instead of real terrain probes, so **the robot can't sense steps or obstacles**.

It refuses a policy whose output depends on body height or linear velocity (`robot_state[30:34]`), because the robot can't measure them. A policy may keep state between calls. `--check` only loads and checks the policy, without connecting to anything. S-M-ICRL's real-robot loop uses it before each run.

### Test in the simulator

Install the simulator with `go2/setup_env.sh --with-sim`, then:

```bash
# Terminal 1
conda activate go2; export PYTHONNOUSERSITE=1
python go2/sim_headless.py --flat --dt 0.002

# Terminal 2
conda activate go2; export PYTHONNOUSERSITE=1
python go2/run_policy.py go2/policies/walk_tmnxt_iter17.py --seconds 10
```

Terminal 2 should end with `Result: finished`, and `base_x` in terminal 1 should reach about 6-7 m.

If a calf speed fault stops the walk, check two things. The simulator needs `--dt 0.002`, because at its default 5 ms the calves oscillate. It also has to keep up with real time, so don't run other heavy jobs alongside it.

### Test on the robot

**None of this has been tried on a real robot yet.** Do these stages in order, and stop if anything looks or sounds wrong. You need sections 1-4 above done, `$IFACE` set, the remote, and a harness or stand that can hold the robot.

The script asks you to type `yes` and press B on the remote, then lies the robot down and turns sport mode off. It stands up and waits for you to press Enter before walking.

**1. Read-only.** This publishes nothing, so it can't move the robot:

```bash
python go2/run_policy.py go2/policies/walk_tmnxt_iter17.py --robot "$IFACE" --dry-run --seconds 30
```

Put the robot in damping mode with the remote or app so the legs move by hand, then check that:

- Lowering the right side raises `roll`, and lowering the nose raises `pitch`.
- Each thigh value rises as that leg swings backward, and each calf value gets more negative as the knee folds.
- `B button` shows `pressed` while you hold B.

**2. Hanging, feet clear of the ground.** Run `--seconds 0` first, which only stands up and lies down, then `--seconds 5`:

```bash
python go2/run_policy.py go2/policies/walk_tmnxt_iter17.py --robot "$IFACE" --seconds 0
python go2/run_policy.py go2/policies/walk_tmnxt_iter17.py --robot "$IFACE" --seconds 5
```

In simulation the target changes on nearly every policy call, thighs stay within 0.65-1.2 rad, and calves within -2.05 to -1.7 rad. A buzzing or fast shaking leg isn't expected, so stop if you see one.

While it's hanging, also test each way of stopping:

- Press B during a run.
- Press Ctrl-C during a run.
- Find out what the motors do when commands stop, by running `pkill -9 -f run_policy.py` in a second terminal during a run. The script can't damp the motors after this, so keep the robot held and clear of people. You'll need to restart the robot afterward.

**3. On the floor, in a slack harness.** Run `--seconds 0`, then `--seconds 2`, and build up through 3 and 5 to 10:

```bash
python go2/run_policy.py go2/policies/walk_tmnxt_iter17.py --robot "$IFACE" --seconds 2
```

- Clear a flat lane of about 8 x 4 m. Feet only lift about 2-3.5 cm, so remove cables and anything else above 1 cm.
- Expect about 0.6 m/s and some sideways drift, because nothing corrects its heading. Drift reached about 1.6 m in simulation.
- Keep someone ready to press B, and watch `hottest` and `current` in the output.

### What stops a run

These go straight to damping, which lets the robot sink to the ground:

- B on the remote, or Ctrl-C
- Tilt above 0.5 rad
- Joint speed above 25 rad/s (hip, thigh) or 14 rad/s (calf)
- A motor at 70 °C
- A joint at its limit
- No robot state for 0.1 s
- The control loop stalling for 0.2 s, or the policy failing

**Nothing protects the robot if the script is killed, the computer fails, or the cable comes out.** Stage 2 shows you what happens then.

The script aims to keep torque within 60% of the limits (`--torque-fraction`). Peaks can go above that: the thigh reached about 21 N·m in simulation, against a 14 N·m target and a 23.7 N·m limit.

Each run saves its data to `go2/logs/<time>.npz`, including `rt/utlidar/robot_odom` when it publishes. Sport mode stays off after a run, so restart the robot to get it back.

## Check the odometry

`check_odom.py` tests whether the robot's own position estimate is good enough to score S-M-ICRL's walk reward on the real robot. It only subscribes to topics. The robot moves only when you move it: with the remote, with `run_policy.py` in another terminal, or by hand.

Do this once [Test on the robot](#test-on-the-robot) works. You need two terminals, the remote, floor tape, a tape measure, and the clear lane from stage 3. Then run:

```bash
python go2/check_odom.py record "$IFACE"
```

It takes you through five steps, waiting for Enter between them:

| Step | What you do | What it shows |
|---|---|---|
| 1. Still, sport mode on | Leave the robot standing as it is after start-up, and stand back for 20 s | the noise with sport mode running |
| 2. Remote walk (optional) | Walk it about 1 m forward with the remote | accuracy with sport mode's own estimator running |
| 3. Policy walk | Run `run_policy.py --robot` in the other terminal | whether each topic survives sport mode being released and follows a low-level walk |
| 4. Moved by hand (optional) | Slide or carry it about 1 m forward, holding the body | whether each topic follows the robot when no legs are stepping |
| 5. Still, sport mode off | Stand back for 20 s | the noise after the release |

- **Measuring:** before each move step, lay tape along the robot's heading and mark the floor under the middle of its body. Afterwards, type how far the middle of its body moved along the tape, and to the left of it (right is negative). Leave a blank for anything you didn't measure.
- **Still steps:** the lidar sees people, so stand well back. `--still-seconds` changes their length.
- **Stopping:** Ctrl-C in this terminal only stops the recording, not the robot. Each step is saved as it finishes.
- **Timing:** recording runs alongside `run_policy.py` on the same computer. Check that its `worst loop overrun` stays like your earlier runs.

When it finishes, it prints a report and saves the recording to `go2/logs/odom_<time>.npz`, with a plot next to it. To see the report again, run `python go2/check_odom.py analyze go2/logs/odom_<time>.npz`. `--free` skips the steps and records until Ctrl-C.

### Reading the report

It checks four topics. `rt/utlidar/robot_odom` and `rt/sportmodestate` published on the robot with sport mode on. `rt/utlidar/robot_pose` and `rt/odommodestate` are guesses, and `no messages` for them may only mean that their message type is different.

1. **Publishing after sport mode was released.** `rt/sportmodestate` stops when sport mode is released, so that's taken as the release time. `STOPPED` means a topic can't be used under low-level control. A `jumped` line just after the release suggests that the estimate reset.
2. **Following the robot.** This shows how far each topic says the robot moved in each move step, next to your measurements and the IMU's yaw. `DIDN'T MOVE` means the topic stayed put while the legs were moving.
3. **Walk reward added by noise.** The walk reward adds up every small forward step in x and only part of every backward one, so noise alone earns reward. The table shows how much a still robot would earn in one 5 s episode, unfiltered and after moving averages of 0.1-0.5 s. Compare it with the 9 that walking 1 m earns. Besides the still steps, it uses stretches where the joints and gyro show the robot was still, such as the stand before the policy walks.
4. **Height and yaw.** This compares each topic's z with the base height from leg kinematics, and its yaw with the IMU's. If the std is small, that z can stand in for the simulator's base height once the offset is removed.

### Try the odometry check in the simulator

Run each command in its own terminal, and stop the recording with Ctrl-C once the robot has lain down:

```bash
python go2/sim_headless.py --flat --dt 0.002 --lidar
python go2/check_odom.py record lo --domain 1 --free
python go2/run_policy.py go2/policies/walk_tmnxt_iter17.py --seconds 8
```

**This only shows that the script works.** The simulated `robot_odom` is the true position, with no noise, and `rt/sportmodestate` never stops in the simulator.

## Film with a phone

S-M-ICRL's real-robot loop can show the LLM pictures of each run, filmed by a phone. `phone_camera.py` reads the phone's video stream over the network. It never talks to the robot.

**This hasn't been tried with a real phone.** It has only read a fake stream on this computer. The app names and addresses below are from the apps' own docs, so trust what the app shows over them.

### 1. Stream from the phone

Install an app that streams the camera over HTTP, and start its server:

| Phone | App | Stream address |
|---|---|---|
| Android | IP Webcam | `http://<phone IP>:8080/video`. The app shows the IP at the bottom of the screen after **Start server**. |
| Android or iPhone | DroidCam | `http://<phone IP>:4747/video` |

Any app that gives an MJPEG or RTSP address should work, and the check below shows if yours does. In the app, choose landscape and about 1280x720 at 30 fps. Keep the phone charging, and stop its screen from locking, because many apps stop streaming when it locks. If the app can lock focus and exposure, lock them.

### 2. Let this computer reach the phone

They're usually both on the same Wi-Fi. Many campus and guest networks stop devices reaching each other, though. If the check below says `can't open`, try one of these:

- **Phone hotspot:** connect this computer's Wi-Fi to the phone's hotspot, and use the address the app shows then. The LLM's traffic goes over the phone's data.
- **Tailscale:** if this computer uses Tailscale, install it on the phone too, and use the phone's `100.x.y.z` address. This works across networks.
- **SSH tunnel:** if another computer can open the stream (its browser shows it) and can SSH into this one, run `ssh -N -R 8080:<phone IP>:8080 <user>@<this computer>` on it, and leave it running. This computer then uses `http://localhost:8080/video`. The stream goes through the other computer, so it has to stay on.
- **USB:** some Android phones (Android 14 or later) can be a USB webcam: plug it in and choose **Webcam** in the USB notification. Then use the camera's number instead of an address, for example `0` for `/dev/video0`. The cable has to reach the phone's spot.

The phone's address can change when it reconnects, so if a check that used to pass fails, look at the app again.

### 3. Place the phone

Put it on a tripod or something else steady, 2-3 m to the side of the lane, level with the middle of the lane, and a little above the robot's height. Point it across the lane, so the robot crosses the picture from one side to the other. Frame the start mark and as far ahead as a good walk reaches. Leave it in the same place for the whole loop, so that the LLM can compare pictures between iterations.

### 4. Check the stream

Stand the robot at the start mark, then run:

```bash
python go2/phone_camera.py check http://192.168.1.23:8080/video
```

It reads the stream for 5 s and saves a frame as `camera_check.jpg`. Open the frame to check the view. A `PASS` means at least 10 frames per second arrived, with no gap over 0.5 s.

| Message | What to try |
|---|---|
| `can't open ...` | Open the address in a browser on this computer. If the browser shows video, the address is fine. If it doesn't, check the address and that the app is streaming. If both are right, the network is keeping the devices apart, so see step 2. |
| `opened but sent no frames` | Close any other viewer, such as a browser tab, because some apps serve one viewer at a time. |
| `... frames/s is below 10` or `a ... s gap between frames` | Lower the app's resolution or quality, or move the phone or the computer closer to the Wi-Fi access point. |

To try a recording, run `python go2/phone_camera.py record <address> test.mp4` and press Ctrl-C to stop it. Next to the video, `test.json` keeps when each frame arrived.

### 5. Use it in the loop

In S-M-ICRL's `func_config/go2/walk/walk_real.yaml`, set:

```yaml
llm:
  send_images: true
real_robot:
  camera: http://192.168.1.23:8080/video
```

S-M-ICRL's `real_robot.md` explains what the loop does with the pictures.

**Timing:** the recording runs alongside `run_policy.py` on the same computer, at a lower priority. In the simulator on a 12-core computer, the worst loop overrun stayed under 2 ms with and without it. On the robot, compare the `Timing` line of your first runs with the camera against your earlier runs.
