# Go2 setup on the robot computer

Follow these steps on the Ubuntu computer wired to the robot. They end with a read-only check that sends no commands to the robot.

**Don't run other scripts yet.** Unitree's examples can move the robot as soon as you give them a network interface. `stand_test.py` only works with the simulator, so it can't reach the robot. `run_policy.py` only reaches the robot if you pass `--robot`, and [Run a policy](#run-a-policy) covers when to do that. `check_lidar.py` is also read-only, and [Check the lidar](#check-the-lidar) covers it.

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

Set it here, replacing the example name, then give the port an address on the robot's network:

```bash
IFACE=enp3s0
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

Each run saves its data to `go2/logs/<time>.npz`. Sport mode stays off after a run, so restart the robot to get it back.
