# Lidar session 2: the env vector from the raw cloud

The 23 November session found three things:

- **Only the raw cloud survives.** `rt/utlidar/cloud`, in the lidar's own frame, keeps publishing with sport mode off. `rt/utlidar/cloud_deskewed` stops about 1.3 s after sport mode is released.
- **No mount was saved.** The calibration fitted the deskewed cloud, so the lidar's env vector was never built during a walk.
- **The box was hard to see.** One raw cloud is about 1,400 points, too few to see a box 2–3 m ahead reliably.

What changed:

- **Calibration.** `lidar_calibrate.py` now fits the raw cloud. It picks the way round closer to Unitree's Go2 model, and only counts a board near the distance you give.
- **Built-in mount.** `run_policy.py` uses the mount fitted offline to the 23 November recording when the calibration has none. It is 2.5° from Unitree's model.
- **Accumulation.** The env vector is built from the clouds of the last 0.4 s, about 6 clouds. Each cloud is moved by how far the robot has gone since. Replaying the 23 November box walk this way hit the box on 11 of 42 clouds, against 4 with single clouds.
- **Report.** `lidar_report.py` prints object_dist over the walk, and `--replay` rebuilds a walk's env vectors with other windows offline.

Steps 1 and 2 don't walk the robot; steps 3–5 do.

## 0. Copy the code to the robot computer

On the development computer:

```bash
cd /Volumes/DEV/IRL/unitree_sdk2
scp go2/lidar_env.py go2/lidar_worker.py go2/lidar_calibrate.py go2/lidar_report.py go2/run_policy.py \
    go2/LIDAR_SESSION_2.md daniel@en4166249l:/home/daniel/git/unitree_sdk2_python/go2/
```

`run_policy.py` includes your local change `MIN_BATTERY_PERCENT = 1`.

## Set-up on the robot computer (both terminals)

```bash
cd /home/daniel/git/unitree_sdk2_python
export IFACE=eno1
python go2/run_policy.py go2/policies/class_argon_walk.py --check     # expect: Policy OK
```

**B now stops softly while walking.** The robot goes to standing in 0.5 s, holds for 1 s, lies down, and sport mode comes back on by itself. If you need it to drop at once, press B a second time after letting go. B outside the walk, and Ctrl-C, still damp straight away. Try it once in the harness first: start a walk, press B after about 1 s, and check that the robot stands rather than falling forward. The result line should say `stopped with B (soft stop: stood, then lay down)`.

**Motor heat.** On 23 November the RR hip went from 31 °C to 71 °C in 12 minutes. Every run prints "hottest motor" before you type yes. If it is above about 60 °C, let the robot lie down for a few minutes first. That 60 °C is my suggestion, not a Unitree limit.

## 1. Measure the lidar mount (read-only, about 1 minute)

- **Robot:** standing still in sport mode on a flat floor, with nobody within 2 m.
- **Board:** a flat board, at least 0.6 m wide and 0.5 m tall, standing on the floor and square to the robot.
- **Position:** straight ahead and centred on the robot's nose line, so that it isn't off to one side.
- **Distance:** the board's face 1.00 m from the robot's centre. Measure on the floor, from the point midway between the front and rear feet.

```bash
python go2/calibrate.py --robot "$IFACE" --steps lidar
```

Press Enter when asked. Check these lines in its output:

| Line | Expected |
|---|---|
| `Fitted to:` | `rt/utlidar/cloud (frame 'utlidar_lidar')` |
| `Mount vs Unitree's Go2 model:` | a few degrees (2.5 on 23 November) |
| `Board:` | `seen at 0.9x–1.0x m` |
| `Probes: error` | each under 0.03 m |
| `object_dist:` | close to 1.0 m |

If it says `Board: no board found ...`, the mount is still saved, using Unitree's forward offset, and the runs below still work. Move the board straight ahead and run the command again only if you have time. Remove the board afterwards.

## 2. Stand only: is the env vector built with sport mode off?

```bash
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 0
python go2/lidar_report.py go2/logs/<time>.npz
```

- **Start-up line:** `Env: flat (...), lidar mount from the calibration, clouds of the last 0.4 s`. If step 1 was skipped, it says `fitted to the 2026-11-23 recording` instead.
- **Report:** in "Clouds per stage", `rt/utlidar/cloud` should show **usable** clouds in "standing up", "settling" and "lying down". On 23 November it showed 0 usable.

If they are still 0 usable, stop here and send the report output.

## 3. Walk in an open lane (the policy still gets flat floor)

```bash
# terminal 2, first:
nice -n 10 python go2/watch.py --robot "$IFACE"
# terminal 1:
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 4
python go2/lidar_report.py go2/logs/<time>.npz --replay 0,0.4,0.8
```

What to check in the report:

- **"While walking":** close to 100% of rebuilds with a fresh usable cloud. On 23 November it was 0%.
- **Probes:** probe0–2 near minus the body height (about −0.25 to −0.28).
- **object_dist:** mostly `-`. Things in the room 3–4 m ahead may show.
- **Control loop:** p99 about 3 ms, with almost nothing over 4 ms.

## 4. The same walk with a box 2.0 m ahead

- **Box:** at least 0.45 m tall, straight ahead.
- **Distance:** its face 2.0 m from the robot's centre.
- **Stopping:** the robot won't avoid it. At about 0.8 m/s, 2 s of walking covers about 1.6 m, so it stops short of the box by itself. Keep B ready anyway: one press stands it and lies it down.

```bash
# terminal 2:
nice -n 10 python go2/watch.py --robot "$IFACE"
# terminal 1:
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 2
python go2/lidar_report.py go2/logs/<time>.npz --replay 0,0.4,0.8
```

Expected: `object_dist, nearest per 0.5 s` falls over the walk, for example from about 2 m to under 1 m. "Object ahead" should appear on a good share of rebuilds, more with the 0.4 and 0.8 s windows than with 0.

## 5. Optional, only if steps 2–4 looked right: the policy gets the lidar's vector

Use the same box set-up as step 4:

```bash
# terminal 2:
nice -n 10 python go2/watch.py --robot "$IFACE"
# terminal 1:
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 2 --env-source lidar
python go2/lidar_report.py go2/logs/<time>.npz
```

- **Walking:** the robot should walk exactly as in step 4, because `class_argon_walk` ignores the env inputs.
- **Lidar stops:** if no usable cloud arrives for 0.5 s during the walk, the run stops and the motors go straight to damping. Keep a hand or the harness close.

## Afterwards: zip and copy back

On the robot computer:

```bash
cd /home/daniel/git/unitree_sdk2_python
D=$(date +%Y-%m-%d)
zip -r lidar_run2_$D.zip go2/logs/${D}_*.npz go2/logs/watch/${D}_*.npz \
    go2/logs/calibration/latest.json go2/logs/calibration/${D}_*
unzip -l lidar_run2_$D.zip | tail -3
```

Save the terminal output to `term_res2.txt` too, if you can.

On the development computer:

```bash
cd /Volumes/DEV/IRL/unitree_sdk2
scp "daniel@en4166249l:/home/daniel/git/unitree_sdk2_python/lidar_run2_*.zip" .
```

## Options you may want

| Option | What it does |
|---|---|
| `--lidar-window 0.8` | a longer accumulation window; `0` uses single clouds, as before |
| `--lidar-mount recovered` | always use the 23 November mount, whatever the calibration has |
| `--lidar-mount calibration` | use only the calibration's mount |
| `--no-lidar` | don't read the lidar |
