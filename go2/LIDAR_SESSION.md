# Lidar session: commands from start to finish

This session measures the Go2's lidar, records it during normal runs, and then gives the policy the lidar's env vector. Steps 1 and 2 don't walk the robot; steps 3–5 do. Background: [reports/2026-11-20_next_steps.md](reports/2026-11-20_next_steps.md).

## 0. Get the code onto the robot computer

Pull the branch on the robot computer, or copy the files across. The repository on the robot computer is at `/home/daniel/git/unitree_sdk2_python`. To copy from the development computer:

```bash
cd unitree_sdk2
scp go2/lidar_env.py go2/lidar_worker.py go2/lidar_calibrate.py go2/lidar_report.py \
    go2/run_policy.py go2/calibrate.py go2/README.md go2/ROBOT_SESSION.md go2/LIDAR_SESSION.md \
    daniel@<robot-computer>:/home/daniel/git/unitree_sdk2_python/go2/
scp go2/policies/class_argon_walk.py daniel@<robot-computer>:/home/daniel/git/unitree_sdk2_python/go2/policies/
```

## Set-up on the robot computer (both terminals)

```bash
cd /home/daniel/git/unitree_sdk2_python
export IFACE=eno1
python go2/run_policy.py go2/policies/class_argon_walk.py --check     # expect: Policy OK
```

If sport mode is off, restart the robot. Step 1 needs sport mode on.

## 1. Measure the lidar (read-only, about 1 minute)

- **Robot:** standing still in sport mode on a flat floor, with nobody within 2 m.
- **Board:** a flat board at least 0.6 m wide and 0.5 m tall, standing square to the robot.
- **Distance:** the board's face 1.00 m from the robot's centre. Measure on the floor, from the point midway between the front and rear feet.

```bash
python go2/calibrate.py --robot "$IFACE" --steps lidar
```

Press Enter when asked. A good result shows:
- at least one cloud topic with messages
- `measured` all `True`
- probe error under 0.03 m
- `object_dist` close to 1.0 m
- the nearest floor seen ahead under 0.30 m

The mount it measures goes into `go2/logs/calibration/latest.json`. Remove the board afterwards.

## 2. Stand only: does the lidar keep publishing with sport mode off?

```bash
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 0
python go2/lidar_report.py go2/logs/<time>.npz
```

In the report's "Clouds per stage" table, the rows "standing up", "settling" and "lying down" should show clouds. **If all three show 0, skip steps 4 and 5**, but still run step 3.

## 3. Walk in an open lane, with the lidar recorded

The policy still gets flat-floor values, as before. Start the watcher first:

```bash
# terminal 2, first:
nice -n 10 python go2/watch.py --robot "$IFACE"
# terminal 1:
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 10
python go2/lidar_report.py go2/logs/<time>.npz
```

Check the report's control-loop line. The p99 cycle should be about 3 ms or less, with almost no cycles over 4 ms, as in the 13:46 run on 20 November. If it is clearly worse, rerun with `--no-lidar` and report it.

## 4. The same walk, with a box about 2.5 m ahead

The box must be **at least 0.45 m tall**. A lower box passes under the obstacle ray, in the simulator as well. The robot won't avoid the box, so keep B ready.

```bash
# terminal 2:
nice -n 10 python go2/watch.py --robot "$IFACE"
# terminal 1:
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 4
python go2/lidar_report.py go2/logs/<time>.npz
```

Expected: "object ahead" on most rebuilds, with the nearest distance dropping toward 1–1.5 m.

## 5. Optional, only if steps 2–4 looked right: give the policy the lidar's vector

```bash
# terminal 2:
nice -n 10 python go2/watch.py --robot "$IFACE"
# terminal 1:
python go2/run_policy.py go2/policies/class_argon_walk.py --robot "$IFACE" --policy-hz 10 --seconds 4 --env-source lidar
```

The robot should walk exactly as in step 4, because this policy ignores the env inputs. **If the lidar stops for 0.5 s during the walk, the run stops and the motors go straight to damping**, so keep a hand or the harness close.

## Afterwards

Zip these and put the zip in the repository root:
- `go2/logs/calibration/<newest folder>/`
- each run's `go2/logs/<time>.npz` and `go2/logs/<time>_lidar.npz`
- `go2/logs/watch/`
