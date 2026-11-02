# Go2 setup on the robot computer

Follow these steps on the Ubuntu computer wired to the robot. They end with a read-only check that sends no commands to the robot.

**Don't run other scripts yet.** Unitree's examples can move the robot as soon as you give them a network interface. `stand_test.py` only works with the simulator, so it can't reach the robot.

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
