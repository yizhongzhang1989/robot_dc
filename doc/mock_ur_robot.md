# Mock UR Robot (URSim)

URSim is the Polyscope controller packaged as a Docker container. Over the network it is indistinguishable from a real UR — dashboard `29999`, URScript `30001/30002`, realtime `30003`, RTDE `30004`. Use it to develop and test when no physical arm is available, or to stand up extra robots alongside the real UR15.

> Not simulated: cameras, force sensors, vision pipeline. Only the robot side.

---

## 1. Prerequisites (one-time)

- Ubuntu 22.04 + ROS 2 Humble.
- UR debs: `sudo apt install -y ros-humble-ur-client-library ros-humble-ur-robot-driver ros-humble-ur-description`
- The setup script handles Docker, the image pull, the IP alias, and the URCap staging on first run.

Quick check:

```bash
ls /opt/ros/humble/lib/ur_client_library/start_ursim.sh   # must exist
```

---

## 2. Start / stop

```bash
# Start one URSim. The container is named ursim_<model>_<last-octet>.
sudo bash scripts/setup_ursim.sh --ip 192.168.1.16 --model ur10e

# Start a second one in parallel (different IP).
sudo bash scripts/setup_ursim.sh --ip 192.168.1.17 --model ur15

# Tear down — pick one form.
sudo bash scripts/teardown_ursim.sh --ip   192.168.1.16
sudo bash scripts/teardown_ursim.sh --name ursim_ur10e_16
sudo bash scripts/teardown_ursim.sh --all
```

Supported models: `ur3/5/10` (CB3), `ur3e/5e/7e/10e/12e/16e`, `ur8long`, `ur15/18/20/30` (e-series).

The container is restartable across stop/start without re-running setup:

```bash
docker stop  ursim_ur10e_16
docker start ursim_ur10e_16
docker logs -f ursim_ur10e_16
```

> The `<IP>/32` alias on `eno1` is **non-persistent** — gone after reboot. Re-run `setup_ursim.sh` after a reboot.

---

## 3. Power on the simulated arm

URSim boots in `POWER_OFF` with brakes engaged. Either:

- **GUI**: open `http://<IP>:6080/vnc.html`, click POWER_OFF at the bottom-left → **ON** → **Start**.
- **Headless** (scriptable):

```bash
python3 - <<'PY'
import socket, time
def cmd(s,c,t=1.0): s.sendall(c.encode()+b"\n"); time.sleep(t); return s.recv(4096).decode().strip()
s = socket.socket(); s.connect(("192.168.1.16", 29999)); s.recv(4096)
print(cmd(s, "power on", t=2.0))
while "IDLE" not in cmd(s, "robotmode", t=0.5): time.sleep(1)
print(cmd(s, "brake release", t=3.0))
while "RUNNING" not in cmd(s, "robotmode", t=0.5): time.sleep(1)
print("ready")
PY
```

> Polyscope cold-boot is **60–120 s**. The dashboard accepts TCP connections almost immediately but doesn't answer commands until Polyscope is fully up. Poll for a real `Robotmode:` reply, never trust a fixed sleep.

---

## 4. Verify it works

```bash
cd /home/robot/Documents/robot_dc/colcon_ws && source install/setup.bash
python3 - <<'PY'
from ur_robot_arm.ur15 import UR15Robot
import time
r = UR15Robot("192.168.1.16", 30002); r.open()
r.popup("Hello from URSim", title="Mock Robot")
before = r.get_actual_joint_positions()
target = list(before); target[0] += 0.2
r.movej(target, a=0.5, v=0.3, blocking=False); time.sleep(4)
after = r.get_actual_joint_positions()
print("delta j0:", round(after[0]-before[0], 3), "(expect ~0.2)")
r.movej(before, a=0.5, v=0.3, blocking=False); time.sleep(4)
r.close()
PY
```

You should see the popup on the Polyscope teach pendant and `delta j0: 0.2` printed.

---

## 5. Visualize the mock in RViz

Drive RViz from the same URScript port `UR15Robot` uses — no `ur_robot_driver` needed (avoids RTDE port-30004 contention with the rest of this repo's stack).

Build once:

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --symlink-install --packages-select mock_ur_visualization
source install/setup.bash
```

Then launch one of:

```bash
# Standalone — URSim mock at .16 (UR10e)
ros2 launch mock_ur_visualization visualize.launch.py \
    robot_ip:=192.168.1.16 ur_type:=ur10e

# Standalone — real UR15 at .15
ros2 launch mock_ur_visualization visualize.launch.py \
    robot_ip:=192.168.1.15 ur_type:=ur15

# Alongside ur15_bringup (driver already publishes /robot_description, /tf
# and /joint_states — skip ours to avoid the two-models flicker)
ros2 launch mock_ur_visualization visualize.launch.py \
    use_robot_state_publisher:=false use_joint_publisher:=false
```

Re-run the `movej` from §4 in a second terminal and the arm in RViz tracks the motion live. Full reference: [colcon_ws/src/mock_ur_visualization/README.md](../colcon_ws/src/mock_ur_visualization/README.md).

---

## 6. Use it from the project

### 6.1 Drop-in IP replacement

Edit [config/robot_config.yaml](../config/robot_config.yaml) so everything in `colcon_ws/src/ur15_*` and `scripts/ur_*` targets the mock — no code changes needed:

```yaml
ur15:
  robot:
    ip: "192.168.1.16"      # was 192.168.1.15
```

Revert when the real UR15 is back.

### 6.2 Side-by-side (multi-robot)

Keep the real UR15 on `.15` and run a UR10e mock on `.16`. Both speak the same protocol on the same ports at different IPs — no conflict.

### 6.3 Full `ur_robot_driver` (ROS 2 control pipeline)

Needed only when you want `ros2_control` controllers (e.g. `joint_trajectory_controller`). Requires the External Control URCap to be enabled in Polyscope and a program running it.

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e robot_ip:=192.168.1.16 launch_rviz:=true
```

For URScript / dashboard / RTDE work alone, this stack is **not** required.

---

## 7. Common gotchas

- **`get robot model` returns `UR10` for a UR10e simulator.** URSim quirk; the kinematics are correct. Don't branch on this string.
- **`is in remote control -> false` after boot.** Direct URScript on 30002 still works. To use dashboard `play` you must enable Remote Control in Polyscope (Hamburger → Settings → System → Remote Control).
- **Two `robot_state_publisher`s = RViz flicker.** If `ur15_bringup` is running, launch `mock_ur_visualization` with `use_robot_state_publisher:=false use_joint_publisher:=false`.
- **`curl http://<IP>:29999/` fails.** Dashboard is not HTTP; it's a plain-text line protocol. Use `nc`, `bash`'s `/dev/tcp`, or a Python socket. The banner `Connected: Universal Robots Dashboard Server` is what fools `curl` into reporting `HTTP/0.9`.
- **Polyscope GUI ports `5900` / `6080` need explicit forwarding.** Handled by `scripts/setup_ursim.sh`; vanilla `start_ursim.sh` forwards only `29999` + `30001-30004`.
- **`docker ps` says permission denied right after first run.** Log out and back in, run `newgrp docker`, or prefix with `sudo` until you re-login.
- **Last octet of the LAN IP must be `2–254`.** The script derives the docker-internal IP from it.
- **`192.168.56.101` always works without any LAN alias.** That is the docker-network IP of the first container. The `192.168.1.16` alias only exists so the IP matches what the real UR10e would use.

---

## 8. Endpoints

| Port | Service | Notes |
|---|---|---|
| 29999 | Dashboard | UR Dashboard protocol (not HTTP). |
| 30001 | Secondary URScript | RTSI / secondary client interface. |
| 30002 | Primary URScript | What `UR15Robot` uses for `movej`, `popup`, `textmsg`, … |
| 30003 | Realtime data | 1 ms cycle joint / TCP / force readback. |
| 30004 | RTDE | `ur_robot_driver` controller pipeline. |
| 5900 | VNC | Native client (e.g. Remmina). |
| 6080 | noVNC | Polyscope GUI in a browser: `http://<IP>:6080/vnc.html`. |

---

## 9. References

- URSim helper: `/opt/ros/humble/lib/ur_client_library/start_ursim.sh`
- URSim image: <https://hub.docker.com/r/universalrobots/ursim_e-series>
- UR Dashboard protocol: <https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/>
- `ur_robot_driver`: <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/>
- Setup scripts: [scripts/setup_ursim.sh](../scripts/setup_ursim.sh), [scripts/teardown_ursim.sh](../scripts/teardown_ursim.sh)
