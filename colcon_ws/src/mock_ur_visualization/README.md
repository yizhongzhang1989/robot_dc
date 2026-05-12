# mock_ur_visualization

Lightweight RViz visualization for a UR robot, driven by direct **URScript polling** instead of `ur_robot_driver` / `ros2_control`.

It is intended for the workflow used in this repo, where the arm is commanded
via [`ur_robot_arm.ur15.UR15Robot`](../ur_robot_arm/ur_robot_arm/ur15.py)
(direct primary-port URScript on TCP 30002). RViz just needs joint angles —
we read them from the same URScript socket and republish as `JointState`.

## What it starts

| Node | Purpose |
| ---- | ------- |
| `joint_publisher` | polls the UR primary URScript port at 30 Hz, publishes `sensor_msgs/JointState` on `/joint_states` |
| `robot_state_publisher` | uses `ur_description`'s xacro to publish `/tf` from `/joint_states` |
| `rviz2` | preconfigured scene: Grid + TF + RobotModel (reads `/robot_description`) |

No `ur_robot_driver`, no `dashboard_client`, no External Control program required.

## Build

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --symlink-install --packages-select mock_ur_visualization
source install/setup.bash
```

## Launch

```bash
# Default: UR10e at 192.168.1.16 (the URSim mock used in this repo)
ros2 launch mock_ur_visualization visualize.launch.py

# Real UR15 at 192.168.1.15
ros2 launch mock_ur_visualization visualize.launch.py \
    robot_ip:=192.168.1.15 ur_type:=ur15

# Different model / port / rate
ros2 launch mock_ur_visualization visualize.launch.py \
    robot_ip:=192.168.1.17 ur_type:=ur5e rate_hz:=50.0
```

## Arguments

| Arg | Default | Description |
| --- | ------- | ----------- |
| `robot_ip` | `192.168.1.16` | IP of the robot (real or URSim) |
| `port` | `30002` | URScript primary port |
| `ur_type` | `ur10e` | UR model used to select the URDF (one of `ur3/5/10[e]`, `ur7e/12e/16e`, `ur8long`, `ur15/18/20/30`) |
| `rate_hz` | `30.0` | JointState publish rate |
| `rviz` | `true` | Set `false` to skip launching RViz |
| `joint_prefix` | `""` | Optional prefix applied to both joint names and `tf_prefix` |
| `use_robot_state_publisher` | `true` | Set `false` when another launch (e.g. `ur_robot_driver` via `ur15_bringup`) is already publishing `/robot_description` and `/tf` — avoids the duplicate-RSP flicker |
| `use_joint_publisher` | `true` | Set `false` when `ur_robot_driver` / `ros2_control` is already publishing `/joint_states` |

### Running alongside `ur15_bringup` (or any `ur_robot_driver`-based stack)

If you already have a stack publishing `/robot_description`, `/tf` and `/joint_states`, only RViz is missing. Skip our publishers to avoid duplicates:

```bash
ros2 launch mock_ur_visualization visualize.launch.py \
    use_robot_state_publisher:=false \
    use_joint_publisher:=false
```

Running with both `true` while another RSP is alive causes a **two-models flicker** in RViz (each RSP publishes slightly different transforms for the same frames — TF resolves to whichever arrived last).

## Verify it works

In one terminal, launch this package as above. In a second terminal, send a
small motion via URScript:

```bash
source ~/Documents/robot_dc/colcon_ws/install/setup.bash
python3 - <<'PY'
from ur_robot_arm.ur15 import UR15Robot
r = UR15Robot('192.168.1.16', 30002); r.open()
q = list(r.get_actual_joint_positions())
print('start', q)
q[0] += 0.4; r.movej(q, a=1.0, v=0.5)
q[0] -= 0.4; r.movej(q, a=1.0, v=0.5)
r.close()
PY
```

The arm in RViz should swing the shoulder-pan joint by ±0.4 rad live.

## Why not use `ur_robot_driver`?

In this repo we already command the UR arms via direct URScript. Running
`ur_robot_driver` alongside causes RTDE port contention (port 30004) and
duplicate-client mayhem — symptoms include "RViz frozen" with `/joint_states`
publishing zeros. This package sidesteps that completely: one reader, one
publisher, one source of truth.
