# Multi-UR Support Plan

Goal: extend the current single-arm (UR15) stack so that **N UR robots** can run side-by-side on this host — sharing what is genuinely shared (Redis status store, image-labeling, positioning, calibration, workflow-config, FFPP) while keeping per-robot pieces (driver, dashboard, camera, web UI, RViz) cleanly isolated.

This plan is concrete: it lists the exact files that need to change, the config schema, the launch-graph, the URI / port / topic conventions, and a four-stage migration that does not break the running UR15 stack.

---

## 1. Current state (single-arm baseline)

Inventory of what the system runs today, per `config/robot_config.yaml` and `robot_bringup/launch/ur15_bringup.py`:

| Component | Package | Per-robot or shared? | Notes |
|---|---|---|---|
| Redis status store + web dashboard | `robot_status_redis` | **shared** | Already namespaces by robot name (`robot_status:ur15:...`, `robot_status:duco:...`). |
| 3D positioning service | `positioning_3d_service` | **shared** | Stateless HTTP service on :8004. |
| Image labeling | `image_labeling_service` | **shared** | Stateless HTTP service on :8007. |
| Camera-calibration web | `camcalib_web_service` | **shared** | Stateless HTTP service on :8006. |
| Workflow config center | `ur_workflow` | **shared** (but named `ur15_*`) | Single web UI on :8008, currently UR15-only. |
| UR control / driver | `ur_robot_arm` → `ur_control.launch.py` | **per-robot** | Owns `/joint_states`, RTDE 30004, dashboard 29999. |
| Camera node | `camera_node/ur15_cam_launch.py` | **per-robot** | One RTSP stream per arm. |
| Robot web (dashboard) | `ur_web` | **per-robot** | Per-arm Flask UI on :8030; subscribes to that arm's camera + joint topics; talks URScript on 30002 to that arm only. |
| RViz visualization | `mock_ur_visualization` (new) | **per-robot** | `robot_state_publisher` + RViz preset; can also be shared by launching one RViz with two RobotModels. |

The `ConfigManager` already supports `get_robot(name)` and `list_robots()`, and the bringup launch is already configuration-driven (it reads `<robot>.launch_modules` and dispatches). That is the lever the multi-robot expansion pulls on.

---

## 2. Sharing matrix

What stays shared, what splits, and how the splits happen.

### 2.1 Stays shared (one process serves all arms)

| Service | Mechanism | Port |
|---|---|---|
| `robot_status_redis` | Already namespaces by robot. Each per-arm node writes to its own key prefix. | 8005 |
| `positioning_3d_service` | Stateless, called per-request with the dataset path. Sessions are per-call. | 8004 |
| `image_labeling_service` | Stateless. | 8007 |
| `camcalib_web_service` | Stateless. Operates on whatever calibration folder is passed. | 8006 |
| `workflow_config_center` | UI gains a "robot" selector dropdown that filters by the per-arm workflow folder. See §4. | 8008 |
| FFPP client | External, already shared via URL in `services.positioning_3d.ffpp_url`. | — |

### 2.2 Splits per arm (one process per robot)

| Component | What makes it per-arm | Naming convention |
|---|---|---|
| UR driver | Owns dashboard / URScript / RTDE sockets — only one client per arm per port. | Node name `ur_control_<name>`. |
| Camera node | Each arm has its own RTSP camera and image topic. | Topic `/<name>_camera/image_raw`, port `8019+i`. |
| Robot web UI | Per-arm Flask app + per-arm dashboard / freedrive / TCP controls. | Port `8030+i`. |
| Workflow runner | Per-arm because it sends URScript and reads per-arm joint state. | Subscribes to `/<name>_camera/image_raw` + `/<name>/joint_states`. |
| RViz model (when needed) | One URDF per arm, one `robot_state_publisher`. | Distinct `tf_prefix` per arm. |
| Joint state publisher | Each arm publishes to its own namespace. | `/<name>/joint_states`. |

### 2.3 Topic / TF conventions

Per-arm topics live under the arm's namespace:

```
/<name>/joint_states          sensor_msgs/JointState
/<name>/robot_description     std_msgs/String
/<name>_camera/image_raw      sensor_msgs/Image
/<name>_camera/take_snapshot  std_srvs/Trigger
```

TF: each arm uses `tf_prefix:=<name>_` so frames are `ur15_base_link`, `ur10e_base_link`, etc. Use one shared `world` frame as the common root and a static transform `world → <name>_base` per arm (publish via `static_transform_publisher` in the per-arm launch).

> The `tf_prefix` + `world` root is what lets a single RViz show all arms together without frame-name collisions.

---

## 3. Config schema

Today the YAML has a top-level `ur15:` block. Generalize to a list of robots, then keep a per-name lookup:

```yaml
version: "3.0"

# List of robots. The bringup iterates this list and dispatches one
# launch group per entry (see §5).
robots:
  - name: ur15                      # also used as ROS namespace
    type: ur15                      # ur_description xacro model
    enabled: true
    robot:
      ip: 192.168.1.15
      ports: { control: 30002, dashboard: 29999, rtde: 30004 }
      launch_rviz: false
    camera:
      name: UR15Camera
      ip: 192.168.1.101
      rtsp_url: rtsp://admin:123456@192.168.1.101/stream0
      server_port: 8019
      topic: /ur15_camera/image_raw
    web:
      port: 8030
      camera_topic: /ur15_camera/image_raw
    world_tf:                       # static transform world -> <name>_base
      xyz: [0, 0, 0]
      rpy: [0, 0, 0]
    launch_modules:                 # which per-robot modules to launch
      - { package: ur_robot_arm, launch_file: ur15_control_launch.py }
      - { package: camera_node,    launch_file: ur15_cam_launch.py }
      - { package: ur_web,       launch_file: ur15_web_launch.py, delay: 5.0 }

  - name: ur10e_mock                # the URSim at .16
    type: ur10e
    enabled: true
    robot:
      ip: 192.168.1.16
      ports: { control: 30002, dashboard: 29999, rtde: 30004 }
    camera:
      name: UR10eMockCamera
      ip: 192.168.1.103
      rtsp_url: rtsp://admin:123456@192.168.1.103/stream0
      server_port: 8020
      topic: /ur10e_mock_camera/image_raw
    web:
      port: 8031
      camera_topic: /ur10e_mock_camera/image_raw
    world_tf:
      xyz: [1.2, 0, 0]              # 1.2 m to the right of UR15
      rpy: [0, 0, 0]
    launch_modules:
      - { package: ur_robot_arm, launch_file: ur15_control_launch.py }
      - { package: camera_node,    launch_file: ur15_cam_launch.py }
      - { package: ur_web,       launch_file: ur15_web_launch.py, delay: 5.0 }

# Singleton services (one process for the whole host).
services:
  robot_status_redis: { web: { port: 8005 } }
  positioning_3d:     { port: 8004 }
  image_labeling:     { port: 8007 }
  camcalib_web:       { port: 8006 }
  workflow_config:    { port: 8008 }
```

The top-level `services:` block already exists; only the per-robot section moves from `ur15:` → list-of-robots.

`ConfigManager` API additions (small, backward-compatible):

```python
config.list_robots()                              # already exists
config.get_robot(name)                            # already exists; route to robots[*]
config.get_service('robot_status_redis')          # new convenience
```

A short shim keeps the old top-level `ur15.*` key reads working during the migration (return the matching entry from `robots:` when someone asks for `ur15`).

---

## 4. Workflow Config Center → multi-robot

Today `ur_workflow/web/workflow_api.py` opens one workflow folder and accepts `--ur15-ip 192.168.1.16 --ur15-port 30002`. To go multi-arm:

1. Rename the package's data folder layout from `temp/workflow_files/` to `temp/workflow_files/<robot_name>/`. Pre-existing files migrate by `mv temp/workflow_files temp/workflow_files.ur15 && mkdir temp/workflow_files && mv temp/workflow_files.ur15 temp/workflow_files/ur15`.
2. The Flask app now takes `--robots ur15:192.168.1.15:30002,ur10e_mock:192.168.1.16:30002` (or reads it directly from the config file).
3. The UI gains a robot dropdown at the top. The chosen robot scopes the workflow folder, the URScript socket, and the joint-state subscription.
4. Service stays a single process on :8008 — only the data and the active target switch per request.

Rename the ROS package directory itself from `ur_workflow` to `workflow_config_center` to drop the misleading prefix. Same for `ur_web` → `robot_web` (a per-arm web UI parameterized by `robot_name`).

> **Why rename now**: tomorrow's user reading `ur_web` for the UR10e arm will be confused. A flat rename + `ament_index` cleanup is cheaper than carrying the `ur15_` name in code that's no longer UR15-specific.

---

## 5. Launch graph

### 5.1 Top-level: `robot_bringup/launch/multi_robot_bringup.py` (new)

Replaces `ur15_bringup.py` over time. Pseudocode:

```python
def generate_launch_description():
    cfg = ConfigManager()
    actions = []

    # 1. Shared services — start once.
    for svc in ['robot_status_redis', 'positioning_3d_service',
                'image_labeling_service', 'camcalib_web_service',
                'workflow_config_center']:
        actions.append(ExecuteProcess(cmd=['ros2', 'launch', svc, f'{svc}_launch.py']))

    # 2. Per-robot — iterate the list.
    for robot in cfg.list_robots():
        rcfg = cfg.get_robot(robot)
        if not rcfg.get('enabled', True): continue
        for mod in rcfg.get('launch_modules', []):
            actions.append(TimerAction(
                period=mod.get('delay', 0.0),
                actions=[ExecuteProcess(cmd=[
                    'ros2', 'launch', mod['package'], mod['launch_file'],
                    f'robot_name:={robot}',                       # <-- KEY
                ])],
            ))
    return LaunchDescription(actions)
```

Backward-compat: keep `ur15_bringup.py` as a thin wrapper that filters the list to just `ur15` before iterating.

### 5.2 Per-arm launch files take `robot_name`

Every per-robot launch file gains a single `robot_name` `LaunchConfiguration` and reads the rest from `config.get_robot(robot_name)`. Example for `ur_robot_arm/launch/ur15_control_launch.py`:

```python
robot_name = LaunchConfiguration('robot_name')
def _build(context):
    name = robot_name.perform(context)
    rcfg = ConfigManager().get_robot(name)
    return [IncludeLaunchDescription(
        ur_control_launch_py,
        launch_arguments={
            'ur_type': rcfg.get('type'),
            'robot_ip': rcfg.get('robot.ip'),
            'tf_prefix': f'{name}_',
            'controller_namespace': name,
            'launch_rviz': 'false',
        }.items(),
    )]
return LaunchDescription([
    DeclareLaunchArgument('robot_name', default_value='ur15'),
    OpaqueFunction(function=_build),
])
```

`ur_control.launch.py` from `ur_robot_driver` already accepts `tf_prefix` and `controller_namespace`, so no upstream change is needed.

The same pattern applies to:
- `camera_node/launch/<old_name>_cam_launch.py` → `camera_launch.py`, takes `robot_name`.
- `ur_web/launch/ur15_web_launch.py` → `robot_web_launch.py`, takes `robot_name`.
- `mock_ur_visualization/launch/visualize.launch.py` — already supports `joint_prefix`/`tf_prefix`; add a `robot_name` arg that fills both from config.

### 5.3 Static world transform

Inside each per-arm launch, add:

```python
Node(package='tf2_ros', executable='static_transform_publisher',
     arguments=['--frame-id', 'world',
                '--child-frame-id', f'{name}_base',
                '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2])])
```

xyz/rpy come from `world_tf` in the robot's config. RViz's Fixed Frame becomes `world`.

---

## 6. Port allocation

Predictable, no collisions. Index `i = 0..N-1`:

| Service | Port | Notes |
|---|---|---|
| Singleton services | 8004, 8005, 8006, 8007, 8008 | unchanged |
| Per-arm camera HTTP | 8019 + i | 8019 (UR15), 8020 (UR10e mock), … |
| Per-arm web UI | 8030 + i | 8030, 8031, … |
| URScript / dashboard / RTDE | fixed at 29999 / 30001-30004 | unique per **IP**, not per host |

Camera and web ports are read from each robot's `camera.server_port` / `web.port`, so allocation lives in YAML, not in code.

---

## 7. Per-arm `robot_status_redis` keys

The package already accepts a namespace. Audit and fix any hard-coded `'ur15'` callers:

```bash
grep -rn "'ur15'" colcon_ws/src/ | grep -v test
```

The expected pattern is `set_to_status(robot_name, key, value)` everywhere. Pass `robot_name` (and its inverse for readers) through the Node parameters declared in §5.2.

---

## 8. RViz: one viewer for all arms

Two valid options:

- **One RViz per arm** (simpler, default). Just launch `mock_ur_visualization` once per enabled robot with `robot_name:=ur15`, etc. Each picks up its own URDF and joint poll.
- **One RViz showing all arms** (recommended for ops). Build a fan-in launch in `mock_ur_visualization` that, given a list of robot names, starts one `robot_state_publisher` and one `joint_publisher` per arm (each with its own `tf_prefix`), then a single RViz with one `RobotModel` display per `/<name>/robot_description`. Save the multi-arm RViz config as `config/multi_robot.rviz`.

Either way the `tf_prefix` work from §5.2 / §5.3 is the prerequisite — without it the two URDFs collide on frame names and RViz flickers (the exact failure mode we already hit and fixed for the single mock case).

---

## 9. Migration — four stages, each independently shippable

Each stage is small, leaves the system bootable, and can be reverted on its own.

### Stage A — config + ConfigManager (no behaviour change)

1. Add a `robots:` list to `config/robot_config.yaml` containing one entry that mirrors the current `ur15:` block. Keep `ur15:` in place for now.
2. Teach `ConfigManager.get_robot('ur15')` to look in either location and prefer `robots:` when both exist.
3. Add `ConfigManager.list_robots_v2()` returning the new list (the old `list_robots()` keeps returning top-level keys minus `version`/`shared` until Stage D).
4. Add unit tests in `colcon_ws/src/common/test/test_config_manager.py`.

Verification: `ur15_bringup.py` still launches everything unchanged.

### Stage B — parameterize the per-arm launch files

For each of `ur_robot_arm`, `camera_node`, `ur_web`, `ur_workflow`:

1. Add `robot_name` `LaunchConfiguration` (default `'ur15'`).
2. Rewrite the launch body to fetch its values from `config.get_robot(robot_name)`.
3. Pass `tf_prefix:=<name>_` and `controller_namespace:=<name>` to `ur_control.launch.py`.
4. Where nodes are spawned, pass `robot_name` as a Node parameter so the runtime code can namespace topics + redis keys correctly.

Verification: launching `ros2 launch ur_robot_arm ur15_control_launch.py robot_name:=ur15` still drives the real UR15 exactly as today.

### Stage C — multi-robot bringup + second arm

1. Add `robot_bringup/launch/multi_robot_bringup.py` as in §5.1.
2. Add a second entry to `robots:` for the mock UR10e at `192.168.1.16`.
3. Launch the mock arm side by side: confirm `/ur15/joint_states` + `/ur10e_mock/joint_states` both publish, `tf` resolves with prefixes, RViz shows both arms.
4. Iterate on `world_tf` until the two arms are placed correctly in the scene.

### Stage D — rename + cleanup (after second arm runs green)

1. Rename packages: `ur_workflow` → `workflow_config_center`, `ur_web` → `robot_web`. Keep the old package names as `setup.cfg` `aliases` for a release or two so external scripts don't break.
2. Drop the legacy top-level `ur15:` block in `robot_config.yaml`; only the `robots:` list remains. Remove the back-compat shim from `ConfigManager`.
3. `ur15_bringup.py` is removed; everything goes through `multi_robot_bringup.py`.
4. Update [doc/mock_ur_robot.md](mock_ur_robot.md) and [doc/lift_robot_system_guide.md](lift_robot_system_guide.md) with the new launch invocation.

---

## 10. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Two `robot_state_publisher`s on the same `/tf` (RViz flicker — we hit this with the mock). | `tf_prefix` per arm, one RSP per arm, single `world` root. Section §5.3. |
| Two ROS nodes with the same name (`ur15_web_node`, etc.). | Node names take `robot_name` suffix (`<name>_web_node`). Already partly there in `mock_ur_visualization`. |
| Two clients hammering UR dashboard 29999 of the same robot. | Per-arm processes target *different* IPs; never two clients per IP. Add a runtime guard in `UR15Robot.open()` that warns if it sees an existing connection. |
| `robot_status_redis` key collision. | Namespace by `robot_name` everywhere; CI test in `test_config_manager.py` asserts no caller passes a literal `'ur15'`. |
| RTSP cameras share the same default `192.168.1.10x`. | Per-arm `camera.ip` in YAML — the schema already supports it; the migration just stops hard-coding `.101` in launch files. |
| Web UI port collision (8030). | `camera.server_port` and `web.port` are per-robot YAML keys — the migration is just plumbing them through. |
| `ur15_bringup.py` is in muscle-memory. | Keep it during stages A–C as a one-line wrapper. Only delete in stage D. |

---

## 11. Out of scope (do later)

- MoveIt 2 multi-arm planning. The driver + URDF parts of this plan are what MoveIt needs anyway, so adding MoveIt later is additive.
- Shared end-effector / tool library. Today `ur15.tool.*` is single-arm; turning into `robots[*].tool` happens for free in Stage A.
- A unified web dashboard that shows all arms in one page. The shared Redis store + per-arm web UIs already give 80% of this; a single aggregation page is one frontend change, not a backend change.
- Sim / hardware switching via a single flag. URSim already looks like hardware, so this is already covered by changing `robot.ip`.

---

## 12. Definition of done

1. `ros2 launch robot_bringup multi_robot_bringup.py` brings up the real UR15 (192.168.1.15) **and** the URSim UR10e (192.168.1.16) **and** shared services, with no port / topic / Redis-key collisions.
2. One RViz instance shows both arms in correct relative position via the `world` frame.
3. Killing the UR10e mock leaves UR15 untouched and vice-versa (verified with `docker stop` on URSim + `pkill ur_ros2_control_node` on the UR10e launch).
4. `config/robot_config.yaml` is the only file that needs editing to add or remove an arm.
5. All grep results for `'ur15'` in non-test code resolve to a `robot_name` variable, not a string literal.

This is the plan to execute when you're ready to start Stage A.
