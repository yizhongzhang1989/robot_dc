# UR15 Robot Manual

This document covers the configuration, launch, and operation of the UR15 robot arm system within the Robot DC platform.

---

## 1. System Overview

The UR15 subsystem includes:

| Component | Package | Description | Dashboard |
|-----------|---------|-------------|-----------|
| Robot driver | `ur_robot_arm` | UR robot driver + arm control node | — |
| Camera | `camera_node` | RTSP capture → ROS 2 image + MJPEG stream | :8019 |
| Web dashboard | `ur_web` | Main control & calibration UI | :8030 |
| Workflow engine | `ur_workflow` | JSON-based task sequencing | :8008 |
| 3D positioning | `positioning_3d_service` | Multi-view triangulation via FlowFormer++ | :8004 |
| Image labeling | `image_labeling_service` | Web-based keypoint annotation | :8007 |
| Camera calibration | `camcalib_web_service` | Intrinsic & eye-in-hand calibration | :8006 |
| Status store | `robot_status_redis` | Redis-backed status management | :8005 |

---

## 2. Configuration

All UR15 settings are in `config/robot_config.yaml`. Key items to update:

- **`ur15.robot.ip`** — UR15 robot IP address (default: `192.168.1.15`). The robot must be in **Remote Control mode**.
- **`ur15.launch_modules`** — List of modules launched by `ur15_bringup.py`. Set `enabled: false` on any module to skip it (e.g., if hardware is not connected).
- **`ur15.camera.ip`** / **`ur15.camera.rtsp_url`** — IP camera address and RTSP stream URL. This is configured for the IP camera used in the Beijing Lab. If you are not using this type of camera, disable the `camera_node` module in `launch_modules`. Other modules that need an image feed can subscribe directly to the ROS 2 topic (`/ur15_camera/image_raw`) from any image source.
- **`services.positioning_3d.ffpp_url`** — FlowFormer++ server URL for 3D positioning.

See `config/robot_config.example.yaml` for the full reference with all available settings (tool definitions, calibration parameters, web ports, etc.).

---

## 3. Preparation

### 3.1 Prepare Chessboard Configuration

A chessboard (or ChArUco board) is needed for camera calibration. After the first launch of the UR15 system, the directory `temp/ur15_cam_calibration_data` is automatically created.

To generate the chessboard configuration:

1. Launch the system: `ros2 launch robot_bringup ur15_bringup.py`
2. Open the Camera Calibration Web at `http://<host>:8006`
3. Go to the **Pattern Generator** page
4. Configure the pattern to match the physical chessboard you are using (rows, columns, square size, etc.)
5. Download the generated JSON configuration file
6. Place it at `temp/ur15_cam_calibration_data/chessboard_config.json`

This file is referenced by `ur15.web.chessboard_config_path` in `robot_config.yaml` and is used during calibration data collection and processing. The configuration will be loaded automatically the next time you launch the UR15 system.

---

## 4. Launching

### 4.1 Full System Launch

Start all enabled UR15 modules:

```bash
ros2 launch robot_bringup ur15_bringup.py
```

This sequentially launches each module from `ur15.launch_modules`. The `ur_web` module has a 5-second delay to allow the camera node to initialize first.

### 4.2 Individual Module Launch

Launch specific modules independently:

```bash
# Robot driver only
ros2 launch ur_robot_arm ur15_control_launch.py

# Robot driver + arm node
ros2 launch ur_robot_arm ur_robot_arm.launch.py

# Robot driver + arm node + joystick
ros2 launch ur_robot_arm ur15_robot_arm_with_joy.launch.py

# Camera node
ros2 launch camera_node ur15_cam_launch.py

# Web interface
ros2 launch ur_web ur15_web_launch.py
```

### 4.3 Disabling Modules

To skip a module (e.g., when hardware is not connected), set `enabled: false` in `robot_config.yaml`:

```yaml
    - package: "ur_robot_arm"
      launch_file: "ur15_control_launch.py"
      delay: 0.0
      enabled: false  # skip if UR15 not connected
```

---

## 5. Web Interfaces

After launch, the following web interfaces are available:

| URL | Purpose |
|-----|---------|
| `http://<host>:8030` | UR15 main dashboard — camera feed, joint states, TCP pose, freedrive control, calibration validation |
| `http://<host>:8019` | Camera MJPEG live stream |
| `http://<host>:8005` | Robot status dashboard (Redis) |
| `http://<host>:8004` | 3D positioning service |
| `http://<host>:8006` | Camera calibration web tool |
| `http://<host>:8007` | Image labeling / keypoint annotation |
| `http://<host>:8008` | Workflow config center (CRUD API) |

---

## 6. ROS 2 Topics and Services

### Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/ur15_camera/image_raw` | `sensor_msgs/Image` | `camera_node` | Camera frames |
| `/joint_states` | `sensor_msgs/JointState` | `ur_robot_driver` | Joint positions/velocities |
| `/cartesian_motion_controller/current_pose` | `geometry_msgs/PoseStamped` | `ur_robot_driver` | Current TCP pose |
| `/target_frame` | `geometry_msgs/PoseStamped` | `ur15_robot_arm_node` | Target pose commands |
| `/ur_robot_arm/status` | `std_msgs/String` | `ur15_robot_arm_node` | Robot status messages |

### Services (robot_status_redis)

| Service | Description |
|---------|-------------|
| `SetStatus` | Store a status value |
| `GetStatus` | Retrieve a status value |
| `ListStatus` | List all status keys |
| `DeleteStatus` | Delete a status entry |

---

## 7. Robot Control API

The `URRobot` class (`ur_robot_arm.ur_robot`) provides direct socket-based control:

```python
from ur_robot_arm.ur_robot import URRobot

robot = URRobot()
robot.open("192.168.1.15", 30002)

# Joint-space move (6 joint angles in radians)
robot.movej([1.11, -1.02, 2.55, -2.84, -0.71, -1.95], a=0.5, v=0.5)

# Linear TCP move (x, y, z, rx, ry, rz)
robot.movel([0.5, 0.2, 0.3, 0.0, 3.14, 0.0], a=0.5, v=0.1)

# TCP offset move
robot.move_tcp([0.0, 0.0, -0.05, 0.0, 0.0, 0.0], a=0.5, v=0.1)

# Read current TCP pose
pose = robot.get_actual_tcp_pose()

robot.close()
```

Commands are sent as URScript over TCP, formatted as `def myProg():\n  {command}\nend\n`.

---

## 8. Dashboard Control

Use the dashboard interface (port 29999) for robot state management:

```bash
python3 scripts/ur_dashboard_control.py
```

Operations available:
- Power on/off
- Brake release
- Program play/stop/pause
- Robot mode and safety status queries

---

## 9. Camera Calibration

For the full hand-eye calibration process (image collection, auto capture, calibration, report review, and validation), see the dedicated guide: [Hand-Eye Calibration Guide](handeye_calibration.md).

Summary of steps:
1. Manually capture at least 30 chessboard images from varied angles via the web dashboard at `:8030`
2. Run auto capture to re-take images without hand shake
3. Calibrate (intrinsic + hand-eye) via the web UI or `python3 scripts/ur_cam_calibrate.py`
4. Review calibration reports (reprojection error, camera matrix, hand-eye transform)
5. Validate accuracy using base projection overlay in the web dashboard

Results are saved to `temp/ur15_cam_calibration_result/`.

---

## 10. Workflow System

The workflow engine executes JSON-defined task sequences combining robot movements, image capture, and 3D positioning.

### 10.1 Running a Workflow

```bash
# Via CLI
ros2 run ur_workflow run_workflow --config /path/to/workflow.json

# Dry-run validation
ros2 run ur_workflow run_workflow --config workflow.json --dry-run
```

Or use the Workflow Config Center at `http://<host>:8008` to create, edit, and run workflows.

### 10.2 Workflow Format

```json
{
  "context": {
    "robot_ip": "192.168.1.15",
    "robot_port": 30002,
    "camera_topic": "/ur15_camera/image_raw",
    "positioning_service_url": "http://localhost:8004"
  },
  "operations": [
    {
      "type": "robot_move",
      "move_type": "movej",
      "robot_pose": [1.11, -1.02, 2.55, -2.84, -0.71, -1.95],
      "velocity": 0.5,
      "acceleration": 0.5,
      "wait_time": 1.0
    },
    {
      "type": "capture_image"
    },
    {
      "type": "positioning",
      "sub_type": "upload_view",
      "reference_name": "rack_bottom_left"
    }
  ]
}
```

### 10.3 Operation Types

| Type | Description |
|------|-------------|
| `robot_move` | Joint (`movej`), linear (`movel`), or offset (`move_tcp`) movement |
| `capture_image` | Capture and store camera image |
| `record_data` | Record robot pose data |
| `positioning` | 3D positioning operations (upload references, init session, get results) |

Example workflows are in `colcon_ws/src/ur_workflow/examples/`.

---

## 11. Rack Positioning (Calibration Workflow)

The rack positioning workflow locates a GB200 server rack using multi-view 3D positioning. It has three stages:

1. **Create a dataset** (one-time) — capture reference images and label rack corner keypoints
2. **Create a workflow** (one-time) — configure robot poses and 3D fitting parameters
3. **Run workflow online** (each time) — execute the workflow to compute the rack position, verify with "Draw GB200 Rack"

See [Rack Calibration Guide](rack_calibration.md) for the detailed step-by-step process.

---

## 12. Utility Scripts

Key scripts in `scripts/` for UR15 operations:

| Script | Purpose |
|--------|---------|
| `ur_dashboard_control.py` | Robot power, brake, program control via dashboard API |
| `ur_move_to_target.py` | Move to predefined joint positions (J6→J1 order) |
| `ur_joystick_control.py` | Real-time Cartesian joystick teleoperation |
| `ur_cam_calibrate.py` | Eye-in-hand calibration with ChArUco board |
| `ur_capture.py` | Automated image capture with pose recording |
| `ur_get_frame.py` | Get current end-effector frame |
| `ur_locate_rack.py` | Rack detection and localization |
| `ur_gripper.py` | Gripper control |
| `ur_force_test.py` | Force sensor testing |
| `ur_manual_collect_data.py` | Manual data collection (poses + images) |
| `ur_auto_collect_data.py` | Automated data collection |

---

## 13. Troubleshooting

| Issue | Solution |
|-------|---------|
| Cannot connect to robot | Verify robot is in Remote Control mode and reachable at `192.168.1.15` |
| Camera feed not showing | Check RTSP URL and camera IP `192.168.1.101`; verify with `ffplay rtsp://...` |
| ur_web starts before camera | Increase the delay for `ur_web` in `launch_modules` (default: 5s) |
| 3D positioning fails | Ensure FlowFormer++ server is running and `ffpp_url` is correctly set |
| Redis connection error | Verify Redis is running: `redis-cli ping` should return `PONG` |
| Port conflict | Check `ss -tlnp` for conflicting services on ports 8004–8030 |
| Calibration poor quality | Ensure good lighting, use all 20 poses with diverse orientations, check chessboard detection |
