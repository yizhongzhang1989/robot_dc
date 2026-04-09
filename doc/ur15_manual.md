# UR15 Robot Manual

This document covers the configuration, launch, and operation of the UR15 robot arm system within the Robot DC platform.

---

## 1. System Overview

The UR15 subsystem includes:

| Component | Package | Description | Dashboard |
|-----------|---------|-------------|-----------|
| Robot driver | `ur15_robot_arm` | UR robot driver + arm control node | — |
| Camera | `camera_node` | RTSP capture → ROS 2 image + MJPEG stream | :8019 |
| Web dashboard | `ur15_web` | Main control & calibration UI | :8030 |
| Workflow engine | `ur15_workflow` | JSON-based task sequencing | :8008 |
| 3D positioning | `positioning_3d_service` | Multi-view triangulation via FlowFormer++ | :8004 |
| Image labeling | `image_labeling_service` | Web-based keypoint annotation | :8007 |
| Camera calibration | `camcalib_web_service` | Intrinsic & eye-in-hand calibration | :8006 |
| Status store | `robot_status_redis` | Redis-backed status management | :8005 |

---

## 2. Configuration

All UR15 settings are in `config/robot_config.yaml`. Key sections:

### 2.1 Robot Connection

```yaml
ur15:
  robot:
    type: "ur15"
    ip: "192.168.1.15"        # Robot IP address
    ports:
      control: 30002          # URScript command interface
      dashboard: 29999        # Dashboard server (power, brake, program control)
      rtde: 30004             # Real-time data exchange
      modbus: 502             # Modbus TCP
      rs485: 54321            # RS485 serial communication
    launch_rviz: false
    max_velocity: 1.0
    max_acceleration: 1.4
```

The robot must be in **Remote Control mode** (not local Teach Pendant mode) for programmatic control.

### 2.2 Tool Definitions

Four end-effector tools are defined with flange-to-tip offsets:

```yaml
  tool:
    tool_rotate:    { length: 0.325, angle_z: -31 }
    tool_pushpull:  { length: 0.325, angle_z: -31 }
    tool_frame:     { length: 0.345, angle_z: -33 }
    tool_extract:   { length: 0.225, angle_z: -31 }
```

- `length` — distance from flange to tool tip (meters)
- `angle_z` — rotation offset around Z axis (degrees)

### 2.3 Camera

```yaml
  camera:
    name: "UR15Camera"
    ip: "192.168.1.101"
    rtsp_url: "rtsp://admin:123456@192.168.1.101/stream0"
    server_port: 8019         # MJPEG web stream
    topic: "/ur15_camera/image_raw"
    stream_fps: 25
    jpeg_quality: 75
    max_width: 800
    publish_ros_image: true
```

### 2.4 Web Interface

```yaml
  web:
    port: 8030
    camera_topic: "/ur15_camera/image_raw"
    dataset_path: "dataset"
    calibration_data_path: "temp/ur15_cam_calibration_data"
    calibration_result_path: "temp/ur15_cam_calibration_result"
    chessboard_config_path: "temp/ur15_cam_calibration_data/chessboard_config.json"
    rack_positions: "temp/ur15_rack_positions.json"
```

### 2.5 Camera Calibration

```yaml
  calibration:
    chessboard:
      rows: 9
      cols: 6
      square_size: 0.025      # meters
    collect:
      num_poses: 20
      delay_seconds: 2
```

### 2.6 Launch Modules

The `ur15.launch_modules` list controls which modules are started by the bringup launch. Each module can be individually enabled/disabled:

```yaml
  launch_modules:
    - package: "robot_status_redis"
      launch_file: "robot_status_launch.py"
      delay: 0.0
      enabled: true           # Set to false to skip
    # ... (8 modules total)
```

### 2.7 Service Ports

Shared services are configured under the `services` section:

| Service | Config Key | Default Port |
|---------|-----------|-------------|
| Robot Status Redis | `services.robot_status_redis.web.port` | 8005 |
| 3D Positioning | `services.positioning_3d.port` | 8004 |
| Image Labeling | `services.image_labeling.port` | 8007 |
| Camera Calibration | `services.camcalib_web.port` | 8006 |
| Workflow Config Center | `services.workflow_config_center.port` | 8008 |

The FlowFormer++ server URL is set at `services.positioning_3d.ffpp_url` (e.g., `http://10.172.100.34:8001`).

---

## 3. Launching

### 3.1 Full System Launch

Start all enabled UR15 modules:

```bash
ros2 launch robot_bringup ur15_bringup.py
```

This sequentially launches each module from `ur15.launch_modules`. The `ur15_web` module has a 5-second delay to allow the camera node to initialize first.

### 3.2 Individual Module Launch

Launch specific modules independently:

```bash
# Robot driver only
ros2 launch ur15_robot_arm ur15_control_launch.py

# Robot driver + arm node
ros2 launch ur15_robot_arm ur15_robot_arm.launch.py

# Robot driver + arm node + joystick
ros2 launch ur15_robot_arm ur15_robot_arm_with_joy.launch.py

# Camera node
ros2 launch camera_node ur15_cam_launch.py

# Web interface
ros2 launch ur15_web ur15_web_launch.py
```

### 3.3 Disabling Modules

To skip a module (e.g., when hardware is not connected), set `enabled: false` in `robot_config.yaml`:

```yaml
    - package: "ur15_robot_arm"
      launch_file: "ur15_control_launch.py"
      delay: 0.0
      enabled: false  # skip if UR15 not connected
```

---

## 4. Web Interfaces

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

## 5. ROS 2 Topics and Services

### Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/ur15_camera/image_raw` | `sensor_msgs/Image` | `camera_node` | Camera frames |
| `/joint_states` | `sensor_msgs/JointState` | `ur_robot_driver` | Joint positions/velocities |
| `/cartesian_motion_controller/current_pose` | `geometry_msgs/PoseStamped` | `ur_robot_driver` | Current TCP pose |
| `/target_frame` | `geometry_msgs/PoseStamped` | `ur15_robot_arm_node` | Target pose commands |
| `/ur15_robot_arm/status` | `std_msgs/String` | `ur15_robot_arm_node` | Robot status messages |

### Services (robot_status_redis)

| Service | Description |
|---------|-------------|
| `SetStatus` | Store a status value |
| `GetStatus` | Retrieve a status value |
| `ListStatus` | List all status keys |
| `DeleteStatus` | Delete a status entry |

---

## 6. Robot Control API

The `UR15Robot` class (`ur15_robot_arm.ur15`) provides direct socket-based control:

```python
from ur15_robot_arm.ur15 import UR15Robot

robot = UR15Robot()
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

## 7. Dashboard Control

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

## 8. Camera Calibration

### 8.1 Data Collection

Collect calibration images and robot poses via the web UI at `:8030`, or use the script:

```bash
python3 scripts/ur_capture.py
```

This captures images from the camera while recording the robot's TCP pose at each position. Data is saved to `temp/ur15_cam_calibration_data/`.

**Settings**: 9×6 chessboard, 25mm square size, 20 poses with 2-second delays (configurable in `ur15.calibration`).

### 8.2 Run Calibration

```bash
python3 scripts/ur_cam_calibrate.py
```

Performs:
1. Intrinsic calibration (camera matrix, distortion coefficients)
2. Eye-in-hand (hand-eye) calibration using ChArUco board
3. Generates calibration report
4. Saves results to `temp/ur15_cam_calibration_result/`

### 8.3 Web-Based Calibration

Use the camera calibration service at `http://<host>:8006` for an interactive calibration workflow with image upload, visualization, and result export.

---

## 9. Workflow System

The workflow engine executes JSON-defined task sequences combining robot movements, image capture, and 3D positioning.

### 9.1 Running a Workflow

```bash
# Via CLI
ros2 run ur15_workflow run_workflow --config /path/to/workflow.json

# Dry-run validation
ros2 run ur15_workflow run_workflow --config workflow.json --dry-run
```

Or use the Workflow Config Center at `http://<host>:8008` to create, edit, and run workflows.

### 9.2 Workflow Format

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

### 9.3 Operation Types

| Type | Description |
|------|-------------|
| `robot_move` | Joint (`movej`), linear (`movel`), or offset (`move_tcp`) movement |
| `capture_image` | Capture and store camera image |
| `record_data` | Record robot pose data |
| `positioning` | 3D positioning operations (upload references, init session, get results) |

Example workflows are in `colcon_ws/src/ur15_workflow/examples/`.

---

## 10. Rack Positioning (Calibration Workflow)

The rack positioning workflow locates a GB200 server rack using multi-view 3D positioning:

1. Enter freedrive mode and move the robot to capture reference views of rack corners
2. Label keypoints (e.g., `GB200_Rack_Bottom_Left_Corner`) in the image labeling tool at `:8007`
3. Load a positioning workflow template (e.g., `wf_rack_location_simplify.json`) in the workflow config center at `:8008`
4. Modify poses as needed, then run the workflow
5. Verify results using "Draw GB200 Rack" in the web dashboard at `:8030`

See [calibration_guideline.md](calibration_guideline.md) for step-by-step instructions.

---

## 11. Utility Scripts

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

## 12. Troubleshooting

| Issue | Solution |
|-------|---------|
| Cannot connect to robot | Verify robot is in Remote Control mode and reachable at `192.168.1.15` |
| Camera feed not showing | Check RTSP URL and camera IP `192.168.1.101`; verify with `ffplay rtsp://...` |
| ur15_web starts before camera | Increase the delay for `ur15_web` in `launch_modules` (default: 5s) |
| 3D positioning fails | Ensure FlowFormer++ server is running and `ffpp_url` is correctly set |
| Redis connection error | Verify Redis is running: `redis-cli ping` should return `PONG` |
| Port conflict | Check `ss -tlnp` for conflicting services on ports 8004–8030 |
| Calibration poor quality | Ensure good lighting, use all 20 poses with diverse orientations, check chessboard detection |
