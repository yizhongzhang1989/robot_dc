# Robot DC Control System

A modular ROS 2-based control system for DC motors over RS-485 Modbus RTU, featuring centralized Modbus management, motor control, web interface, and simulation support. Designed for Jetson-based robot platforms with easy extensibility.

---

## Repository Structure

```
robot_dc/
├── colcon_ws/                     # ROS 2 workspace containing all packages
│   └── src/
│       ├── cam_node/              # RTSP camera snapshot service
│       ├── leadshine_motor/       # Leadshine motor control node and simulation logic
│       ├── modbus_driver/         # Central Modbus RTU driver and simulator
│       ├── modbus_driver_interfaces/ # Shared service/message interface definitions
│       ├── monitor/               # Robot data monitoring and recording
│       ├── robot_bringup/         # Unified system-level launch entry points
│       ├── robot_teleop/          # Joystick teleoperation support
│       └── robot_web/             # Web-based interface for control and monitoring
├── doc/                          # Documentation and FAQs
│   └── FAQ/
│       ├── enable_ch340_usb_serial_on_jetson.md
│       └── setup_joystick_ros2.md
├── scripts/                      # Utility and testing scripts
│   ├── robot_monitor_manager.py  # Unified data management tool
│   ├── test_udp_send.py          # UDP test data sender
│   ├── cam.py                    # Original camera script (reference)
│   ├── jog_motor.py
│   └── serial_port_finder.py
├── install.sh                    # Setup helper script
├── requirements.txt              # Python dependencies
└── README.md                     # This file - global project overview
```

---

## Key Points

* **ROS 2 workspace** is located inside `colcon_ws/` with all source packages under `src/`.
* Each ROS 2 package contains its own detailed README.
* Individual package README files (relative paths):

  * [cam\_node](colcon_ws/src/cam_node/README.md)
  * [leadshine\_motor](colcon_ws/src/leadshine_motor/README.md)
  * [modbus\_driver](colcon_ws/src/modbus_driver/README.md)
  * [monitor](colcon_ws/src/monitor/README.md) - **Robot data monitoring and recording**
  * [robot\_teleop](colcon_ws/src/robot_teleop/README.md)
  * [robot\_web](colcon_ws/src/robot_web/README.md)
* Shared interfaces are in [`modbus_driver_interfaces`](colcon_ws/src/modbus_driver_interfaces).
* **Robot monitoring** tools are in [`scripts/robot_monitor_manager.py`](scripts/robot_monitor_manager.py) for unified data management.
* Documentation and troubleshooting FAQs are available in the `doc/FAQ/` folder:

  * [Enable CH340 USB serial on Jetson](doc/FAQ/enable_ch340_usb_serial_on_jetson.md)
  * [Setup joystick support in ROS 2](doc/FAQ/setup_joystick_ros2.md)
* Utility scripts for motor jogging and serial port detection are located in the [`scripts/`](scripts) directory.
* Use [`install.sh`](install.sh) to set up dependencies and environment quickly.
* Python dependencies are listed in [`requirements.txt`](requirements.txt).

---

## Quick Start

### 1. System Setup

**Build and source the workspace:**

```bash
cd colcon_ws
colcon build
source install/setup.bash
```

### 2. Robot Control

**Launch the full robot system (real hardware):**

```bash
ros2 launch robot_bringup robot_launch.py
```

**Alternatively, run in simulation mode (no physical hardware required):**

```bash
ros2 launch robot_bringup simulate_motor_launch.py
```

**Access the web interface for control and monitoring:**

Open a browser and navigate to:
```
http://<hostname>:8000
```

### 3. Robot Data Monitoring

**Start monitoring (automatic data recording):**

```bash
ros2 launch monitor monitor.launch.py
```

**List recorded sessions:**

```bash
python3 scripts/robot_monitor_manager.py --list --detailed
```

**Analyze a specific session:**

```bash
python3 scripts/robot_monitor_manager.py --analyze ~/robot_data/2025-XX-XX/robot_monitor_HHMMSS
```

**View actual data content:**

```bash
# View robot data (TCP position, joint angles, force sensor)
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-XX-XX/robot_monitor_HHMMSS --topic /robot_data --limit 5 --full

# View log data
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-XX-XX/robot_monitor_HHMMSS --topic /log_data --limit 5
```

**Stop monitoring:**

```bash
pkill -f monitor_node
```

> 💡 **Tip:** Replace `HHMMSS` with the actual session timestamp from the `--list` command output.

### 4. Camera Monitoring

**Take camera snapshots via the web interface:**

The system includes a dual-camera RTSP snapshot service. Once the system is running, you can:
- Access the web interface at `http://<hostname>:8000`
- Use the "Take Snapshot" button to capture images from both cameras
- Images are automatically displayed in the web interface
- Service endpoint: `/snapshot` (std_srvs/srv/Trigger)

> 💡 **For detailed usage examples, APIs, and command formats, refer to individual package READMEs.**

---

## Robot Data Monitoring

The system includes a comprehensive monitoring solution that automatically records robot data to ROS 2 bags for analysis and replay.

### Monitoring Workflow

1. **Start Monitoring** → 2. **List Sessions** → 3. **Analyze Data** → 4. **View Content**

```bash
# 1. Start monitoring (records UDP data to ROS 2 bags)
ros2 launch monitor monitor.launch.py &

# 2. Send test data (optional)
python3 scripts/test_udp_send.py -n 20 -i 0.5

# 3. List all recorded sessions
python3 scripts/robot_monitor_manager.py --list --detailed

# 4. Analyze specific session
python3 scripts/robot_monitor_manager.py --analyze ~/robot_data/2025-XX-XX/robot_monitor_HHMMSS

# 5. View actual data content
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-XX-XX/robot_monitor_HHMMSS --topic /robot_data --limit 5 --full
```

### Monitoring Data Types

**Robot Data** (`/robot_data` topic):
- **RobotTcpPos**: TCP position [x, y, z, rx, ry, rz] in mm and degrees
- **RobotAxis**: Joint angles [J1-J6] in degrees
- **RobotTrack**: Track position
- **FTSensorData**: Force/torque sensor readings [Fx, Fy, Fz, Mx, My, Mz]
- **FTTarget**: Force/torque target values

**Log Data** (`/log_data` topic):
- System log messages and events
- Error notifications and status updates

### Management Commands

```bash
# Show configuration and status
python3 scripts/robot_monitor_manager.py --config

# List sessions for specific date
python3 scripts/robot_monitor_manager.py --list --date 2025-08-25

# Clean up old data (dry run)
python3 scripts/robot_monitor_manager.py --cleanup 30

# Test UDP connection
python3 scripts/robot_monitor_manager.py --test
```

### Data Storage

- **Location**: `~/robot_data/YYYY-MM-DD/robot_monitor_HHMMSS.microsec/`
- **Format**: ROS 2 bag files (SQLite + zstd compression)
- **Metadata**: Session info, topic statistics, duration tracking
- **Compatibility**: Works with standard `ros2 bag play/info` commands

---

## Remote Inspecting

For remote visual monitoring of the robot or workspace, you can launch a USB camera video stream server using the script located at [`scripts/webcam_server.py`](scripts/webcam_server.py).

1. **Start the webcam server**

   Make sure your USB camera is connected to the device (e.g., Jetson), then run:

   ```bash
   python3 scripts/webcam_server.py
   ```

   > The server listens on port **8010** by default. You can change this by modifying the script.

2. **View the camera stream**

   On any device in the same network, open a browser and visit:

   ```
   http://<robot-hostname>:8010
   ```

   Replace `<robot-hostname>` with the actual IP or hostname of your robot (e.g., `192.168.1.50`).

   > ⚠️ **Note:** The camera may take a few seconds to initialize. If the video doesn’t appear immediately, wait for a moment without refresh.

3. **Usage Notes**

   * The camera only streams when at least one viewer is connected.
   * Once all viewers disconnect, the camera is automatically turned off to save resources.
   * The stream supports multiple viewers simultaneously.

---


## Features

* **Centralized Modbus RTU management** - Unified control of all Modbus devices
* **Leadshine motor control** via ROS 2 service interface
* **Real-time robot data monitoring** - UDP data collection and ROS 2 bag recording
* **Comprehensive data analysis tools** - Session management, data viewing, and statistics
* **RTSP camera snapshot service** for dual-camera systems
* **Web-based interface** for visualization and command
* **Simulation mode** for development without hardware
* **Joystick teleoperation support**
* **Modular architecture** for easy extension

### Robot Monitoring Features

* ✅ **UDP data monitoring** - Listens on ports 5566 (robot data) and 5577 (log data)
* ✅ **Automatic ROS 2 bag recording** - Saves data to `~/robot_data/` with timestamps
* ✅ **Data compression** - Uses zstd compression for efficient storage
* ✅ **Session management** - Automatic session info and metadata generation
* ✅ **Unified data viewer** - View robot TCP position, joint angles, force sensor data
* ✅ **Analysis tools** - Session statistics, message counts, duration tracking
* ✅ **Native ROS 2 integration** - Compatible with standard `ros2 bag` tools

---

## Documentation

* **Package-specific documentation**:
  * [Robot Monitor (monitor)](colcon_ws/src/monitor/README.md) - Data monitoring and recording
  * [Leadshine Motor](colcon_ws/src/leadshine_motor/README.md) - Motor control
  * [Modbus Driver](colcon_ws/src/modbus_driver/README.md) - Communication layer
  * [Robot Web](colcon_ws/src/robot_web/README.md) - Web interface

* **FAQ and setup guides** are available in the [`doc/FAQ/`](doc/FAQ/) folder:
  * [Enabling CH340 USB serial devices on Jetson](doc/FAQ/enable_ch340_usb_serial_on_jetson.md)
  * [Setting up joystick support in ROS 2](doc/FAQ/setup_joystick_ros2.md)

* **Monitoring tools**:
  * [`robot_monitor_manager.py`](scripts/robot_monitor_manager.py) - Unified data management
  * [`test_udp_send.py`](scripts/test_udp_send.py) - UDP test data generator

---

## Contributing

See individual package README files (linked above) for testing instructions, development practices, and coding style.

---

## License & Maintainer

* Licensed under the MIT License.
* Maintained by Jetson Developer — contact via [jetson@todo.todo](mailto:jetson@todo.todo).

---
