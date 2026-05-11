# Robot DC Control System

A modular ROS 2-based control system for the DC robot, which consists of a dual-arm robot (UR15 + Duco GCR5-910) and a lift robot platform. Features centralized Modbus management, multi-robot coordination, web dashboards, camera calibration, 3D positioning, and Redis-based status management.

---

## Prerequisites

* **OS:** Ubuntu 22.04 (x86_64 or Jetson ARM64)
* **ROS 2:** Humble Hawksbill (`ros-humble-desktop` or `ros-humble-base`)
* **Python:** 3.10+
* **Git** (with submodule support)
* **Redis** server (for `robot_status_redis`)
* **Robot Vision System** (on a separate machine):
  The [`scripts/ThirdParty/robot_vision`](scripts/ThirdParty/robot_vision/README.md) system must be deployed on a dedicated machine with GPU support. This provides the FlowFormer++ (FFPP) keypoint tracking server used by the 3D positioning service. After setup, set the FFPP server URL in `config/robot_config.yaml`:
  ```yaml
  services:
    positioning_3d:
      ffpp_url: "http://<vision-machine-ip>:8101"  # Note: http:// prefix is required

  shared:
    network:
      ffpp_server:
        url: "http://<vision-machine-ip>:8101"
  ```
  Replace `<vision-machine-ip>` with the actual IP of the machine running the FFPP server. Both entries must be updated.

---

## Quick Start

1. **Clone and initialize submodules:**

   ```bash
   git clone <repo-url> robot_dc
   cd robot_dc
   git submodule update --init --recursive
   ```

2. **Copy and edit the centralized configuration file:**

   This single config file is shared by all modules across the system.

   ```bash
   cp config/robot_config.example.yaml config/robot_config.yaml
   # Edit config/robot_config.yaml with your robot IP addresses, camera URLs,
   # FFPP server URL, etc.
   ```

   > **Firewall:** Make sure all network ports listed in `config/robot_config.yaml` are allowed through your firewall (e.g. `sudo ufw allow <port>`), otherwise the web dashboards, cameras, and robot interfaces will not be reachable.

3. **Install system dependencies:**

   These APT packages are required for the UR15 driver, RTSP camera probing, and Redis status backend.

   - `redis-server` — backend for `robot_status_redis`
   - `ffmpeg` — provides `ffprobe`, used by `camera_node` to detect RTSP stream resolution
   - `ros-humble-ur` — UR ROS 2 driver metapackage (provides `ur_robot_driver`, `ur_dashboard_msgs`, `ur_msgs`, etc.)

   ```bash
   sudo apt-get update
   sudo apt-get install -y redis-server ffmpeg ros-humble-ur

   # Start and enable Redis
   sudo systemctl start redis-server
   sudo systemctl enable redis-server
   redis-cli ping  # Should return "PONG"
   ```

4. **Install Python dependencies:**

   ```bash
   pip3 install -r requirements.txt
   ```

5. **Build and source the workspace:**

   ```bash
   cd colcon_ws
   colcon build
   source install/setup.bash
   ```

   > **Tip:** Add `source ~/Documents/robot_dc/colcon_ws/install/setup.bash` to your `~/.bashrc` so the workspace is sourced automatically in every new terminal.

6. **Launch the UR15 system (all modules):** (see [UR15 Manual](doc/ur15_manual.md) for details)

   ```bash
   ros2 launch robot_bringup ur15_bringup.py
   ```

   This starts all enabled modules defined in `config/robot_config.yaml` under `ur15.launch_modules`:
   | Module | Default Port | Description |
   |--------|------|-------------|
   | `robot_status_redis` | 8005 | Redis status store + web dashboard |
   | `positioning_3d_service` | 8004 | 3D positioning (requires FFPP server) |
   | `image_labeling_service` | 8007 | Image annotation tool |
   | `camcalib_web_service` | 8006 | Camera calibration |
   | `ur15_workflow` | 8008 | Workflow config center |
   | `ur15_robot_arm` | — | UR robot driver |
   | `camera_node` | 8019 | RTSP camera → ROS2 + MJPEG |
   | `ur15_web` | 8030 | Main web dashboard |

   Disable any module by setting `enabled: false` in the config if the corresponding hardware is not connected.

7. **Launch the lift robot system:**

   ```bash
   ros2 launch robot_bringup lift_robot_bringup.py
   ```

8. **Launch in simulation mode (no physical hardware):**

   ```bash
   ros2 launch robot_bringup simulate_motor_launch.py
   ```

---

## Key Points

* **ROS 2 workspace** is located inside `colcon_ws/` with all source packages under `src/`.
* Each ROS 2 package contains its own detailed README.
* Individual package README files (relative paths):

  * [cam\_node](colcon_ws/src/cam_node/README.md)
  * [leadshine\_motor](colcon_ws/src/leadshine_motor/README.md)
  * [modbus\_driver](colcon_ws/src/modbus_driver/README.md)
  * [robot\_status](colcon_ws/src/robot_status/README.md) - Original file-based status management
  * [robot\_status\_redis](colcon_ws/src/robot_status_redis/README.md) - **Recommended** high-performance Redis-based status management
  * [robot\_teleop](colcon_ws/src/robot_teleop/README.md)
  * [robot\_web](colcon_ws/src/robot_web/README.md)
* Shared interfaces are in [`modbus_driver_interfaces`](colcon_ws/src/modbus_driver_interfaces).
* Documentation and troubleshooting FAQs are available in the `doc/FAQ/` folder:

  * [Enable CH340 USB serial on Jetson](doc/FAQ/enable_ch340_usb_serial_on_jetson.md)
  * [Setup joystick support in ROS 2](doc/FAQ/setup_joystick_ros2.md)
* Utility scripts for motor jogging and serial port detection are located in the [`scripts/`](scripts) directory.
* Python dependencies are listed in [`requirements.txt`](requirements.txt).

---

## Remote Inspecting

For remote visual monitoring of the robot or workspace, you can launch a USB camera video stream server using the script located at [`scripts/webcam_server.py`](scripts/webcam_server.py).

---

## Repository Structure

```
robot_dc/
├── colcon_ws/                     # ROS 2 workspace containing all packages
│   └── src/
│       ├── camcalib_web_service/  # Camera calibration web service
│       ├── camera_node/           # Camera node for image capture
│       ├── common/                # Common utilities and shared code
│       ├── draw_wire_sensor/      # Draw wire sensor integration
│       ├── duco_gcr5_910_urdf/    # URDF models for Duco GCR5-910 robot
│       ├── duco_robot_arm/        # Duco robot arm control
│       ├── duco_robot_arm_state/  # Duco robot arm state management
│       ├── feetech_servo/         # Feetech servo motor control
│       ├── image_labeling_service/ # Image labeling service
│       ├── image_process/         # Image processing utilities
│       ├── image_streaming/       # Image streaming service
│       ├── leadshine_motor/       # Leadshine motor control node and simulation logic
│       ├── lift_robot_force_sensor/ # Force sensor integration (single channel)
│       ├── lift_robot_force_sensor_4channel/ # 4-channel force sensor integration
│       ├── lift_robot_interfaces/ # Custom ROS 2 interfaces for lift robot
│       ├── lift_robot_platform/   # Lift platform control
│       ├── lift_robot_web/        # Web interface for lift robot
│       ├── modbus_devices/        # Modbus device drivers
│       ├── modbus_driver/         # Central Modbus RTU driver and simulator
│       ├── modbus_driver_interfaces/ # Shared service/message interface definitions
│       ├── motor_status/          # Motor status monitoring
│       ├── platform_controller/   # Platform motion controller
│       ├── positioning_3d_service/ # 3D positioning service
│       ├── robotiq_2f140_gripper/ # Robotiq 2F-140 gripper control
│       ├── robotiq_2f140_gripper_web/ # Web interface for Robotiq gripper
│       ├── robotiq_gripper_msgs/  # Robotiq gripper message definitions
│       ├── robot_arm_web/         # Generic robot arm web interface
│       ├── robot_bringup/         # Unified system-level launch entry points
│       ├── robot_status/          # Original file-based robot status management
│       ├── robot_status_redis/    # Redis-based status management (recommended)
│       ├── robot_teleop/          # Joystick teleoperation support
│       ├── robot_web/             # Web-based interface for control and monitoring
│       ├── system_monitor/        # System monitoring utilities
│       ├── test_web/              # Web testing utilities
│       ├── ur15_robot_arm/        # UR15 robot arm control package
│       ├── ur15_web/              # Web interface for UR15 robot
│       ├── ur15_workflow/         # UR15 workflow management
│       └── urdf_web_viewer/       # URDF model web viewer
├── config/                        # Configuration files
│   └── robot_config.example.yaml  # Example robot configuration
├── doc/                           # Documentation and FAQs
│   ├── FAQ/
│   │   ├── enable_ch340_usb_serial_on_jetson.md
│   │   └── setup_joystick_ros2.md
│   ├── action_control_commands.md # Action control command reference
│   ├── calibration_guideline.md   # Calibration procedures
│   └── lift_robot_system_guide.md # Lift robot system guide
├── resources/                     # Resource files
│   └── 3d_model/                  # GB200 server's 3D models and CAD files
├── scripts/                       # Utility and testing scripts (100+ scripts)
│   ├── cam.py                     # Camera utilities
│   ├── webcam_server.py           # Webcam streaming server
│   ├── jog_motor.py               # Motor jogging utility
│   ├── serial_port_finder.py      # Serial port detection
│   ├── duco_*.py                  # Duco robot utilities
│   ├── ur_wobj_*.py               # Implementation of task operation 
│   ├── force_sensor_*.py          # Force sensor utilities
│   └── ...                        # Additional utility scripts
├── requirements.txt               # Python dependencies
└── README.md                      # This file - global project overview
```

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

* **High-Performance Status Management**
  * Redis-based robot status with <200μs operation latency
  * Thread-safe, no ROS2 executor conflicts
  * Automatic pickle serialization for any Python object (dict, numpy, custom classes)
  * Event-driven auto-save with Redis keyspace notifications
  * Web dashboard with real-time visualization at http://localhost:8005
* **Robot Control**
  * Centralized Modbus RTU management
  * Leadshine motor control via ROS 2 service interface
  * RTSP camera snapshot service for dual-camera systems
  * Web-based interface for visualization and command
  * Simulation mode for development without hardware
  * Joystick teleoperation support
* **Architecture**
  * Modular design for easy extension
  * Flexible **kwargs interface for backward compatibility
  * Direct Redis access eliminates service call overhead

---

## Documentation

* Detailed FAQs and setup guides are available in the [`doc/FAQ/`](doc/FAQ/) folder.
* Examples:

  * [Enabling CH340 USB serial devices on Jetson](doc/FAQ/enable_ch340_usb_serial_on_jetson.md)
  * [Setting up joystick support in ROS 2](doc/FAQ/setup_joystick_ros2.md)

---

## Contributing

See individual package README files (linked above) for testing instructions, development practices, and coding style.

---

## License & Maintainer

* Licensed under the MIT License.
* Maintained by Jetson Developer — contact via [jetson@todo.todo](mailto:jetson@todo.todo).

---
