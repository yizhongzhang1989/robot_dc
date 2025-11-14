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
│       ├── robot_bringup/         # Unified system-level launch entry points
│       ├── robot_teleop/          # Joystick teleoperation support
│       └── robot_web/             # Web-based interface for control and monitoring
├── doc/                          # Documentation and FAQs
│   └── FAQ/
│       ├── enable_ch340_usb_serial_on_jetson.md
│       └── setup_joystick_ros2.md
├── scripts/                      # Utility and testing scripts
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
  * [robot\_teleop](colcon_ws/src/robot_teleop/README.md)
  * [robot\_web](colcon_ws/src/robot_web/README.md)
* Shared interfaces are in [`modbus_driver_interfaces`](colcon_ws/src/modbus_driver_interfaces).
* Documentation and troubleshooting FAQs are available in the `doc/FAQ/` folder:

  * [Enable CH340 USB serial on Jetson](doc/FAQ/enable_ch340_usb_serial_on_jetson.md)
  * [Setup joystick support in ROS 2](doc/FAQ/setup_joystick_ros2.md)
* Utility scripts for motor jogging and serial port detection are located in the [`scripts/`](scripts) directory.
* Use [`install.sh`](install.sh) to set up dependencies and environment quickly.
* Python dependencies are listed in [`requirements.txt`](requirements.txt).

---

## Quick Start

1. **Build and source the workspace:**

   ```bash
   cd colcon_ws
   colcon build
   source install/setup.bash
   ```

2. **Launch the full robot system (real hardware):**

   ```bash
   ros2 launch robot_bringup robot_launch.py
   ```

   ## Draw-wire Sensor Calibration

   Interactive script: `scripts/draw_wire_calibrate.py`

   Usage:
   1. Build and launch system so `/draw_wire_sensor/data` is publishing.
   2. Source workspace:
      ```bash
      source install/setup.bash
      python3 scripts/draw_wire_calibrate.py
      ```
   3. Move platform to a known physical height, type the numeric height and press Enter.
   4. Repeat for several heights covering the operating range.
   5. Type `cal` to compute linear calibration: `height ≈ sensor * scale + offset`.
   6. Type `save` to write `calibration_draw_wire.json`.
   7. Type `list` to view collected samples; `quit` to exit.

   Default raw field used is `register_1`. Adjust `RAW_FIELD` in the script if needed.
3. **Alternatively, run in simulation mode (no physical hardware required):**

   ```bash
   ros2 launch robot_bringup simulate_motor_launch.py
   ```

4. **Access the web interface for control and monitoring:**

   Open a browser and navigate to:

   ```
   http://<hostname>:8000
   ```

   > On Linux, you may need to allow port 8000 through your firewall to access from other PCs.

       ## Platform Overshoot Multi-Region Calibration (Web UI)

       The platform now supports predictive early-stop overshoot compensation using a multi-region calibration workflow.

       Workflow Summary:
       1. Open the web interface (`lift_robot_web` package, default port 8090).
       2. In the "Platform Overshoot Calibration" card click "Detect Range":
            * Performs initialization (pushrod timed down, platform down to base ≈832mm) and records the minimum height.
            * Sends manual UP and waits for stall detection (no height change for ≥0.5s) to record maximum height.
       3. Click "Multi-Region Calibrate" to automatically split the full range into 50 mm segments.
            Each segment runs the existing auto calibration (6 random alternating points). For each segment the residual overshoot (drift after stop pulse) is learned and saved.
       4. Region results are written into `colcon_ws/config/platform_overshoot_calibration.json`.

       JSON Format (v2):
       ```json
       {
          "enable": true,
          "generated_at": 1763099999.123,
          "generated_at_iso": "2025-11-14 14:05:12",
          "format_version": 2,
          "default": { "overshoot_up": 2.55, "overshoot_down": 2.78 },
          "regions": [
             { "lower": 832.0, "upper": 882.0, "overshoot_up": 2.40, "overshoot_down": 2.60, "generated_at": 1763099999.234, "generated_at_iso": "2025-11-14 14:05:12" },
             { "lower": 882.0, "upper": 932.0, "overshoot_up": 2.35, "overshoot_down": 2.55, "generated_at": 1763099999.987, "generated_at_iso": "2025-11-14 14:06:05" }
          ]
       }
       ```

       Runtime Selection:
       * When `goto_height` is invoked the `lift_robot_platform` node selects region where `lower <= target < upper` (last region inclusive upper bound).
       * If no region matches it falls back to `default` values.
       * Legacy single-value file (top-level `overshoot_up` / `overshoot_down`) is auto-migrated when first region is saved.

       Notes:
       * Overshoot value stored is the residual drift after the stop pulse (not total target error).
       * Minimum margin `OVERSHOOT_MIN_MARGIN` is enforced (below this early-stop is skipped).
       * Safe to regenerate: existing region with same bounds can be overwritten (`overwrite:true`).

       To apply new calibration restart the `lift_robot_platform` node (no rebuild required).

5. **Take camera snapshots via the web interface:**

   The system includes a dual-camera RTSP snapshot service. Once the system is running, you can:
   - Access the web interface at `http://<hostname>:8000`
   - Use the "Take Snapshot" button to capture images from both cameras
   - Images are automatically displayed in the web interface
   - Service endpoint: `/snapshot` (std_srvs/srv/Trigger)

6. **Refer to individual package READMEs for usage examples, APIs, and command formats.**

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

* Centralized Modbus RTU management
* Leadshine motor control via ROS 2 service interface
* RTSP camera snapshot service for dual-camera systems
* Web-based interface for visualization and command
* Simulation mode for development without hardware
* Joystick teleoperation support
* Modular architecture for easy extension

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
