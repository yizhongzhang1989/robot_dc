# Robot DC Control System

A modular ROS 2-based control system for DC motors over RS-485 Modbus RTU, featuring centralized Modbus management, motor control, web interface, and simulation support. Designed for Jetson-based robot platforms with easy extensibility.

---

## Repository Structure

```
robot_dc/
├── colcon_ws/                     # ROS 2 workspace containing all packages
│   └── src/
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

5. **Refer to individual package READMEs for usage examples, APIs, and command formats.**

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
