# Robot DC Control System

A modular ROS 2-based control system for DC motors over RS-485 Modbus RTU, featuring centralized Modbus management and motor control. Designed for Jetson-based robot platforms with easy extensibility.

---

## Repository Structure

```
robot_dc/
├── colcon_ws/                     # ROS 2 workspace containing all packages
│   └── src/
│       ├── leadshine_motor/       # Leadshine motor ROS 2 package
│       ├── modbus_driver/         # Modbus RTU centralized bus driver package
│       ├── modbus_driver_interfaces/ # Shared Modbus service interface definitions
│       └── robot_bringup/         # Unified launch package for the robot
├── doc/                          # Documentation and FAQs
│   └── FAQ/
│       ├── enable_ch340_usb_serial_on_jetson.md
│       └── setup_joystick_ros2.md
├── scripts/                      # Utility and testing scripts
│   ├── jog_motor.py
│   └── serial_port_finder.py
├── install.sh                    # Setup helper script
├── requirements.txt              # Python dependencies
└── README.md                    # This file - global project overview
```

---

## Key Points

* **ROS 2 workspace** is located inside `colcon_ws/` with all source packages under `src/`.
* Each ROS 2 package contains its own detailed [README](colcon_ws/src).
* Individual package README files (relative paths):

  * [leadshine\_motor](colcon_ws/src/leadshine_motor/README.md)
  * [modbus\_driver](colcon_ws/src/modbus_driver/README.md)
  * [modbus\_driver\_interfaces](colcon_ws/src/modbus_driver_interfaces)
  * [robot\_bringup](colcon_ws/src/robot_bringup)
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

2. **Launch the full robot system:**

   ```bash
   ros2 launch robot_bringup robot_launch.py
   ```

3. **Use motor commands and utilities as described in the respective package README files.**

---

## Documentation

* Detailed FAQs and setup guides are available in the [`doc/FAQ/`](doc/FAQ/) folder.
* For example:

  * [Enabling CH340 USB serial devices on Jetson](doc/FAQ/enable_ch340_usb_serial_on_jetson.md)
  * [Setting up joystick support in ROS 2](doc/FAQ/setup_joystick_ros2.md)

---

## Contributing

See individual package README files (links above) for guidelines on testing, development, and coding standards.

---

## License & Maintainer

* Licensed under MIT License.
* Maintained by Jetson Developer — contact via [jetson@todo.todo](mailto:jetson@todo.todo).

---

