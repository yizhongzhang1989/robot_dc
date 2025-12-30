# Robot DC Control System

A modular ROS 2-based control system for DC motors over RS-485 Modbus RTU, featuring centralized Modbus management, motor control, web interface, and simulation support. Designed for Jetson-based robot platforms with easy extensibility.

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
├── install.sh                     # Setup helper script
├── requirements.txt               # Python dependencies
└── README.md                      # This file - global project overview
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
* Use [`install.sh`](install.sh) to set up dependencies and environment quickly.
* Python dependencies are listed in [`requirements.txt`](requirements.txt).

---

## Quick Start

1. **Install Redis (Required for robot_status_redis):**

   ```bash
   # Install Redis server
   sudo apt-get update
   sudo apt-get install redis-server
   
   # Install Python Redis client
   pip3 install redis
   
   # Start Redis and enable auto-start on boot
   sudo systemctl start redis-server
   sudo systemctl enable redis-server
   
   # Verify Redis is running
   redis-cli ping  # Should return "PONG"
   ```

2. **Build and source the workspace:**

   ```bash
   cd colcon_ws
   colcon build
   source install/setup.bash
   ```

3. **Launch the robot status system (Redis-based, recommended):**

   ```bash
   # Launch with web dashboard at http://localhost:8005
   ros2 launch robot_status_redis robot_status_launch.py
   
   # Or with custom settings
   ros2 launch robot_status_redis robot_status_launch.py \
     web_port:=8080 \
     auto_save_file_path:=/path/to/status.json
   ```

4. **Launch the full robot system (real hardware):**

   ```bash
   ros2 launch robot_bringup robot_launch.py
   ```

5. **Alternatively, run in simulation mode (no physical hardware required):**

   ```bash
   ros2 launch robot_bringup simulate_motor_launch.py
   ```

6. **Use robot status in your Python code:**

   ```python
   from robot_status_redis.client_utils import RobotStatusClient
   import numpy as np
   
   # Create client (no ROS2 node required)
   client = RobotStatusClient()
   
   # Store any Python object (dict, numpy arrays, custom classes)
   client.set_status('robot1', 'pose', {'x': 1.5, 'y': 2.3, 'z': 0.5})
   client.set_status('robot1', 'camera_matrix', np.eye(3))
   
   # Retrieve with original types preserved
   pose = client.get_status('robot1', 'pose')  # Returns dict
   matrix = client.get_status('robot1', 'camera_matrix')  # Returns numpy array
   
   # View all status at http://localhost:8005
   ```

7. **Access the web interface for control and monitoring:**

   Open a browser and navigate to:

   ```
   http://<hostname>:8000  # Main robot control
   http://<hostname>:8005  # Robot status dashboard
   ```

   > On Linux, you may need to allow ports 8000 and 8005 through your firewall to access from other PCs.

8. **Take camera snapshots via the web interface:**

   The system includes a dual-camera RTSP snapshot service. Once the system is running, you can:
   - Access the web interface at `http://<hostname>:8000`
   - Use the "Take Snapshot" button to capture images from both cameras
   - Images are automatically displayed in the web interface
   - Service endpoint: `/snapshot` (std_srvs/srv/Trigger)

9. **Refer to individual package READMEs for usage examples, APIs, and command formats.**

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
