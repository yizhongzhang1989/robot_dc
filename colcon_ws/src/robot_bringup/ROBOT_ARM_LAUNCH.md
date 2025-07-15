# Robot Bringup - Robot Arm Launch

This document describes the robot arm launch file in the `robot_bringup` package.

## 🚀 Launch File

### `robot_arm_launch.py`
Configurable launch file for robot arm system with parameters.

**Usage:**
```bash
# With default parameters
ros2 launch robot_bringup robot_arm_launch.py

# With custom parameters
ros2 launch robot_bringup robot_arm_launch.py \
  robot_ip:=192.168.1.20 \
  robot_port:=7004 \
  device_id:=2 \
  web_port:=8081
```

**Available Parameters:**
- `robot_ip` (default: 192.168.1.10) - IP address of the DUCO robot arm
- `robot_port` (default: 7003) - Port number of the DUCO robot arm  
- `device_id` (default: 1) - Device ID for the robot arm
- `web_port` (default: 8080) - Port for the web interface

**What it launches:**
- DUCO robot arm node with configurable parameters
- Robot arm web interface with configurable parameters (2 second delay)

---

## 🎯 Quick Start

### Basic Usage:
```bash
# Launch robot arm system with default settings
ros2 launch robot_bringup robot_arm_launch.py

# Access web interface at: http://localhost:8080
```

### Custom Configuration:
```bash
# Custom robot arm configuration
ros2 launch robot_bringup robot_arm_launch.py \
  robot_ip:=192.168.1.100 \
  web_port:=9090

# Access web interface at: http://localhost:9090
```

---

## 🔧 Configuration Notes

### Robot Arm Connection:
- Ensure robot arm is powered on and connected to network
- Verify IP address is correct (default: 192.168.1.10)
- Check that port 7003 is accessible on the robot

### Web Interface:
- Default web port is 8080
- Web interface will be available at `http://localhost:<web_port>`
- Make sure the port is not already in use

### Device ID:
- Device ID determines the ROS topic name: `/arm{device_id}/cmd`
- Default device ID is 1, which creates topic `/arm1/cmd`
- Must match between robot arm node and web interface

---

## 🛠️ Troubleshooting

### Robot Arm Not Connecting:
```bash
# Check network connectivity
ping 192.168.1.10
telnet 192.168.1.10 7003

# Check if robot arm node is running
ros2 node list | grep duco_robot_arm
```

### Web Interface Not Loading:
```bash
# Check if web server is running
ros2 node list | grep robot_arm_web_server

# Check if port is available
netstat -tulpn | grep :8080

# Try different port
ros2 launch robot_bringup robot_arm_launch.py web_port:=8081
```

### Command Not Working:
```bash
# Check topic communication
ros2 topic echo /arm1/cmd

# Test manual command
ros2 topic pub /arm1/cmd std_msgs/String "data: 'power_on'"
```

---

## 📋 System Integration

The robot arm system is designed to work independently:

- **Independent Operation**: Robot arm runs as a standalone system
- **Web Interface**: Available at `http://localhost:8080` (or custom port)
- **ROS Integration**: Uses `/arm{device_id}/cmd` topic for commands
- **Configurable**: All parameters can be customized via launch arguments

---

## 🔗 Related Commands

```bash
# Show available launch arguments
ros2 launch robot_bringup robot_arm_launch.py --show-args

# Check running nodes
ros2 node list

# Monitor robot arm commands
ros2 topic echo /arm1/cmd

# Check robot arm status
ros2 topic list | grep arm

# Stop specific nodes
ros2 lifecycle set /duco_robot_arm_node shutdown
ros2 lifecycle set /robot_arm_web_server shutdown
```
