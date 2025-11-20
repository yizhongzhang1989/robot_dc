# Robot Status Management System

A ROS2 package for centralized robot status management with dynamic multi-robot support, hierarchical namespaces, and web dashboard visualization.

## Features

- **Dynamic Multi-Robot Support**: Add robots without code changes - namespaces are created on-the-fly
- **Hierarchical Organization**: Organize status by namespace (per-robot or shared)
- **Three Service Interfaces**: `set_status`, `get_status`, `list_status`
- **Web Dashboard**: Real-time visualization at http://localhost:8005
- **Thread-Safe Operations**: Concurrent access protection with locking
- **JSON Support**: Store complex data structures as JSON strings
- **Python Client Utils**: Easy integration with helper class

## Installation

```bash
cd ~/your_workspace
colcon build --packages-select robot_status
source install/setup.bash
```

## Quick Start

### 1. Launch the System

```bash
ros2 launch robot_status robot_status_launch.py
```

**Launch Parameters:**
- `web_enabled:=true/false` - Enable/disable web dashboard (default: true)
- `web_port:=8005` - Web server port (default: 8005)
- `web_host:=0.0.0.0` - Web server host (default: 0.0.0.0)

**Example:**
```bash
ros2 launch robot_status robot_status_launch.py web_port:=8080
```

### 2. Access the Web Dashboard

Open your browser and navigate to:
- Local: http://localhost:8005
- Network: http://YOUR_IP:8005

The dashboard auto-refreshes every 2 seconds and displays all status organized by namespace.

## Service Interface

### Set Status

Store a status value for any namespace.key combination.

**Service:** `/robot_status/set`

**Command:**
```bash
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'pose', value: '{\"x\":1.5,\"y\":2.3,\"z\":0.5}'}"
```

**Parameters:**
- `ns` (string): Namespace identifier (e.g., 'robot1', 'robot2', 'shared')
- `key` (string): Status key (e.g., 'pose', 'battery', 'state')
- `value` (string): Value as string or JSON

**Response:**
- `success` (bool): Operation success status
- `message` (string): Result message

**Examples:**
```bash
# Set robot pose (JSON)
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'pose', value: '{\"x\":1.5,\"y\":2.3,\"z\":0.5}'}"

# Set battery level (plain string)
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot2', key: 'battery', value: '85'}"

# Set shared mission ID
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'shared', key: 'mission_id', value: 'task_123'}"

# Set robot state
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'state', value: 'IDLE'}"
```

### Get Status

Retrieve a specific status value.

**Service:** `/robot_status/get`

**Command:**
```bash
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot1', key: 'pose'}"
```

**Parameters:**
- `ns` (string): Namespace identifier
- `key` (string): Status key to retrieve

**Response:**
- `success` (bool): Whether the parameter exists
- `value` (string): The stored value
- `message` (string): Success or error message

**Examples:**
```bash
# Get robot1 pose
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot1', key: 'pose'}"

# Get robot2 battery
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot2', key: 'battery'}"

# Get shared mission_id
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'shared', key: 'mission_id'}"
```

### List Status

List all status or filter by namespace.

**Service:** `/robot_status/list`

**Command:**
```bash
# List all status
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: ''}"

# List only robot1 status
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: 'robot1'}"
```

**Parameters:**
- `ns` (string): Namespace filter (empty string = all namespaces)

**Response:**
- `success` (bool): Operation success
- `namespaces` (string[]): List of namespace identifiers
- `status_dict` (string): JSON dictionary of all status
- `message` (string): Summary message

**Examples:**
```bash
# List all namespaces and status
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: ''}"

# Filter by robot1 only
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: 'robot1'}"

# Filter by shared namespace
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: 'shared'}"
```

### Delete Status

Remove a specific status key or an entire namespace.

**Service:** `/robot_status/delete`

**Command:**
```bash
# Delete specific key
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: 'pose'}"

# Delete entire namespace
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: ''}"
```

**Parameters:**
- `ns` (string): Namespace identifier
- `key` (string): Status key to delete (empty string = delete entire namespace)

**Response:**
- `success` (bool): Operation result
- `message` (string): Result message

**Examples:**
```bash
# Delete robot1 pose
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: 'pose'}"

# Delete robot1 battery
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot2', key: 'battery'}"

# Delete entire robot1 namespace (all keys)
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: ''}"

# Delete entire shared namespace
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'shared', key: ''}"
```

## Python Client API

Use the `RobotStatusClient` helper class for easy integration:

### Basic Usage

```python
#!/usr/bin/env python3
import rclpy
from robot_status.client_utils import RobotStatusClient

rclpy.init()
node = rclpy.create_node('my_node')

# Create client
client = RobotStatusClient(node)

# Wait for services
client.wait_for_services(timeout_sec=5.0)

# Set status (automatically handles JSON encoding)
client.set_status('robot1', 'pose', {'x': 1.5, 'y': 2.3, 'z': 0.5})
client.set_status('robot1', 'battery', 95)
client.set_status('robot1', 'state', 'MOVING')

# Get status (automatically decodes JSON if possible)
pose = client.get_status('robot1', 'pose')
print(f"Pose: {pose}")  # {'x': 1.5, 'y': 2.3, 'z': 0.5}

battery = client.get_status('robot1', 'battery')
print(f"Battery: {battery}%")  # 95

# List all status
all_status = client.list_status()
print(f"All status: {all_status}")

# List robot1 status only
robot1_status = client.list_status(namespace='robot1')
print(f"Robot1 status: {robot1_status}")

# Get all namespaces
namespaces = client.get_namespaces()
print(f"Active robots: {namespaces}")  # ['robot1', 'robot2', 'shared']

# Delete specific key
client.delete_status('robot1', 'pose')

# Delete entire namespace
client.delete_status('robot2')  # Deletes all robot2 keys

node.destroy_node()
rclpy.shutdown()
```

### Running Examples

The package includes example scripts:

```bash
# Set example status for multiple robots
python3 install/robot_status/share/robot_status/examples/set_example.py

# Query and display status
python3 install/robot_status/share/robot_status/examples/get_example.py

# Multi-robot simulation (3 robots with continuous updates)
python3 install/robot_status/share/robot_status/examples/multi_robot_example.py
```

## Use Cases

### Single Robot Status

```bash
# Set robot state
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'state', value: 'IDLE'}"

# Set current task
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'current_task', value: 'navigation'}"

# Set error status
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'error', value: 'none'}"
```

### Multi-Robot Fleet

```bash
# Robot 1 status
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'location', value: 'warehouse_A'}"

# Robot 2 status
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot2', key: 'location', value: 'warehouse_B'}"

# Robot 3 status
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot3', key: 'location', value: 'charging_station'}"

# Shared fleet status
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'shared', key: 'active_robots', value: '3'}"
```

### Monitoring and Coordination

```bash
# Check all robots
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: ''}"

# Check specific robot
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot2', key: 'battery'}"

# Update shared mission
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'shared', key: 'mission_status', value: 'in_progress'}"
```

## Architecture

### Component Overview

```
robot_status/
├── robot_status_node.py    # Core status storage using ROS2 parameters
├── web_dashboard.py        # Flask web interface + REST API
└── client_utils.py         # Python helper utilities
```

### Design Principles

1. **Dynamic Namespaces**: No pre-configuration required - namespaces are created automatically when first used
2. **Hierarchical Storage**: Status stored as `namespace.key` parameters (e.g., `robot1.pose`)
3. **Thread-Safe**: All parameter operations protected with threading locks
4. **Flexible Values**: Supports plain strings and JSON-encoded complex objects
5. **Persistent Services**: Services remain available throughout node lifetime

### Data Flow

```
Client → /robot_status/set → RobotStatusNode → ROS2 Parameters
Client → /robot_status/get → RobotStatusNode → ROS2 Parameters
Client → /robot_status/list → RobotStatusNode → ROS2 Parameters

Web Browser → WebDashboard (Flask) → RobotStatusNode Services → Display
```

## Troubleshooting

### Services Not Available

```bash
# Check if node is running
ros2 node list | grep robot_status

# List available services
ros2 service list | grep robot_status

# Restart the system
ros2 launch robot_status robot_status_launch.py
```

### Web Dashboard Not Accessible

```bash
# Check if port is in use
lsof -i:8005

# Try different port
ros2 launch robot_status robot_status_launch.py web_port:=8080

# Check firewall settings
sudo ufw allow 8005
```

### Parameter Not Found Error

Make sure you set the status before trying to get it:

```bash
# Set first
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'test', value: 'hello'}"

# Then get
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot1', key: 'test'}"
```

## API Reference

### Service Definitions

**SetStatus.srv**
```
string ns           # Namespace identifier
string key          # Status key
string value        # Value (string or JSON)
---
bool success        # Operation result
string message      # Result message
```

**DeleteStatus.srv**
```
string ns           # Namespace identifier
string key          # Status key to delete (empty = delete namespace)
---
bool success        # Operation result
string message      # Result message
```

**ListStatus.srv**
```
string ns           # Namespace filter (empty = all)
---
bool success        # Operation result
string[] namespaces # List of namespace identifiers
string status_dict  # JSON dictionary of status
string message      # Summary message
```

## Performance Considerations

- **Concurrency**: Thread-safe operations allow multiple simultaneous requests
- **Memory**: Status stored in-memory as ROS2 parameters
- **Scalability**: Tested with multiple robots and high-frequency updates
- **Network**: Web dashboard uses polling (2s interval) - adjust if needed

## License

This package is part of the robot_dc workspace.

## Contributing

When extending this package:
1. Maintain backward compatibility with existing service interfaces
2. Add thread safety for any new shared state
3. Update this README with new features
4. Include example usage in the examples/ directory

## Support

For issues or questions:
1. Check the troubleshooting section
2. Review example scripts
3. Examine log files in ~/.ros/log/
