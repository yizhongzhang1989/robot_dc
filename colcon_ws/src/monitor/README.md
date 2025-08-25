# Robot Monitor Node

## Overview

The Robot Monitor Node is a ROS2 node that listens to robot UDP data streams and automatically records them to rosbag files. This node implements the same functionality as the `dk_test.py` script but adds ROS2 integration and automatic data recording capabilities.

## Features

- ✅ **UDP Data Listening**: Listens simultaneously on ports 5566 (robot data) and 5577 (log data)
- ✅ **Port Sharing**: Uses SO_REUSEPORT to share port 5566 with robot_arm_web_server
- ✅ **Automatic Timestamps**: Adds nanosecond-level timestamps to all received data
- ✅ **ROS2 Topic Publishing**: 
  - `/robot_data` - Robot status data (TCP position, joint angles, force sensors, etc.)
  - `/log_data` - System logs and status messages (optional, for debugging and status monitoring)
- ✅ **Automatic rosbag Recording**: Automatically starts rosbag recording and saves to specified directory
- ✅ **Configurable Storage**: Supports custom data storage locations
- ✅ **Session Information**: Automatically generates session metadata files
- ✅ **Data Compression**: Uses zstd compression to save storage space

## System Requirements

- ROS2 Humble
- Python 3.8+
- Linux system with SO_REUSEPORT support

## Quick Start

### 1. Build and Install

```bash
# Enter workspace and build
cd /path/to/robot_dc3/colcon_ws
colcon build --packages-select monitor
source install/setup.bash
```

### 2. Start Monitoring

```bash
# Use default storage location (~/robot_data)
ros2 launch monitor monitor.launch.py

# Specify custom storage location
ros2 launch monitor monitor.launch.py data_dir:=/your/custom/path

# Or set environment variable
export ROBOT_DATA_DIR=/your/custom/path
ros2 launch monitor monitor.launch.py
```

### 3. Data Management

```bash
# View all sessions
python3 scripts/robot_monitor_manager.py --list --detailed

# Analyze specific session
python3 scripts/robot_monitor_manager.py --analyze ~/robot_data/2025-08-25/robot_monitor_HHMMSS

# View data content
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 5 --full

# View raw binary data format
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 2 --raw
```

## Runtime Information

### Node Startup Logs

After the Monitor node starts, it will display the following information in real-time in the terminal:

```
[INFO] [robot_monitor_node]: Robot Monitor Node started
[INFO] [robot_monitor_node]: Data will be saved to: /home/user/robot_data
[INFO] [robot_monitor_node]: Rosbag will be saved to: /home/user/robot_data/2025-08-25/robot_monitor_134753
[INFO] [robot_monitor_node]: Session info saved to: /home/user/robot_data/2025-08-25/robot_monitor_134753_info.json
[INFO] [robot_monitor_node]: Started rosbag recording with PID: 12345
[INFO] [robot_monitor_node]: Listening on ports 5566 (data) and 5577 (log)
[INFO] [robot_monitor_node]: Using SO_REUSEPORT to share port 5566 with robot_arm_web_server
[INFO] [robot_monitor_node]: Data receiver bound to 0.0.0.0:5566
[INFO] [robot_monitor_node]: Log receiver bound to 0.0.0.0:5577
[INFO] [robot_monitor_node]: [DATA] From 192.168.1.100:5566 - RobotTcpPos: [-98.85, 658.5, 500.85, -3.81, 0.13, -170.38]
[INFO] [robot_monitor_node]: [LOG] From 192.168.1.100:5577: System status OK
```

> ✅ **Real-time Monitoring**: When the node is running, it automatically displays received data in the terminal, no additional commands needed to view real-time data streams.

### Data Content Examples

**Robot Data** (`/robot_data` topic) - Main data stream:
- **RobotTcpPos**: TCP position [x, y, z, rx, ry, rz] (mm and degrees)
- **RobotAxis**: Joint angles [J1-J6] (degrees)
- **RobotTrack**: Rail position
- **FTSensorData**: Force/torque sensor [Fx, Fy, Fz, Mx, My, Mz]
- **FTTarget**: Force/torque target values

**Log Data** (`/log_data` topic) - Auxiliary data stream (optional):
- System log messages and events
- Error notifications and status updates
- Debug information

> 📝 **Note**: In actual use, focus mainly on the `/robot_data` topic. The `/log_data` topic is for system logs and may have no data or less data.

## Data Storage

### Storage Structure

```
Data Root Directory/
├── 2025-08-25/
│   ├── robot_monitor_134753/          # rosbag data directory
│   │   ├── robot_monitor_134753_0.db3.zstd # Compressed SQLite database file
│   │   └── metadata.yaml              # rosbag metadata
│   └── robot_monitor_134753_info.json # Session information file
├── 2025-08-26/
└── ...
```

### Storage Location Configuration

**Method 1: Environment Variable (Recommended)**
```bash
export ROBOT_DATA_DIR=/your/custom/path
```

**Method 2: Launch Parameter**
```bash
ros2 launch monitor monitor.launch.py data_dir:=/your/custom/path
```

**Method 3: View Current Configuration**
```bash
python3 scripts/robot_monitor_manager.py --config
```

### Session Information File Example

```json
{
  "session_start": "2025-08-25T13:47:53.123456",
  "bag_name": "robot_monitor_134753",
  "data_sources": {
    "robot_data_port": 5566,
    "log_data_port": 5577
  },
  "topics": ["/robot_data", "/log_data"]
}
```

## Management Tools

### robot_monitor_manager.py - Unified Data Management

**Main Features**: Configuration management, session listing, data analysis, content viewing

```bash
# Basic operations
python3 scripts/robot_monitor_manager.py --config           # View configuration
python3 scripts/robot_monitor_manager.py --list --detailed  # List sessions
python3 scripts/robot_monitor_manager.py --test             # Test connection

# Data analysis
python3 scripts/robot_monitor_manager.py --analyze ~/robot_data/2025-08-25/robot_monitor_HHMMSS

# Data viewing
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 5 --full
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --topic /log_data  # View logs (may have no data)

# View raw data format
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 2 --raw  # Hexadecimal raw data

# View database structure and timestamp storage details
python3 scripts/robot_monitor_manager.py --db-info ~/robot_data/2025-08-25/robot_monitor_HHMMSS

# Data cleanup
python3 scripts/robot_monitor_manager.py --cleanup 30          # Preview cleanup
python3 scripts/robot_monitor_manager.py --cleanup-confirm 30  # Confirm cleanup
```

### test_udp_send.py - Test Data Sending Tool

```bash
python3 scripts/test_udp_send.py -n 20 -i 0.5  # Send 20 messages with 0.5s interval
```

> 💡 **Note**: This tool is for development and testing environments. In actual deployment, the robot will send UDP data directly.

## Data Viewing

### Real-time Monitoring

When the monitor node is running, all received data will be displayed in real-time directly in the node terminal:

```
[INFO] [robot_monitor_node]: [DATA] 15:49:05.844 From 192.168.1.100:5566 - RobotTcpPos: [-41.04, 238.16, 652.56, 87.34, 6.03, 179.41], RobotAxis: [1.15, -0.45, 1.31, 0.81, -1.62, -1.95], FTSensorData: [0.84, 1.02, 29.48, 0.26, -0.56, 0.36]
[INFO] [robot_monitor_node]: [LOG] 15:49:06.123 From 192.168.1.100:5577: System status OK
```

### Timestamp Explanation

### Detailed Timestamp Information

**rosbag Timestamp Storage Location**:
- **Table**: `messages`
- **Field**: `timestamp` (INTEGER type, 64-bit)
- **Precision**: Nanosecond level (1756108145806574972 = 2025-08-25 15:49:05.806574972)
- **Source**: System time when ROS2 node receives UDP data packets

**Database Field Description**:

| Field Name  | Type      | Purpose                                     | Your Actual Data Example     |
|-------------|-----------|---------------------------------------------|------------------------------|
| `id`        | INTEGER   | Unique message identifier                   | 1, 2, 3...                   |
| `topic_id`  | INTEGER   | Topic ID (1=/robot_data, 2=/log_data)      | 1                            |
| `timestamp` | INTEGER   | **Timestamp (nanoseconds)**                | 1756108145806574972          |
| `data`      | BLOB      | **Robot data (binary)**                    | 232 bytes of robot data      |

**Why Use Reception Timestamp**:
1. Robot's raw data contains no timestamp, only position, angle, force sensor data
2. Reception timestamp more accurately reflects when data was recorded
3. Avoids network delay and clock synchronization issues

**View Complete Database Information**:
```bash
python3 scripts/robot_monitor_manager.py --db-info ~/robot_data/2025-08-25/robot_monitor_HHMMSS
```
Shows: database structure, timestamp storage details, message statistics, recording duration, etc.

**Raw Data Format**:
```json
{
  "RobotTcpPos": [-41.04, 238.16, 652.56, 87.34, 6.03, 179.41],
  "RobotAxis": [1.15, -0.45, 1.31, 0.81, -1.62, -1.95],
  "RobotTrack": 0,
  "FTSensorData": [0.85, 1.01, 29.2, 0.26, -0.58, 0.36],
  "FTTarget": [0, 0, 0, 0, 0, 0]
}
```

**Storage Format**:
```json
{
  "raw_message": "{Original robot JSON data}"
}
```
Timestamps are stored in the rosbag message header.

### View Historical Data

```bash
# View data content, supports compressed files
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 5 --full

# View raw binary data format (hexadecimal and ASCII)
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 2 --raw
```

> 💡 **Note**: When the Monitor node is running, it will print received data in real-time in the terminal:
> - `[DATA] timestamp From IP - RobotTcpPos: [...], RobotAxis: [...], FTSensorData: [...]`
> - `[LOG] timestamp From IP: message`

