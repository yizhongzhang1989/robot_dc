# Lift Robot System Guide

## Deployment Instructions

### System Requirements

- **Operating System**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS2 Version**: Humble Hawksbill

### Installation Steps

#### 1. Clone Repository

```bash
git clone https://github.com/yizhongzhang1989/robot_dc.git
cd robot_dc
```

#### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

**Note**: `nlopt` in requirements.txt is optional and can be skipped if not needed.

#### 3. Build ROS2 Workspace

```bash
cd colcon_ws
colcon build
source install/setup.bash
```

#### 4. Configure System

Copy example configuration and modify as needed:

```bash
cd ..
cp config/robot_config.example.yaml config/robot_config.yaml
# Edit robot_config.yaml with your device settings
```

**Note**: `robot_config.yaml` is gitignored and must be created locally.

#### 5. Launch System

Refer to the **System Launch** section below for available launch methods.

## System Overview

Lift Robot is a ROS2-based elevator platform control system that integrates Modbus communication, multi-sensor data acquisition, and web monitoring interface.

## Node Architecture

### Core Nodes

```
lift_robot_system
├── modbus_driver (Modbus RS485 Communication Driver)
│   ├── modbus_manager_node (Modbus Master Manager)
│   └── modbus_dashboard (Optional: Web Dashboard, Port 5000)
│
├── lift_robot_platform (Elevator Platform Control)
│   └── lift_robot_node_action (Platform Motion Control, device_id=50)
│
├── draw_wire_sensor (Draw Wire Sensor)
│   └── draw_wire_sensor_node (Height Measurement, device_id=51)
│
├── lift_robot_force_sensor (Force Sensor, Dual Channel)
│   ├── force_sensor_node_right (Right Force Sensor, device_id=52/53)
│   └── force_sensor_node_left (Left Force Sensor, device_id=53/52)
│
└── lift_robot_web (Web Monitoring Interface)
    ├── server (HTTP/WebSocket Server, Port 8090)
    └── cmd_processor (Command Queue Processor)
```

### Node Functions

#### 1. **modbus_driver**
- **Function**: Modbus RTU (RS485) master, manages all slave device communication
- **Serial Port**: Auto-detect or manual specification (e.g., `/dev/ttyUSB0`)
- **Baudrate**: 115200
- **Optional Dashboard**: Real-time Modbus communication monitoring

#### 2. **lift_robot_platform**
- **Function**: Elevator platform position control
- **Topics**:
  - `/lift_robot_platform/command` (Subscribe) - Receives control commands
  - `/lift_robot_platform/status` (Publish) - Publishes platform status
- **Calibration Files**: 
  - `platform_overshoot_calibration.json` - Overshoot compensation
  - `platform_range.json` - Travel range limits

#### 3. **draw_wire_sensor**
- **Function**: Draw wire displacement sensor for platform height measurement
- **Topic**: `/draw_wire_sensor/data` (Publishes String JSON)
- **Sampling Rate**: ~17Hz (read_interval=0.06s)
- **Calibration File**: `draw_wire_calibration.json`

#### 4. **lift_robot_force_sensor**
- **Function**: Dual-channel force sensors for platform load monitoring
- **Topics**:
  - `/force_sensor_right` (Publish Float32) - Right force value
  - `/force_sensor_left` (Publish Float32) - Left force value
  - `/force_sensor_right/raw` (Publish Float32) - Raw value (for calibration)
  - `/force_sensor_left/raw` (Publish Float32) - Raw value (for calibration)
- **Device ID Mapping**: Can swap left/right sensor mapping in configuration
- **Calibration Files**: 
  - `force_sensor_calibration_right.json`
  - `force_sensor_calibration_left.json`

#### 5. **lift_robot_web**
- **Function**: Web monitoring and control interface
- **Port**: 8090 (HTTP) + WebSocket
- **Features**:
  - Real-time sensor data visualization
  - Platform control command sending
  - Sensor calibration tools
  - Platform travel limit settings

## Launch Methods

### Method 1: Full System Launch (Recommended)

Launch all nodes (platform + sensors + web):

```bash
cd colcon_ws
source install/setup.bash
ros2 launch robot_bringup lift_robot_bringup.py
```

**Launch with Parameters** (override config file):

```bash
# Specify serial port
ros2 launch robot_bringup lift_robot_bringup.py modbus_port:=/dev/ttyUSB1

# Enable Modbus dashboard
ros2 launch robot_bringup lift_robot_bringup.py enable_dashboard:=true

# Change force sensor device IDs (swap left/right)
ros2 launch robot_bringup lift_robot_bringup.py \
  force_right_device_id:=53 force_left_device_id:=52

# Change sampling rate to 50Hz
ros2 launch robot_bringup lift_robot_bringup.py \
  draw_wire_interval:=0.02 \
  force_right_interval:=0.02 \
  force_left_interval:=0.02

# Change web server port
ros2 launch robot_bringup lift_robot_bringup.py web_port:=8091

# Disable web interface
ros2 launch robot_bringup lift_robot_bringup.py web_enabled:=false

# Combined parameters example
ros2 launch robot_bringup lift_robot_bringup.py \
  modbus_port:=/dev/ttyUSB0 \
  enable_dashboard:=true \
  web_port:=8091
```

### Method 2: Platform Only Launch (Without Force Sensors)

Launch only platform and draw wire sensor:

```bash
ros2 launch robot_bringup lift_robot_platform_only.py
```

### Method 3: Launch Individual Nodes

Launch nodes separately for testing or debugging:

**Modbus Driver Only**:
```bash
ros2 launch modbus_driver modbus_manager_launch.py
ros2 launch modbus_driver modbus_manager_launch.py enable_dashboard:=true
```

**Platform Only**:
```bash
ros2 launch lift_robot_platform lift_robot_launch.py
ros2 launch lift_robot_platform lift_robot_launch.py device_id:=50
```

**Draw Wire Sensor Only**:
```bash
ros2 launch draw_wire_sensor draw_wire_sensor.launch.py
ros2 launch draw_wire_sensor draw_wire_sensor.launch.py device_id:=51 read_interval:=0.02
```

**Force Sensor Only**:
```bash
# Right sensor
ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py \
  device_id:=52 topic_name:=/force_sensor_right node_name_suffix:=right

# Left sensor
ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py \
  device_id:=53 topic_name:=/force_sensor_left node_name_suffix:=left
```

**Web Interface Only**:
```bash
ros2 launch lift_robot_web lift_robot_web.launch.py
ros2 launch lift_robot_web lift_robot_web.launch.py port:=8091
```

## Parameter Modification Guide

### 1. Modify Device ID Mapping

**Scenario**: Replace sensor hardware or swap left/right sensors

Edit `config/robot_config.yaml`:

```yaml
force_sensor_right:
  device_id: 53        # Change to physical left sensor ID
  
force_sensor_left:
  device_id: 52        # Change to physical right sensor ID
```

### 2. Modify Sampling Frequency

Edit `config/robot_config.yaml`:

```yaml
draw_wire_sensor:
  read_interval: 0.02  # Change to 50Hz (was 0.06 = 17Hz)
  
force_sensor_right:
  read_interval: 0.02  # Change to 50Hz
```

**Note**: High frequency increases Modbus bus load and may affect other devices.

### 3. Modify Web Port

Edit `config/robot_config.yaml`:

```yaml
web:
  port: 8091  # Change to another port
```

### 4. Enable Modbus Dashboard

Edit `config/robot_config.yaml`:

```yaml
modbus_driver:
  dashboard:
    enabled: true      # Change to true
    port: 5000         # Dashboard port
```

Access: `http://localhost:5000`

### 5. Serial Port Configuration

#### Auto-Detection (Default)

The system automatically detects the RS485 serial port by default:

```yaml
modbus_driver:
  port: "auto"  # Default: auto-detection
```

**Auto-Detection Algorithm**:
1. **Priority 1**: Scans for USB serial devices with "485" or "RS485" in description
2. **Priority 2**: Uses first `/dev/ttyUSB*` device if no RS485 keyword found
3. **Fallback**: Uses `/dev/ttyUSB0` if no USB serial devices detected

**Check Detection Result** in logs:
```
[modbus_manager_launch] Detected RS485 device: /dev/ttyUSB0 (USB-SERIAL CH340)
```

#### Check Available Serial Ports

```bash
# List all USB serial devices
ls /dev/ttyUSB*

# View device details with pyserial
python3 -c "import serial.tools.list_ports as lp; [print(f'{p.device}: {p.description}') for p in lp.comports()]"
```

### Calibration Files

Calibration files are located in **`colcon_ws/config/`**, automatically generated by the web interface:

- `draw_wire_calibration.json` - Draw wire sensor calibration
- `force_sensor_calibration_right.json` - Right force sensor calibration
- `force_sensor_calibration_left.json` - Left force sensor calibration
- `platform_overshoot_calibration.json` - Platform overshoot compensation
- `platform_range.json` - Platform travel range


## Sensor Calibration Procedures

### 1. Draw Wire Sensor Calibration

1. Open web interface: `http://localhost:8090`
2. Navigate to "Sensor Calibration" page
3. Move platform to a known height position
4. Input actual height value
5. Click "Add Calibration Point"
6. Repeat steps 3-5, collect at least 2 calibration points
7. Click "Calculate Calibration", system automatically saves to `draw_wire_calibration.json`

### 2. Force Sensor Calibration
click tare button before calibrating
1. Open web interface: `http://localhost:8090`
2. Navigate to "Force Sensor Calibration" page
3. Select sensor channel (right/left)
4. Place known weight on sensor
5. Input actual weight value (kg)
6. Click "Add Calibration Point"
7. Repeat steps 3-5, collect at least 1 calibration points
8. Click "Calculate Calibration", system automatically saves to corresponding JSON file

### 3. Platform Range Detection

1. Open web interface: `http://localhost:8090`
2. Navigate to "Platform Calibration" page
3. Click "Start" button in **Platform Range Detection** section
4. System automatically detects platform travel range and saves to `platform_range.json`

### 4. Platform Overshoot Calibration

1. Open web interface: `http://localhost:8090`
2. Navigate to "Platform Calibration" page
3. Click "Start" button in **Overshoot Calibration** section
4. Wait for system to complete multiple movement cycles
5. Click "Show Plot" to visualize overshoot data
6. Click "Remove Outliers" to clean data
7. Select degree `2` for polynomial fitting
8. Click "Fit" to generate calibration curve
9. Click "Save Fit" to save calibration to `platform_overshoot_calibration.json`

## Development and Debugging

### View Node Status

```bash
# List all nodes
ros2 node list

# View topics
ros2 topic list

# Monitor sensor data
ros2 topic echo /force_sensor_right
ros2 topic echo /draw_wire_sensor/data
```

### View Parameters

```bash
# View force sensor parameters
ros2 param list /lift_robot_force_sensor
ros2 param get /lift_robot_force_sensor device_id

# View platform parameters
ros2 param list /lift_robot_platform
```

### Send Test Commands

```bash
# Move platform to 100mm
ros2 topic pub --once /lift_robot_platform/command std_msgs/msg/String \
  '{"data": "{\"action\": \"move_to\", \"target\": 100}"}'

# Home platform
ros2 topic pub --once /lift_robot_platform/command std_msgs/msg/String \
  '{"data": "{\"action\": \"home\"}"}'
```

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────┐
│                  Web Interface (8090)                    │
│  Monitor / Control / Calibration                        │
└───────────────────┬─────────────────────────────────────┘
                    │ ROS2 Topics
┌───────────────────┴─────────────────────────────────────┐
│              ROS2 Node Layer                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────────┐  │
│  │ Platform │  │DrawWire  │  │ ForceSensor (x2)     │  │
│  │  Node    │  │  Node    │  │  Right / Left        │  │
│  └─────┬────┘  └─────┬────┘  └──────────┬───────────┘  │
└────────┼─────────────┼──────────────────┼──────────────┘
         │             │                  │
┌────────┴─────────────┴──────────────────┴──────────────┐
│              Modbus Manager Node                         │
│  (Modbus RTU Master, RS485)                             │
└───────────────────────┬─────────────────────────────────┘
                        │ Modbus RTU
         ┌──────────────┼──────────────┐
         │              │              │
    ┌────┴───┐    ┌─────┴────┐   ┌────┴────┐
    │Platform│    │DrawWire  │   │ Force   │
    │ ID=50  │    │  ID=51   │   │ ID=52/53│
    └────────┘    └──────────┘   └─────────┘
      Motor       Draw Wire        Force
                   Sensor          Sensor
```

## Maintainers

Robot DC Team

Last Updated: 2025-12-01
