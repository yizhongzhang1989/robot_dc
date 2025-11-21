# Test Web Service

A simple Flask-based web service for testing purposes.

## Features

- Simple web interface with "Test Web" title
- Test button that triggers a POST request
- Visual feedback on button click
- Runs on port 8001 (configurable)

## Installation

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select test_web
source install/setup.bash
```

## Usage

### Launch with default settings (port 8001)

```bash
ros2 launch test_web test_web_launch.py
```

### Launch with custom port

```bash
ros2 launch test_web test_web_launch.py port:=8080
```

### Access the web interface

Open your browser and navigate to:

```
http://localhost:8001
```

## Parameters

- **port** (int, default: 8001): Port for the web service
- **host** (string, default: '0.0.0.0'): Host address for the web service

## License

MIT
