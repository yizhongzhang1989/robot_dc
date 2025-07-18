# URDF Web Viewer

A ROS2 package that provides a web-based interface for visualizing URDF (Unified Robot Description Format) files with real-time joint state updates.

## Features

- **Web-based 3D Visualization**: Interactive 3D rendering of URDF models in a web browser
- **Real-time Joint Updates**: Displays current joint states from ROS2 topics
- **STL Mesh Support**: Loads and displays STL mesh files referenced in URDF
- **REST API**: Provides HTTP endpoints for URDF data, joint states, and mesh files
- **Standalone Operation**: Can run independently without external dependencies
- **ROS2 Integration**: Subscribes to joint state topics for live updates

## Architecture

The package consists of:
- **HTTP Server**: Serves the web interface and provides REST API endpoints
- **ROS2 Node**: Handles joint state subscriptions and ROS2 integration
- **URDF Parser**: Parses URDF files and extracts joint information
- **Mesh Loader**: Loads and encodes STL mesh files for web delivery
- **Threading**: Non-blocking server architecture for smooth ROS2 operation

## Installation

### Prerequisites

- ROS2 Humble or later
- Python 3.8+
- Web browser with WebGL support

### Build Instructions

1. Clone the repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select urdf_web_viewer
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Method 1: ROS2 Launch (Recommended)

Launch the web viewer with default settings:
```bash
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py
```

Launch with custom URDF file and port:
```bash
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py \
  urdf_file:=/path/to/your/robot.urdf \
  port:=8080
```

### Method 2: Direct Python Execution

Run the server directly:
```bash
python3 src/urdf_web_viewer/urdf_web_viewer/urdf_web_server.py
```

### Method 3: ROS2 Run

Execute as a ROS2 node:
```bash
ros2 run urdf_web_viewer urdf_web_server
```

## Configuration

### Launch Parameters

- `urdf_file`: Full path to the URDF file to visualize
  - Default: `/home/a/Documents/robot_dc/colcon_ws/src/duco_gcr5_910_urdf/urdf/duco_gcr5_910_urdf.urdf`
- `port`: Web server port number
  - Default: `8080`

### Environment Variables

The server automatically detects URDF files and mesh directories based on the package structure.

## Web Interface

Once running, open your web browser and navigate to:
```
http://localhost:8080
```

### Available Endpoints

- `GET /`: Main web interface
- `GET /urdf`: Returns the URDF file content
- `GET /joints`: Returns joint information as JSON
- `GET /meshes/<filename>`: Returns base64-encoded mesh data
- `GET /joint_states`: Returns current joint states as JSON

### Web Interface Features

- **3D Visualization**: Interactive 3D model with mouse controls
- **Joint State Display**: Real-time joint position updates
- **Mesh Rendering**: Proper STL mesh display with materials
- **Responsive Design**: Works on desktop and mobile browsers

## ROS2 Integration

The package integrates with ROS2 through:

### Subscribed Topics

- `/joint_states` (sensor_msgs/JointState): Joint state information

### Published Topics

None (read-only visualization)

### Services

None

## File Structure

```
urdf_web_viewer/
├── launch/
│   └── urdf_web_viewer.launch.py    # ROS2 launch file
├── resource/
│   └── urdf_web_viewer              # Package marker
├── third_party/
│   └── urdf-loaders/                # URDF loader library
├── urdf_web_viewer/
│   ├── __init__.py
│   └── urdf_web_server.py           # Main server implementation
├── package.xml                      # Package metadata
├── setup.py                         # Package setup
└── README.md                        # This file
```

## Technical Details

### Threading Architecture

The server uses a threaded architecture to prevent blocking:
- **Main Thread**: Runs the ROS2 node and handles ROS2 callbacks
- **HTTP Server Thread**: Handles web requests in a separate daemon thread
- **Graceful Shutdown**: Proper cleanup of both threads on termination

### URDF Processing

1. **File Loading**: Reads URDF file from specified path
2. **Joint Extraction**: Parses joint definitions and limits
3. **Mesh Loading**: Loads STL files and encodes them as base64
4. **State Management**: Maintains current joint states for visualization

### Web Technologies

- **Backend**: Python HTTP server with custom request handlers
- **Frontend**: HTML5 with WebGL for 3D rendering
- **3D Library**: Three.js with URDF loader extensions
- **Data Format**: JSON for API responses, base64 for binary data

## Troubleshooting

### Common Issues

1. **Port Already in Use**
   - Change the port using the `port` parameter
   - Check for other services using the same port

2. **URDF File Not Found**
   - Verify the file path is correct and accessible
   - Check file permissions

3. **Mesh Files Not Loading**
   - Ensure STL files are in the correct directory
   - Check that mesh paths in URDF are relative to the package

4. **ROS2 Integration Issues**
   - Verify ROS2 environment is properly sourced
   - Check that joint state topics are being published

### Debug Mode

Enable debug logging by modifying the server code:
```python
logging.basicConfig(level=logging.DEBUG)
```

## Development

### Adding New Features

1. **New Endpoints**: Add handlers to the `URDFRequestHandler` class
2. **ROS2 Integration**: Extend the `URDFWebServer` class
3. **Web Interface**: Modify the HTML/JavaScript in the static file handler

### Testing

Test the server functionality:
```bash
# Test basic connectivity
curl http://localhost:8080/urdf

# Test joint states
curl http://localhost:8080/joint_states

# Test mesh loading
curl http://localhost:8080/meshes/base_link.STL
```

## License

This package is licensed under the MIT License. See the package.xml file for details.

## Contributing

Contributions are welcome! Please ensure:
- Code follows ROS2 conventions
- New features include appropriate documentation
- Changes are tested with real URDF files

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS2 logs for error messages
3. Verify URDF file validity
4. Test with simplified URDF files

## Changelog

### Version 1.0.0
- Initial release
- Basic URDF visualization
- ROS2 integration
- Threading architecture
- STL mesh support
- REST API endpoints
