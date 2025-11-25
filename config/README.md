# Robot Configuration

This directory contains the centralized configuration for all robots and services in the robot_dc workspace.

## Files

- **robot_config.example.yaml** - Template configuration file (committed to git)
- **robot_config.yaml** - Actual configuration with real values (**NOT committed to git**)

## Setup Instructions

### For New Users/Deployments

1. Copy the example configuration:
   ```bash
   cd robot_dc/config
   cp robot_config.example.yaml robot_config.yaml
   ```

2. Edit `robot_config.yaml` with your actual values:
   ```bash
   nano robot_config.yaml
   ```

3. Update the following based on your environment:
   - Robot IP addresses
   - Camera IPs and RTSP URLs (including passwords)
   - Service ports (if different from defaults)
   - Network addresses

4. The configuration will be automatically loaded by all services

## Configuration Structure

The configuration is organized by robot namespace:

```yaml
# UR15 Robot Configuration
ur15:
  robot:
    type: "ur15"
    ip: "192.168.1.15"
    ports: {...}
  camera: {...}
  web: {...}
  calibration: {...}

# Duco Robot Configuration  
duco:
  network: {...}
  camera: {...}
  robot: {...}

# Services Configuration
services:
  image_labeling: {...}
  positioning_3d: {...}
  robot_status_redis: {...}

# Shared/Common Configuration
shared:
  network:
    courier: {...}
    ffpp_server: {...}
```

## Usage Examples

### Quick Start

```python
from common.config_manager import ConfigManager

# Get singleton instance
config = ConfigManager()

# Access robot-specific configuration
ur15 = config.get_robot('ur15')
ur15_ip = ur15.get('robot.ip')
print(f"UR15 IP: {ur15_ip}")
```

### Example 1: Using in ROS2 Launch Files

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from common.config_manager import ConfigManager

def generate_launch_description():
    # Load configuration
    config = ConfigManager()
    ur15 = config.get_robot('ur15')
    
    # Declare arguments with defaults from config
    ur15_ip_arg = DeclareLaunchArgument(
        'ur15_ip',
        default_value=ur15.get('robot.ip'),
        description='IP address of the UR15 robot'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value=str(ur15.get('web.port')),
        description='Web server port'
    )
    
    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    web_port = LaunchConfiguration('web_port')
    
    return LaunchDescription([
        ur15_ip_arg,
        web_port_arg,
        Node(
            package='ur15_web',
            executable='ur15_web_node',
            parameters=[{
                'ur15_ip': ur15_ip,
                'web_port': web_port,
            }]
        )
    ])
```

### Example 2: Using in ROS2 Nodes

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from common.config_manager import ConfigManager

class UR15WebNode(Node):
    def __init__(self):
        super().__init__('ur15_web_node')
        
        # Load configuration
        config = ConfigManager()
        ur15 = config.get_robot('ur15')
        
        # Declare parameters with defaults from config
        self.declare_parameter('ur15_ip', ur15.get('robot.ip'))
        self.declare_parameter('web_port', ur15.get('web.port'))
        
        # Get parameter values
        self.ur15_ip = self.get_parameter('ur15_ip').value
        self.web_port = self.get_parameter('web_port').value
        
        self.get_logger().info(f"Connecting to UR15 at {self.ur15_ip}")
        self.get_logger().info(f"Web interface on port {self.web_port}")

def main():
    rclpy.init()
    node = UR15WebNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Example 3: Path Resolution

```python
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
from pathlib import Path

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Get paths from config
dataset_path = ur15.get('web.dataset_path')
calibration_path = ur15.get('web.calibration_data_path')

# Convert relative paths to absolute
workspace_root = Path(get_workspace_root())
if not Path(dataset_path).is_absolute():
    dataset_path = workspace_root / dataset_path
if not Path(calibration_path).is_absolute():
    calibration_path = workspace_root / calibration_path

print(f"Dataset: {dataset_path}")
print(f"Calibration: {calibration_path}")
```

### Example 4: Multi-Robot Coordination

```python
from common.config_manager import ConfigManager

config = ConfigManager()

# Work with multiple robots
for robot_name in config.list_robots():
    robot = config.get_robot(robot_name)
    robot_ip = robot.get('robot.ip')
    print(f"{robot_name}: {robot_ip}")

# Access shared services
positioning_config = config.get('services.positioning_3d')
print(f"Positioning service on port {positioning_config['port']}")
```

### Example 5: Accessing Camera Configuration

```python
from common.config_manager import ConfigManager

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Get camera configuration
camera = ur15.get('camera')
camera_ip = camera['ip']
rtsp_url = camera['rtsp_url']
server_port = camera['server_port']

# Or with dot notation
camera_name = ur15.get('camera.name')
camera_topic = ur15.get('camera.topic')
```

### Example 6: Checking Configuration Existence

```python
from common.config_manager import ConfigManager

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Check if key exists before using
if ur15.has('camera.rtsp_url'):
    rtsp_url = ur15.get('camera.rtsp_url')
    print(f"RTSP URL: {rtsp_url}")
else:
    print("No camera configured")

# Use default value
timeout = ur15.get('network.timeout', default=10)
```

## API Reference

### ConfigManager

```python
config = ConfigManager()
```

**Methods:**

- **`get(key: str, default=None) -> Any`**  
  Get configuration value using dot notation (e.g., 'services.positioning_3d.port')
  
- **`has(key: str) -> bool`**  
  Check if configuration key exists
  
- **`get_robot(robot_name: str) -> RobotConfig`**  
  Get robot-specific configuration wrapper
  
- **`list_robots() -> List[str]`**  
  Get list of configured robots (e.g., ['ur15', 'duco'])
  
- **`get_all() -> dict`**  
  Get entire configuration dictionary
  
- **`reload()`**  
  Reload configuration from file (useful for development)

### RobotConfig

```python
robot = config.get_robot('ur15')
```

**Methods:**

- **`get(key: str, default=None) -> Any`**  
  Get robot configuration value using dot notation (e.g., 'robot.ip')
  
- **`has(key: str) -> bool`**  
  Check if robot configuration key exists
  
- **`get_all() -> dict`**  
  Get all configuration data for this robot

## Common Patterns

### Pattern 1: Configuration with Parameter Override

```python
# Config provides default, ROS parameter can override
config = ConfigManager()
ur15 = config.get_robot('ur15')

self.declare_parameter('ur15_ip', ur15.get('robot.ip'))
self.ur15_ip = self.get_parameter('ur15_ip').value
```

### Pattern 2: Multi-Level Access

```python
# Get entire section
camera_config = ur15.get('camera')
camera_ip = camera_config['ip']

# Or use dot notation
camera_ip = ur15.get('camera.ip')
```

### Pattern 3: Service Configuration

```python
# Access service configuration
positioning_config = config.get('services.positioning_3d')
host = positioning_config['host']
port = positioning_config['port']

# Or with dot notation
port = config.get('services.positioning_3d.port')
```

## Security Notes

⚠️ **IMPORTANT**: 
- `robot_config.yaml` contains sensitive information (IPs, passwords)
- This file is in `.gitignore` and should **NEVER** be committed to git
- Only commit `robot_config.example.yaml` with placeholder values
- Use environment variables for highly sensitive data

## Environment Variables

You can use environment variable expansion in the config:

```yaml
network:
  cameras:
    ur15_camera:
      rtsp_url: "rtsp://admin:${CAMERA_PASSWORD}@192.168.1.101/stream0"
```

Then set the environment variable:
```bash
export CAMERA_PASSWORD="your_password"
```

## Adding New Robots

To add a new robot configuration:

1. Add a new top-level section in `robot_config.yaml`:
   ```yaml
   ur10:
     network:
       robot_ip: "192.168.1.20"
     services:
       web: 8040
   ```

2. Use in code:
   ```python
   ur10 = config.get_robot('ur10')
   ```

## Testing

### Unit Testing with Mock Configuration

```python
import pytest
from unittest.mock import patch, MagicMock
from common.config_manager import ConfigManager

@pytest.fixture
def mock_config():
    """Fixture providing mock configuration for testing"""
    config_data = {
        'ur15': {
            'robot': {
                'ip': '192.168.1.15',
                'type': 'ur15'
            },
            'web': {
                'port': 8030
            }
        },
        'services': {
            'positioning_3d': {
                'host': 'localhost',
                'port': 8004
            }
        }
    }
    return config_data

def test_node_initialization(mock_config):
    """Test node initialization with mock config"""
    with patch('common.config_manager.ConfigManager.get_all', 
               return_value=mock_config):
        config = ConfigManager()
        ur15 = config.get_robot('ur15')
        
        assert ur15.get('robot.ip') == '192.168.1.15'
        assert ur15.get('web.port') == 8030

def test_with_config_reload():
    """Test configuration reload functionality"""
    config = ConfigManager()
    original_value = config.get('ur15.robot.ip')
    
    # Modify config file here if needed
    config.reload()
    
    new_value = config.get('ur15.robot.ip')
    # Assert expected behavior after reload
```

### Integration Testing

```python
def test_launch_with_config():
    """Test launch file with actual configuration"""
    from common.config_manager import ConfigManager
    
    config = ConfigManager()
    ur15 = config.get_robot('ur15')
    
    # Verify configuration is properly loaded
    assert ur15.has('robot.ip')
    assert ur15.has('web.port')
    
    # Test that values are valid
    ur15_ip = ur15.get('robot.ip')
    assert isinstance(ur15_ip, str)
    assert len(ur15_ip) > 0
```

## Troubleshooting

### Config file not found
- Ensure `robot_config.yaml` exists in `robot_dc/config/`
- Check that `ROBOT_DC_ROOT` environment variable is set (optional)
- Copy from `robot_config.example.yaml` if needed

### Import errors
- Rebuild the common package: `colcon build --packages-select common`
- Source the workspace: `source install/setup.bash`

### Changes not reflected
```python
config = ConfigManager()
config.reload()  # Force reload from file
```

After modifying the config file, you may also need to rebuild:
```bash
cd colcon_ws
colcon build --packages-select common
source install/setup.bash
```

### Python cache issues
If changes to config-related code aren't taking effect:
```bash
cd colcon_ws
find . -type d -name __pycache__ -exec rm -rf {} +
colcon build --packages-select common --cmake-clean-cache
source install/setup.bash
```

### Path resolution issues
- Relative paths in config are resolved relative to workspace root
- Use `common.workspace_utils.get_workspace_root()` for workspace-relative paths
- Check that paths exist before use:
  ```python
  from pathlib import Path
  dataset_path = Path(ur15.get('web.dataset_path'))
  if not dataset_path.exists():
      dataset_path.mkdir(parents=True, exist_ok=True)
  ```

### Validation errors
- Check YAML syntax with: `python3 -c "import yaml; yaml.safe_load(open('robot_config.yaml'))"`
- Ensure all required fields are present (compare with example)
