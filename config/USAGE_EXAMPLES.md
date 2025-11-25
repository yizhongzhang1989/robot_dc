# ConfigManager Usage Examples

## Quick Start

```python
from common.config_manager import ConfigManager

# Get singleton instance
config = ConfigManager()

# Access robot-specific configuration
ur15 = config.get_robot('ur15')
ur15_ip = ur15.get('network.robot_ip')
print(f"UR15 IP: {ur15_ip}")
```

## Complete Examples

### Example 1: Using in ROS2 Launch Files

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from common.config_manager import ConfigManager

def generate_launch_description():
    # Load configuration
    config = ConfigManager()
    ur15 = config.get_robot('ur15')
    
    # Get configuration values
    ur15_ip = ur15.get('network.robot_ip')
    web_port = ur15.get('services.web')
    camera_topic = ur15.get('network.camera.topic')
    
    return LaunchDescription([
        Node(
            package='ur15_web',
            executable='ur15_web_node',
            parameters=[{
                'ur15_ip': ur15_ip,
                'port': web_port,
                'camera_topic': camera_topic
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
        
        # Get values with fallback to ROS parameters
        self.ur15_ip = self.declare_parameter(
            'ur15_ip',
            ur15.get('network.robot_ip')
        ).value
        
        self.camera_topic = self.declare_parameter(
            'camera_topic',
            ur15.get('network.camera.topic')
        ).value
        
        self.get_logger().info(f"Connecting to UR15 at {self.ur15_ip}")

def main():
    rclpy.init()
    node = UR15WebNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Example 3: Using in Standalone Scripts

```python
#!/usr/bin/env python3
from common.config_manager import ConfigManager

def main():
    # Load configuration
    config = ConfigManager()
    ur15 = config.get_robot('ur15')
    
    # Get robot connection info
    robot_ip = ur15.get('network.robot_ip')
    robot_port = ur15.get('network.ports.control')
    
    # Connect to robot
    print(f"Connecting to {robot_ip}:{robot_port}")
    # ... your code here

if __name__ == '__main__':
    main()
```

### Example 4: Multi-Robot Coordination

```python
from common.config_manager import ConfigManager

config = ConfigManager()

# Work with multiple robots
for robot_name in config.list_robots():
    robot = config.get_robot(robot_name)
    robot_ip = robot.get('network.robot_ip')
    print(f"{robot_name}: {robot_ip}")

# Access shared resources
redis_host = config.get('shared.network.redis.host')
ffpp_url = config.get('shared.network.ffpp_server.url')
```

### Example 5: Accessing Camera Configuration

```python
from common.config_manager import ConfigManager

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Get camera configuration
camera = ur15.get('network.camera')
camera_ip = camera['ip']
rtsp_url = camera['rtsp_url']
server_port = camera['server_port']

# Or with dot notation
camera_name = ur15.get('network.camera.name')
```

### Example 6: Path Resolution

```python
from common.config_manager import ConfigManager
from pathlib import Path

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Paths are automatically resolved to absolute paths
dataset_path = Path(ur15.get('paths.dataset'))
calibration_path = Path(ur15.get('paths.calibration_data'))

print(f"Dataset: {dataset_path}")
print(f"Calibration: {calibration_path}")
```

### Example 7: With Default Values

```python
from common.config_manager import ConfigManager

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Use defaults for optional values
custom_port = ur15.get('services.custom_service', default=9000)
timeout = ur15.get('network.timeout', default=10)
```

### Example 8: Checking Configuration Existence

```python
from common.config_manager import ConfigManager

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Check if key exists before using
if ur15.has('network.camera.rtsp_url'):
    rtsp_url = ur15.get('network.camera.rtsp_url')
    print(f"RTSP URL: {rtsp_url}")
else:
    print("No camera configured")
```

## API Reference

### ConfigManager

```python
config = ConfigManager()
```

- **`get(key: str, default=None) -> Any`**  
  Get configuration value using dot notation
  
- **`has(key: str) -> bool`**  
  Check if configuration key exists
  
- **`get_robot(robot_name: str) -> RobotConfig`**  
  Get robot-specific configuration wrapper
  
- **`list_robots() -> List[str]`**  
  Get list of configured robots
  
- **`get_all() -> dict`**  
  Get entire configuration dictionary
  
- **`reload()`**  
  Reload configuration from file (useful for development)

### RobotConfig

```python
robot = config.get_robot('ur15')
```

- **`get(key: str, default=None) -> Any`**  
  Get robot configuration value using dot notation
  
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

self.ur15_ip = self.declare_parameter(
    'ur15_ip',
    ur15.get('network.robot_ip')
).value
```

### Pattern 2: Multi-Level Access

```python
# Get entire section
network_config = ur15.get('network')
robot_ip = network_config['robot_ip']

# Or use dot notation
robot_ip = ur15.get('network.robot_ip')
```

### Pattern 3: Shared Configuration

```python
# Access shared configuration
redis_config = config.get('shared.network.redis')
host = redis_config['host']
port = redis_config['port']
```

## Testing

When writing tests, you can create temporary configurations:

```python
import tempfile
import yaml
from pathlib import Path
from unittest.mock import patch
from common.config_manager import ConfigManager

def test_with_custom_config():
    # Create temporary config
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / 'robot_config.yaml'
        
        test_config = {
            'version': '1.0',
            'ur15': {
                'network': {'robot_ip': '192.168.1.99'}
            }
        }
        
        with open(config_path, 'w') as f:
            yaml.dump(test_config, f)
        
        # Use temporary config
        with patch.object(ConfigManager, '_find_config_file', return_value=config_path):
            ConfigManager._instance = None  # Reset singleton
            ConfigManager._initialized = False
            
            config = ConfigManager()
            assert config.get('ur15.network.robot_ip') == '192.168.1.99'
```
