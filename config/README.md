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
  network:
    robot_ip: "..."
    camera: {...}
  services: {...}
  paths: {...}

# Duco Robot Configuration  
duco:
  network:
    robot_ip: "..."
  services: {...}

# Shared/Common Configuration
shared:
  network:
    redis: {...}
    ffpp_server: {...}
```

## Usage in Code

### Python/ROS2 Nodes:
```python
from common.config_manager import ConfigManager

# Get configuration
config = ConfigManager()

# Robot-specific config
ur15 = config.get_robot('ur15')
ur15_ip = ur15.get('network.robot_ip')

# Shared config
redis_host = config.get('shared.network.redis.host')
```

### Launch Files:
```python
from common.config_manager import ConfigManager

config = ConfigManager()
ur15 = config.get_robot('ur15')

# Use in launch parameters
ur15_ip = ur15.get('network.robot_ip')
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

## Troubleshooting

### Config file not found
- Ensure `robot_config.yaml` exists in `robot_dc/config/`
- Check that `ROBOT_DC_ROOT` environment variable is set (optional)

### Import errors
- Rebuild the common package: `colcon build --packages-select common`
- Source the workspace: `source install/setup.bash`

### Validation errors
- Check YAML syntax with: `python3 -c "import yaml; yaml.safe_load(open('robot_config.yaml'))"`
- Ensure all required fields are present (compare with example)
