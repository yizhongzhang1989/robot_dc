# UR15 Workflow - ROS 2 Package

A ROS 2 package for modular, workflow-driven UR15 robot operations.

## Package Information

- **Package Name**: `ur15_workflow`
- **Type**: ROS 2 Python package (ament_cmake)
- **Version**: 1.0.0
- **Maintainer**: Robot DC Team

## Features

- Declarative workflow definition via YAML configuration
- Modular operation handlers (move, capture, positioning, etc.)
- Conditional execution and error handling
- Data flow between operations via shared context
- Easy to extend with custom operation types

## Package Structure

```
ur15_workflow/
├── CMakeLists.txt          # CMake build configuration
├── package.xml             # ROS package manifest
├── setup.py                # Python package setup
├── setup.cfg               # Python setup configuration
├── resource/               # Package resource marker
├── ur15_workflow/          # Python package
│   ├── __init__.py
│   ├── base.py            # Base classes
│   ├── engine.py          # Workflow engine
│   ├── runner.py          # Main runner
│   └── handlers/          # Operation handlers
│       ├── __init__.py
│       ├── robot_move.py
│       ├── capture.py
│       ├── movement_pattern.py
│       ├── positioning.py
│       └── coordinate_frame.py
├── scripts/               # Executable scripts
│   └── run_workflow.py
├── config/               # Workflow configurations
│   ├── workflow_example.yaml
│   └── workflow_simple.yaml
└── launch/              # Launch files (for future use)
```

## Building the Package

### Build

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select ur15_workflow
```

### Source the workspace

```bash
source ~/Documents/robot_dc/colcon_ws/install/setup.bash
```

## Usage

### Method 1: Using ros2 run

```bash
# Source workspace first
source ~/Documents/robot_dc/colcon_ws/install/setup.bash

# Run with package config files
ros2 run ur15_workflow run_workflow.py --config $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/examples/workflow_simple.yaml

# Run with custom config
ros2 run ur15_workflow run_workflow.py --config /path/to/my_workflow.yaml

# Dry run (validate only)
ros2 run ur15_workflow run_workflow.py --config workflow.yaml --dry-run
```

### Method 2: Using installed entry point

```bash
# After building and sourcing
run_workflow --config $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/examples/workflow_simple.yaml
```

### Method 3: Direct Python import

```python
import rclpy
from ur15_workflow.engine import WorkflowEngine
from ur15_workflow.handlers import register_all_handlers

# Your code here
engine = WorkflowEngine(config_path="workflow.yaml")
register_all_handlers(engine)
results = engine.execute()
```

## Configuration Files

The package includes example workflow configurations in the `config/` directory:

- **workflow_simple.yaml**: Simple single-position capture workflow
- **workflow_example.yaml**: Complete rack positioning workflow

Installed location: `$(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/examples/`

## Quick Start

1. **Build the package**:
   ```bash
   cd ~/Documents/robot_dc/colcon_ws
   colcon build --packages-select ur15_workflow
   source install/setup.bash
   ```

2. **Test with dry-run**:
   ```bash
   ros2 run ur15_workflow run_workflow.py \
     --config $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/examples/workflow_simple.yaml \
     --dry-run
   ```

3. **Run a workflow**:
   ```bash
   ros2 run ur15_workflow run_workflow.py \
     --config $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/examples/workflow_simple.yaml
   ```

## Creating Custom Workflows

1. Create a YAML configuration file based on the examples in `config/`
2. Define operations in the `workflow` list
3. Set global parameters in the `context` section
4. Run using `ros2 run ur15_workflow run_workflow.py --config your_workflow.yaml`

## Extending with Custom Handlers

1. Create a new handler in `ur15_workflow/handlers/`
2. Register it in `ur15_workflow/handlers/__init__.py`
3. Rebuild the package: `colcon build --packages-select ur15_workflow`
4. Use the new operation type in your workflow configuration

## Operation Types

- **robot_move**: Move robot to target pose
- **movement_pattern**: Execute capture patterns (grid_5point, grid_9point, etc.)
- **capture_image**: Capture images from camera
- **positioning**: Compute 3D positions from images
- **coordinate_frame**: Build coordinate frames from points

See workflow configuration examples for detailed parameter documentation.

## Dependencies

- ROS 2 (Humble or later)
- Python 3.8+
- rclpy
- PyYAML
- NumPy
- Standard ROS 2 message packages (std_msgs, sensor_msgs)

## Troubleshooting

### Package not found after build

Make sure to source the workspace:
```bash
source ~/Documents/robot_dc/colcon_ws/install/setup.bash
```

### Import errors

Rebuild the package and source again:
```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select ur15_workflow --symlink-install
source install/setup.bash
```

### Workflow validation fails

Use dry-run mode to check configuration:
```bash
ros2 run ur15_workflow run_workflow.py --config your_workflow.yaml --dry-run
```

## Integration with Other ROS Packages

This package is designed to work alongside other robot control packages. It can:
- Subscribe to camera image topics
- Integrate with positioning services
- Store results in robot_status service
- Work with existing UR15 robot control interfaces

## Future Enhancements

- ROS 2 node implementations for handlers
- Launch file templates
- ROS 2 service/action interfaces
- Parameter server integration
- RViz visualization plugins

## License

TODO

## Maintainer

Robot DC Team - robot@example.com

## Additional Documentation

For detailed architecture and usage documentation, see the original development documentation in the project root.
