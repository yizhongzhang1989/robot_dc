# UR15 Workflow ROS 2 Package - Quick Reference

## Package Location
```
colcon_ws/src/ur15_workflow/
```

## Build & Source

```bash
# Build the package
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select ur15_workflow

# Source the workspace
source ~/Documents/robot_dc/colcon_ws/install/setup.bash
```

## Usage

### Run workflow with package configs

```bash
# Simple workflow
ros2 run ur15_workflow run_workflow.py \
  --config $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config/workflow_simple.yaml

# Example workflow (full rack positioning)
ros2 run ur15_workflow run_workflow.py \
  --config $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config/workflow_example.yaml

# Dry run (validate only)
ros2 run ur15_workflow run_workflow.py \
  --config $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config/workflow_simple.yaml \
  --dry-run
```

### Run workflow with custom config

```bash
ros2 run ur15_workflow run_workflow.py --config /path/to/my_workflow.yaml
```

### Shorter command using environment variable

```bash
# Set once
export UR15_WF_CONFIG=$(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config

# Then use
ros2 run ur15_workflow run_workflow.py --config $UR15_WF_CONFIG/workflow_simple.yaml
```

## Package Structure

```
colcon_ws/src/ur15_workflow/
├── CMakeLists.txt              # CMake build configuration
├── package.xml                 # ROS package manifest
├── setup.py                    # Python package setup
├── setup.cfg                   # Python setup config
├── README.md                   # Package documentation
├── resource/                   # Package resource marker
│   └── ur15_workflow
├── ur15_workflow/              # Python package
│   ├── __init__.py
│   ├── base.py                # Base classes
│   ├── engine.py              # Workflow engine
│   ├── runner.py              # Main runner
│   └── handlers/              # Operation handlers
│       ├── __init__.py
│       ├── robot_move.py
│       ├── capture.py
│       ├── movement_pattern.py
│       ├── positioning.py
│       └── coordinate_frame.py
├── scripts/                    # Executable scripts
│   └── run_workflow.py
├── config/                     # Workflow configurations
│   ├── workflow_example.yaml
│   └── workflow_simple.yaml
└── launch/                     # Launch files (for future)
```

## Installed Files

After building, files are installed to:

```
install/ur15_workflow/
├── lib/ur15_workflow/
│   └── run_workflow.py        # Executable script
└── share/ur15_workflow/
    ├── config/                # Configuration files
    │   ├── workflow_example.yaml
    │   └── workflow_simple.yaml
    ├── launch/                # Launch files
    └── package.xml
```

## Configuration Files Location

```bash
# Get config directory
$(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config/

# List configs
ls $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config/
```

## Creating Custom Workflows

1. Copy an example config:
```bash
cp $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config/workflow_simple.yaml my_workflow.yaml
```

2. Edit the workflow operations

3. Run it:
```bash
ros2 run ur15_workflow run_workflow.py --config my_workflow.yaml
```

## Adding Custom Operation Handlers

1. Create new handler in `ur15_workflow/handlers/my_handler.py`
2. Import and register in `ur15_workflow/handlers/__init__.py`
3. Rebuild:
```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select ur15_workflow
source install/setup.bash
```

## Troubleshooting

### Package not found
```bash
# Make sure to source after building
source ~/Documents/robot_dc/colcon_ws/install/setup.bash
```

### Config file not found
```bash
# Verify config files are installed
ls $(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config/
```

### Python import errors
```bash
# Rebuild with symlink-install for development
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select ur15_workflow --symlink-install
source install/setup.bash
```

### Check package info
```bash
ros2 pkg prefix ur15_workflow
ros2 pkg executables ur15_workflow
```

## Tips

### Development workflow

Use `--symlink-install` to avoid rebuilding after Python changes:

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select ur15_workflow --symlink-install
```

Then you only need to source again after changes to Python files.

### Add to bashrc

Add to `~/.bashrc` for convenience:

```bash
# UR15 Workflow
source ~/Documents/robot_dc/colcon_ws/install/setup.bash
export UR15_WF_CONFIG=$(ros2 pkg prefix ur15_workflow)/share/ur15_workflow/config
```

Then use:
```bash
ros2 run ur15_workflow run_workflow.py --config $UR15_WF_CONFIG/workflow_simple.yaml
```

## Integration with Other Packages

This package can be used alongside other ROS packages in your workspace:

```bash
# Build all packages
cd ~/Documents/robot_dc/colcon_ws
colcon build

# Or build specific packages
colcon build --packages-select ur15_workflow other_package
```

## Next Steps

1. Test with actual robot hardware
2. Create custom workflows for your tasks
3. Add custom operation handlers as needed
4. Create launch files for automated startup
5. Integrate with other ROS nodes in your system
