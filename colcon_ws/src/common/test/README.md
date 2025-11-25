# Common Package Tests

This directory contains unit tests for the `common` package utilities.

## Test Files

### test_workspace_utils.py

Tests for workspace path resolution functionality in `common/workspace_utils.py`.

**Coverage:**

1. **Workspace Root Detection**
   - `test_method0_robot_dc_root_env`: Tests ROBOT_DC_ROOT environment variable (highest priority)
   - `test_method1_ament_index`: Tests ROS package discovery method
   - `test_workspace_root_with_env_override`: Tests custom workspace override
   - `test_env_var_takes_precedence_over_detection`: Verifies environment variable priority
   - `test_invalid_robot_dc_root_ignored`: Tests fallback when invalid path is provided
   - `test_priority_order`: Tests priority of multiple environment variables
   - `test_returns_absolute_path`: Ensures returned paths are absolute
   - `test_returns_existing_directory`: Ensures returned paths exist

2. **Temp Directory Management**
   - `test_creates_temp_directory`: Tests automatic temp directory creation
   - `test_returns_existing_temp_directory`: Tests returning existing temp directory
   - `test_returns_absolute_path`: Ensures temp path is absolute
   - `test_temp_directory_is_writable`: Verifies write permissions
   - `test_raises_error_when_workspace_not_found`: Tests error handling

3. **Integration Tests**
   - `test_real_workspace_detection`: Tests detection in actual workspace
   - `test_temp_directory_in_real_workspace`: Tests temp directory in real workspace

## Running Tests

### Using pytest directly:
```bash
cd /home/robot/Documents/robot_dc/colcon_ws
python3 -m pytest src/common/test/test_workspace_utils.py -v
```

### Using colcon test:
```bash
cd /home/robot/Documents/robot_dc/colcon_ws
colcon test --packages-select common --event-handlers console_direct+
```

### View test results:
```bash
colcon test-result --all --verbose
```

## Test Philosophy

These tests verify that workspace path resolution works correctly across different execution contexts:

- **Source space**: Running from `colcon_ws/src/`
- **Install space**: Running from `colcon_ws/install/`
- **Custom locations**: Using environment variable overrides
- **Multiple environments**: Testing priority and fallback mechanisms

The tests use temporary directories to create isolated test environments while also including integration tests that verify behavior in the actual workspace.

## Environment Variables

The tests verify these environment variables in priority order:

1. `ROBOT_DC_ROOT`: Explicit project root override (highest priority)
2. `COLCON_PREFIX_PATH`: ROS2 colcon workspace environment
3. `ROS_WORKSPACE`: Legacy ROS workspace variable

## Adding New Tests

When adding new utility functions to `common/workspace_utils.py`:

1. Create test class: `class TestYourFunction(unittest.TestCase)`
2. Add unit tests: Test isolated functionality
3. Add integration tests: Test real-world usage
4. Update this README with coverage information

Example:
```python
class TestNewFunction(unittest.TestCase):
    def test_basic_functionality(self):
        """Test basic usage."""
        result = new_function()
        self.assertEqual(result, expected_value)
    
    def test_edge_case(self):
        """Test edge case handling."""
        with self.assertRaises(ValueError):
            new_function(invalid_input)
```

## Continuous Integration

These tests are designed to run in CI/CD pipelines. They:
- Use temporary directories (no side effects)
- Clean up resources in tearDown()
- Are deterministic and repeatable
- Run quickly (< 1 second total)

## Troubleshooting

**Tests fail with "workspace not found":**
- Ensure you're running from within the robot_dc workspace
- Check that colcon_ws/ and scripts/ directories exist
- Verify ROS2 environment is sourced

**Permission errors:**
- Tests may fail if temporary directory creation is blocked
- Check filesystem permissions for /tmp

**Import errors:**
- Ensure common package is built: `colcon build --packages-select common`
- Source the workspace: `source install/setup.bash`
