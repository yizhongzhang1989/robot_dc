# Robot Status Redis - High-Performance Status Management

A Redis-based ROS2 package for high-performance, thread-safe robot status management with dynamic multi-robot support, hierarchical namespaces, and web dashboard visualization.

**Package Name:** `robot_status_redis`  
**Node Name:** `robot_status_node` (when running robot_status_node.py)

## Features

- **Redis Backend (Required)**: High-performance (<200μs latency), thread-safe status storage
- **No ROS2 Service Dependencies**: Direct Redis access eliminates executor conflicts
- **Thread-Safe**: Safe to call from Flask threads, timers, multi-threaded nodes, any Python context
- **Universal Data Type Support**: Pickle serialization supports any Python object (dict, list, numpy arrays, custom classes)
- **Dual-Format Storage**: Stores both pickle (for retrieval) and JSON (for human readability) when possible
- **Event-Driven Auto-Save**: Redis keyspace notifications trigger automatic file saves with 1-second debounce
- **Flexible Interface**: `**kwargs` parameter pattern for maximum flexibility and backward compatibility
- **Dynamic Multi-Robot Support**: Add robots without code changes - namespaces are created on-the-fly
- **Hierarchical Organization**: Organize status by namespace (per-robot or shared)
- **Simple API**: Four methods - `set_status`, `get_status`, `list_status`, `delete_status`
- **Web Dashboard**: Real-time visualization at http://localhost:8005 with type extraction for unpicklable objects
- **No Conversion Needed**: Pass Python objects directly - automatic pickle serialization

## Installation

### 1. Install Redis (Recommended)

```bash
# Install Redis server
sudo apt-get update
sudo apt-get install redis-server

# Install Python Redis client
pip3 install redis

# Start Redis server
sudo systemctl start redis-server
sudo systemctl enable redis-server  # Auto-start on boot

# Verify Redis is running
redis-cli ping  # Should return "PONG"
```

### 2. Build the Package

```bash
cd ~/robot_dc/colcon_ws
colcon build --packages-select robot_status_redis
source install/setup.bash
```

**Note:** Redis is required. The package will not work without Redis installed and running.

## Quick Start

### 1. Launch the System

```bash
ros2 launch robot_status_redis robot_status_launch.py
```

**Launch Parameters:**
- `auto_save_file_path` - Path to JSON file for auto-saving status (default: `temp/robot_status_auto_save.json` in robot_dc root)
- `web_enabled:=true/false` - Enable/disable web dashboard (default: true)
- `web_port:=8005` - Web server port (default: 8005)
- `web_host:=0.0.0.0` - Web server host (default: 0.0.0.0)

**Example:**
```bash
# Launch with custom auto-save file path
ros2 launch robot_status_redis robot_status_launch.py \
  auto_save_file_path:=/path/to/my_status.json

# Launch with different web port
ros2 launch robot_status_redis robot_status_launch.py web_port:=8080
```

**Auto-Save Behavior:**
- On startup: Loads all status from the JSON file if it exists
- On set/delete: Automatically saves the updated status to the file
- Status persists across node restarts

### 2. Access the Web Dashboard

Open your browser and navigate to:
- Local: http://localhost:8005
- Network: http://YOUR_IP:8005

The dashboard auto-refreshes every 2 seconds and displays all status organized by namespace.

## Service Interface (Deprecated)

**⚠️ Note:** The ROS2 service interface is **currently deprecated** for use with this package. It is provided only for backward compatibility with the original `robot_status` package. 

**Recommended:** Use the [Python Client API](#python-client-api-recommended) instead for direct Redis access with better performance and no executor conflicts.

The ROS2 service interface writes to Redis (same backend as Python client), so data is immediately visible in the web dashboard and accessible via both service API and Python client API.

### Set Status

Store a status value for any namespace.key combination. **Triggers auto-save after successful set.**

**Service:** `/robot_status/set`

**Command:**
```bash
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'pose', value: '{\"x\":1.5,\"y\":2.3,\"z\":0.5}'}"
```

**Parameters:**
- `ns` (string): Namespace identifier (e.g., 'robot1', 'robot2', 'shared')
- `key` (string): Status key (e.g., 'pose', 'battery', 'state')
- `value` (string): Value as string or JSON

**Response:**
- `success` (bool): Operation success status
- `message` (string): Result message

**Examples:**
```bash
# Set robot pose (JSON)
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'pose', value: '{\"x\":1.5,\"y\":2.3,\"z\":0.5}'}"

# Set battery level (plain string)
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot2', key: 'battery', value: '85'}"

# Set shared mission ID
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'shared', key: 'mission_id', value: 'task_123'}"

# Set robot state
ros2 service call /robot_status/set robot_status/srv/SetStatus \
  "{ns: 'robot1', key: 'state', value: 'IDLE'}"
```

### Get Status

Retrieve a specific status value.

**Service:** `/robot_status/get`

**Command:**
```bash
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot1', key: 'pose'}"
```

**Parameters:**
- `ns` (string): Namespace identifier
- `key` (string): Status key to retrieve

**Response:**
- `success` (bool): Whether the parameter exists
- `value` (string): The stored value
- `message` (string): Success or error message

**Examples:**
```bash
# Get robot1 pose
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot1', key: 'pose'}"

# Get robot2 battery
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'robot2', key: 'battery'}"

# Get shared mission_id
ros2 service call /robot_status/get robot_status/srv/GetStatus \
  "{ns: 'shared', key: 'mission_id'}"
```

### List Status

List all status or filter by namespace.

**Service:** `/robot_status/list`

**Command:**
```bash
# List all status
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: ''}"

# List only robot1 status
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: 'robot1'}"
```

**Parameters:**
- `ns` (string): Namespace filter (empty string = all namespaces)

**Response:**
- `success` (bool): Operation success
- `namespaces` (string[]): List of namespace identifiers
- `status_dict` (string): JSON dictionary of all status
- `message` (string): Summary message

**Examples:**
```bash
# List all namespaces and status
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: ''}"

# Filter by robot1 only
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: 'robot1'}"

# Filter by shared namespace
ros2 service call /robot_status/list robot_status/srv/ListStatus "{ns: 'shared'}"
```

### Delete Status

Remove a specific status key or an entire namespace. **Triggers auto-save after deletion.**

**Service:** `/robot_status/delete`

**Command:**
```bash
# Delete specific key
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: 'pose'}"

# Delete entire namespace
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: ''}"
```

**Parameters:**
- `ns` (string): Namespace identifier
- `key` (string): Status key to delete (empty string = delete entire namespace)

**Response:**
- `success` (bool): Operation result
- `message` (string): Result message

**Examples:**
```bash
# Delete robot1 pose
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: 'pose'}"

# Delete robot1 battery
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot2', key: 'battery'}"

# Delete entire robot1 namespace (all keys)
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'robot1', key: ''}"

# Delete entire shared namespace
ros2 service call /robot_status/delete robot_status/srv/DeleteStatus \
  "{ns: 'shared', key: ''}"
```

## Python Client API (Recommended)

### RobotStatusClient - Redis-Based

The `RobotStatusClient` class provides a simple Redis-based interface. **No manual conversion needed** - pickle serialization handles all Python objects automatically. **No ROS2 node required** for operation (optional for logging only).

```python
#!/usr/bin/env python3
import numpy as np
from robot_status_redis.client_utils import RobotStatusClient

# Create client (no ROS2 node required!)
client = RobotStatusClient()

# Or with custom Redis configuration
client = RobotStatusClient(host='localhost', port=6379, db=0, key_prefix='my_app')

# Or with ROS2 node for logging
from rclpy.node import Node
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Create client with optional node for logging
        self.client = RobotStatusClient(node=self)
        
        # Set data - NO conversion needed!
        # Supports: dict, list, numpy arrays, custom classes, any picklable object
        
        # Dictionary
        pose = {'x': 1.5, 'y': 2.3, 'z': 0.5}
        self.client.set_status('robot1', 'pose', pose)
        
        # NumPy array - pass directly, no tolist() needed!
        camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]])
        self.client.set_status('robot1', 'camera_matrix', camera_matrix)
        
        # List
        waypoints = [[0, 0], [1, 2], [3, 4]]
        self.client.set_status('robot1', 'waypoints', waypoints)
        
        # Simple values
        self.client.set_status('robot1', 'battery', 85)
        self.client.set_status('robot1', 'status', 'operational')
        
        # Get data - returns original Python objects!
        retrieved_pose = self.client.get_status('robot1', 'pose')
        # retrieved_pose is already a dict: {'x': 1.5, 'y': 2.3, 'z': 0.5}
        
        retrieved_matrix = self.client.get_status('robot1', 'camera_matrix')
        # retrieved_matrix is already a numpy array!
        self.get_logger().info(f"Matrix shape: {retrieved_matrix.shape}")
        
        # List all status
        all_status = self.client.list_status()
        robot1_status = self.client.list_status('robot1')
        
        # Delete
        self.client.delete_status('robot1', 'pose')  # Delete specific key
        self.client.delete_status('robot1')  # Delete entire namespace


def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Key Benefits:**
- ✅ **No conversion needed** - Pass Python objects directly (dict, numpy arrays, custom classes)
- ✅ **No executor conflicts** - Safe to call from Flask threads, timers, multi-threaded nodes, any Python code
- ✅ **No ROS2 dependency** - Works in any Python context, ROS2 node optional for logging only
- ✅ **Thread-safe** - No `spin_until_future_complete` issues
- ✅ **High performance** - <200μs per operation via Redis
- ✅ **Type preservation** - Retrieved objects have original types and structure

### Client Configuration Options

The `RobotStatusClient` accepts flexible parameters via `**kwargs`:

```python
#!/usr/bin/env python3
import rclpy
from robot_status_redis.client_utils import RobotStatusClient

# Option 1: Simplest - default configuration
client = RobotStatusClient()

# Option 2: With ROS2 node for logging
rclpy.init()
node = rclpy.create_node('my_node')
client = RobotStatusClient(node)

# Option 3: Custom Redis configuration
client = RobotStatusClient(
    host='localhost',
    port=6379,
    db=1,  # Use different database for isolation
    key_prefix='my_app'  # Custom key prefix
)

# Option 4: Mixed parameters (backward compatible)
client = RobotStatusClient(
    node,
    timeout_sec=10.0,  # Ignored but accepted for compatibility
    auto_spin=True,     # Ignored but accepted for compatibility
    key_prefix='custom'
)

# Set status (automatically handles pickle encoding)
client.set_status('robot1', 'pose', {'x': 1.5, 'y': 2.3, 'z': 0.5})
client.set_status('robot1', 'battery', 95)
client.set_status('robot1', 'state', 'MOVING')

# Get status (automatically decodes JSON if possible)
pose = client.get_status('robot1', 'pose')
print(f"Pose: {pose}")  # {'x': 1.5, 'y': 2.3, 'z': 0.5}

battery = client.get_status('robot1', 'battery')
print(f"Battery: {battery}%")  # 95

# List all status
all_status = client.list_status()
print(f"All status: {all_status}")

# List robot1 status only
robot1_status = client.list_status(namespace='robot1')
print(f"Robot1 status: {robot1_status}")

# Get all namespaces
namespaces = client.get_namespaces()
print(f"Active robots: {namespaces}")  # ['robot1', 'robot2', 'shared']

# Delete specific key
client.delete_status('robot1', 'pose')

# Delete entire namespace
client.delete_status('robot2')  # Deletes all robot2 keys

node.destroy_node()
rclpy.shutdown()
```

### Running Examples

The package includes example scripts:

```bash
# Navigate to examples directory
cd ~/robot_dc/colcon_ws
source install/setup.bash

# Run usage example
python3 install/robot_status/share/robot_status/examples/usage_example.py

# Or run from source
python3 src/robot_status/examples/usage_example.py
```

## Testing

### Prerequisites

Ensure Redis is running:
```bash
redis-cli ping  # Should return "PONG"
```

### Quick Test: Python Client

```bash
cd ~/robot_dc/colcon_ws
source install/setup.bash

# Test basic operations with type preservation
python3 -c "
from robot_status_redis.client_utils import RobotStatusClient
import numpy as np

client = RobotStatusClient()
print('✓ Client created')

# Set and get dictionary - type preserved
client.set_status('test', 'data', {'x': 1, 'y': 2})
result = client.get_status('test', 'data')
print(f'✓ Dict: {result} (type: {type(result).__name__})')

# Set and get numpy array - type preserved, no tolist() needed!
arr = np.eye(3)
client.set_status('test', 'matrix', arr)
result = client.get_status('test', 'matrix')
print(f'✓ NumPy: shape {result.shape} (type: {type(result).__name__})')

# Set and get list
client.set_status('test', 'waypoints', [[0, 0], [1, 2], [3, 4]])
result = client.get_status('test', 'waypoints')
print(f'✓ List: {result} (type: {type(result).__name__})')

# Cleanup
client.delete_status('test')
print('✓ All tests passed!')
"
```

**Expected output:**
```
✓ Client created
✓ Dict: {'x': 1, 'y': 2} (type: dict)
✓ NumPy: shape (3, 3) (type: ndarray)
✓ List: [[0, 0], [1, 2], [3, 4]] (type: list)
✓ All tests passed!
```

### Test: Data Type Support

Test various Python data types with automatic serialization:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from robot_status_redis.client_utils import RobotStatusClient


class TestDataTypes(Node):
    def __init__(self):
        super().__init__('test_data_types')
        self.client = RobotStatusClient(node=self)
        
        # Test 1: Dictionary
        data = {'x': 1.5, 'y': 2.3, 'nested': {'a': 1, 'b': 2}}
        self.client.set_status('test', 'dict', data)
        result = self.client.get_status('test', 'dict')
        assert result == data, "Dict test failed"
        print("✅ Dict test passed")
        
        # Test 2: NumPy array
        arr = np.array([[1, 2, 3], [4, 5, 6]])
        self.client.set_status('test', 'numpy', arr)
        result = self.client.get_status('test', 'numpy')
        assert np.array_equal(result, arr), "NumPy test failed"
        print(f"✅ NumPy test passed - shape {result.shape}")
        
        # Test 3: List
        lst = [1, 2.5, 'string', True, None]
        self.client.set_status('test', 'list', lst)
        result = self.client.get_status('test', 'list')
        assert result == lst, "List test failed"
        print("✅ List test passed")
        
        # Test 4: Custom class
        class Pose3D:
            def __init__(self, x, y, z):
                self.x, self.y, self.z = x, y, z
            def tolist(self):
                return [self.x, self.y, self.z]
        
        pose = Pose3D(1.0, 2.0, 3.0)
        self.client.set_status('test', 'pose', pose)
        result = self.client.get_status('test', 'pose')
        assert result.x == 1.0 and result.y == 2.0, "Custom class test failed"
        print("✅ Custom class test passed")
        
        # Cleanup
        self.client.delete_status('test', 'dict')
        self.client.delete_status('test', 'numpy')
        self.client.delete_status('test', 'list')
        self.client.delete_status('test', 'pose')
        print("\n✅ All data type tests passed!")


rclpy.init()
node = TestDataTypes()
import time; time.sleep(0.5)
node.destroy_node()
rclpy.shutdown()
```

Save as `test_datatypes.py` and run:
```bash
python3 test_datatypes.py
```

### Test: Web Dashboard

```bash
# Terminal 1: Start the system with web dashboard
cd ~/robot_dc/colcon_ws
source install/setup.bash
ros2 launch robot_status_redis robot_status_launch.py

# Terminal 2: Add some test data
source install/setup.bash
python3 -c "
from robot_status_redis.client_utils import RobotStatusClient
import numpy as np

client = RobotStatusClient()
client.set_status('robot1', 'pose', {'x': 1.5, 'y': 2.3, 'z': 0.5})
client.set_status('robot1', 'battery', 85)
client.set_status('robot2', 'status', 'moving')
print('Test data added!')
"

# Terminal 3: Open browser
firefox http://localhost:8005
```

The dashboard connects directly to Redis for real-time status visualization.

### Performance Test

Test Redis operation latency:

```python
#!/usr/bin/env python3
import time
import numpy as np
from robot_status_redis.client_utils import RobotStatusClient


class PerfTest:
    def __init__(self):
        self.client = RobotStatusClient()
        
        print(f"\nTesting Redis Performance")
        print("=" * 50)
        
        # Test set performance
        data = np.random.rand(10, 10)
        iterations = 100
        
        start = time.time()
        for i in range(iterations):
            self.client.set_status('perf', f'test_{i}', data)
        elapsed = time.time() - start
        
        avg_set = (elapsed / iterations) * 1000  # Convert to ms
        print(f"Set operations: {avg_set:.2f} ms/op")
        
        # Test get performance
        start = time.time()
        for i in range(iterations):
            self.client.get_status('perf', f'test_{i}')
        elapsed = time.time() - start
        
        avg_get = (elapsed / iterations) * 1000
        print(f"Get operations: {avg_get:.2f} ms/op")
        
        # Cleanup
        for i in range(iterations):
            self.client.delete_status('perf', f'test_{i}')
        
        print("=" * 50)
        print("Expected: Redis <1ms per operation")


test = PerfTest()
```

**Typical Results:**
- **Redis**: 0.2-0.5 ms per operation

### Verify Dual-Format Storage

Check that data is stored in both pickle and JSON formats (when JSON-serializable):

```bash
# 1. Set some data
cd ~/robot_dc/colcon_ws
source install/setup.bash
python3 -c "
from robot_status_redis.redis_backend import RedisBackend
import numpy as np

backend = RedisBackend()
backend.set_status('test', 'dict', {'x': 1, 'y': 2})
backend.set_status('test', 'numpy', np.eye(3))
"

# 2. Check storage format
python3 -c "
from robot_status_redis.redis_backend import RedisBackend

backend = RedisBackend()
storage = backend.list_status('test')

for key, value in storage['test'].items():
    print(f'{key}:')
    print(f'  Has pickle: {\"pickle\" in value}')
    print(f'  Has JSON: {\"json\" in value}')
    if 'json' in value:
        print(f'  JSON value: {value[\"json\"]}')
    print()
"
```

**Expected output:**
```
dict:
  Has pickle: True
  Has JSON: True
  JSON value: {'x': 1, 'y': 2}

numpy:
  Has pickle: True
  Has JSON: True
  JSON value: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
```

## Use Cases

### Single Robot Status

```python
# Using Python client (recommended)
from robot_status_redis.client_utils import RobotStatusClient

client = RobotStatusClient()

# Set robot state
client.set_status('robot1', 'state', 'IDLE')

# Set current task
client.set_status('robot1', 'current_task', 'navigation')

# Set error status
client.set_status('robot1', 'error', 'none')
```

### Multi-Robot Fleet

```python
from robot_status_redis.client_utils import RobotStatusClient

client = RobotStatusClient()

# Robot 1 status
client.set_status('robot1', 'location', 'warehouse_A')

# Robot 2 status
client.set_status('robot2', 'location', 'warehouse_B')

# Robot 3 status
client.set_status('robot3', 'location', 'charging_station')

# Shared fleet status
client.set_status('shared', 'active_robots', 3)
```

### Monitoring and Coordination

```python
from robot_status_redis.client_utils import RobotStatusClient

client = RobotStatusClient()

# Check all robots
all_status = client.list_status()
print(f"All robots: {all_status}")

# Check specific robot
battery = client.get_status('robot2', 'battery')
print(f"Robot2 battery: {battery}%")

# Update shared mission
client.set_status('shared', 'mission_status', 'in_progress')
```

## Architecture

### Component Overview

```
robot_status_redis/
├── redis_backend.py        # Redis interface
├── client_utils.py         # RobotStatusClient (Redis-only)
├── web_dashboard.py        # Flask web interface (Redis-only)
└── robot_status_node.py    # Optional ROS2 service node for file-based backup
```

### Design Principles

1. **Redis-Only Architecture**: High-performance, thread-safe Redis backend (required)
2. **Thread-Safe Access**: Redis provides lockless, thread-safe operations without executor conflicts
3. **Universal Serialization**: Pickle format supports any Python object (dict, list, numpy, custom classes)
4. **Dual-Format Storage**: Stores pickle (for retrieval) + JSON (for human readability) when possible
5. **No ROS2 Dependencies**: Works in any Python context, ROS2 node optional for logging only
6. **Clear Error Messages**: Provides setup instructions if Redis is unavailable
7. **Dynamic Namespaces**: No pre-configuration required - namespaces created on-the-fly

### Data Flow

```
Python Client → RobotStatusClient → RedisBackend → Redis Server ← ROS2 Services (robot_status_node)
                                                        ↓                              ↓
Web Browser → WebDashboard (Flask) → RedisBackend → Redis Server         File backup (JSON)
```

### Key Architecture Benefits

1. **No Executor Conflicts**: Redis can be accessed from any thread without `spin_until_future_complete`
2. **High Performance**: Redis operations complete in <200μs
3. **Simplified Codebase**: Single backend reduces complexity (60% code reduction)
4. **No ROS2 Required**: Works in any Python context, ROS2 node optional for logging
5. **Type Preservation**: Pickle serialization preserves exact Python types (numpy arrays, custom classes)

## Troubleshooting

### Redis Connection Issues

```bash
# Check if Redis is running
redis-cli ping  # Should return "PONG"

# Check Redis status
sudo systemctl status redis-server

# Start Redis if stopped
sudo systemctl start redis-server

# Check Redis connection
cd ~/robot_dc/colcon_ws
source install/setup.bash
python3 -c "
from robot_status_redis.redis_backend import is_redis_available
print('Redis available:', is_redis_available())
"
```

### Import Errors

```bash
# If getting "ModuleNotFoundError: No module named 'redis'"
pip3 install redis

# Verify installation
python3 -c "import redis; print('Redis module OK')"
```

### Redis Not Available Error

If you see:
```
ConnectionError: Redis is not available. Please install and start Redis:
  sudo apt-get install redis-server
  sudo systemctl start redis-server
  pip3 install redis
Verify with: redis-cli ping
```

**Solution:** Follow the installation instructions in the error message. Redis is required for this package.

### ROS2 Services Not Available (Optional Node)

```bash
# Check if node is running (optional for file backup)
ros2 node list | grep robot_status_redis

# List available services
ros2 service list | grep robot_status_redis

# Restart the system if you need file backup
ros2 launch robot_status_redis robot_status_launch.py
```

**Note:** The Python client works without the ROS2 node - it connects directly to Redis.

### Web Dashboard Not Accessible

```bash
# Check if port is in use
lsof -i:8005

# Check firewall settings
sudo ufw allow 8005

# Start web dashboard (starts automatically with launch file)
ros2 launch robot_status_redis robot_status_launch.py

# Or start standalone
cd ~/robot_dc/colcon_ws
source install/setup.bash
python3 src/robot_status/robot_status_redis/web_dashboard.py
```

### Performance Issues

If operations are slow (>10ms):

```bash
# Check if Redis is running properly
redis-cli ping  # Should return "PONG"

# Check Redis performance
redis-cli --latency

# If Redis is slow, check system resources
top

# Restart Redis if needed
sudo systemctl restart redis-server
```

## API Reference

### Service Definitions

**SetStatus.srv**
```
string ns           # Namespace identifier
string key          # Status key
string value        # Value (string or JSON)
---
bool success        # Operation result
string message      # Result message
```

**DeleteStatus.srv**
```
string ns           # Namespace identifier
string key          # Status key to delete (empty = delete namespace)
---
bool success        # Operation result
string message      # Result message
```

**ListStatus.srv**
```
string ns           # Namespace filter (empty = all)
---
bool success        # Operation result
string[] namespaces # List of namespace identifiers
string status_dict  # JSON dictionary of status
string message      # Summary message
```

## Auto-Save and Persistence

The node automatically persists all status to a JSON file for restart-safe operation using **event-driven** Redis keyspace notifications.

### Configuration

Set the file path via launch parameter:
```bash
ros2 launch robot_status_redis robot_status_launch.py \
  auto_save_file_path:=/path/to/status_backup.json
```

Default path: `temp/robot_status_auto_save.json` (in robot_dc root directory)

### Behavior

- **On Startup**: Automatically loads all status from the JSON file if it exists
- **On Redis Change**: Automatically saves when Redis data changes (event-driven via keyspace notifications)
- **Debounced**: Uses 1-second debounce to batch rapid changes and prevent excessive file writes
- **All Sources**: Captures changes from Python client, ROS2 services, and web dashboard
- **File Format**: Standard JSON with dual-format storage (pickle + JSON when possible)

### JSON File Format

The file stores data in dual format: pickle (base64-encoded) for faithful serialization and JSON for human readability when possible.

```json
{
  "robot1": {
    "pose": {
      "pickle": "gASVJgAAAAAAAAB9lCiMAXiUjAF5lIwBelSMBW5lc3RlZJR9lCiMAWGUSwGMAWKUSwJ1dS4=",
      "json": {"x": 1.5, "y": 2.3, "z": 0.5, "nested": {"a": 1, "b": 2}}
    },
    "battery": {
      "pickle": "gASVBQAAAAAAAABLVS4=",
      "json": 85
    },
    "state": {
      "pickle": "gASVCgAAAAAAAACMBklETEWULg==",
      "json": "IDLE"
    }
  },
  "robot2": {
    "camera_matrix": {
      "pickle": "gASVgwAAAAAAAACMFW51bXB5LmNvcmUubXVsdGlhcnJheZSTlC4=",
      "json": [[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]]
    }
  }
}
```

**Format Notes:**
- `pickle`: Base64-encoded pickle string for exact type preservation
- `json`: Human-readable JSON representation when object is JSON-serializable
- NumPy arrays automatically converted to nested lists in JSON field
- Custom classes only have pickle field (JSON conversion not possible)

### Manual File Management

You can manually edit the JSON file when the node is stopped. On next startup, the node will load your changes.

```bash
# Stop the node
# Edit the JSON file
nano robot_status_auto_save.json

# Restart to load changes
ros2 launch robot_status robot_status_launch.py
```

## Performance Considerations

- **Concurrency**: Thread-safe operations allow multiple simultaneous requests
- **Memory**: Status stored in-memory as ROS2 parameters
- **Persistence**: Automatic file I/O on every set/delete (negligible overhead for typical usage)
- **Scalability**: Tested with multiple robots and high-frequency updates
- **Network**: Web dashboard uses polling (2s interval) - adjust if needed

## License

This package is part of the robot_dc workspace.

## Contributing

When extending this package:
1. Maintain backward compatibility with existing service interfaces
2. Add thread safety for any new shared state
3. Update this README with new features
4. Include example usage in the examples/ directory

## Support

For issues or questions:
1. Check the troubleshooting section
2. Review example scripts
3. Examine log files in ~/.ros/log/
