#!/usr/bin/env python3
"""
Example: Comprehensive usage with diverse data types

This example demonstrates setting and getting status with various data types:
- Basic types: int, float, str, bool
- Collections: list, tuple, dict, set
- NumPy arrays (1D, 2D, 3D)
- Custom classes
- Nested structures with mixed types
"""

import rclpy
from rclpy.node import Node
from robot_status.client_utils import RobotStatusClient
from robot_status import set_to_status, get_from_status
import numpy as np
from dataclasses import dataclass
from typing import List


# Custom class example 1: Simple class
class RobotConfig:
    def __init__(self, max_speed, acceleration, brake_distance):
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.brake_distance = brake_distance
    
    def __repr__(self):
        return f"RobotConfig(max_speed={self.max_speed}, accel={self.acceleration}, brake={self.brake_distance})"


# Custom class example 2: Dataclass
@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    orientation: float
    velocity: float
    
    def __repr__(self):
        return f"Waypoint(x={self.x}, y={self.y}, z={self.z}, θ={self.orientation}°, v={self.velocity})"


# Custom class example 3: Class with tolist() method for JSON serialization
class Pose3D:
    """3D pose with position and orientation (quaternion)."""
    
    def __init__(self, x, y, z, qx, qy, qz, qw):
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
    
    def tolist(self):
        """Convert to list for JSON serialization."""
        return {
            'position': [self.x, self.y, self.z],
            'orientation': [self.qx, self.qy, self.qz, self.qw]
        }
    
    def __repr__(self):
        return f"Pose3D(pos=[{self.x}, {self.y}, {self.z}], quat=[{self.qx}, {self.qy}, {self.qz}, {self.qw}])"


def test_basic_types(client, node):
    """Test basic data types."""
    node.get_logger().info("\n=== Testing Basic Types ===")
    
    # Integer
    client.set_status('test_robot', 'battery_percent', 85)
    value = client.get_status('test_robot', 'battery_percent')
    node.get_logger().info(f"Integer: {value} (type: {type(value).__name__})")
    assert value == 85
    
    # Float
    client.set_status('test_robot', 'temperature', 42.5)
    value = client.get_status('test_robot', 'temperature')
    node.get_logger().info(f"Float: {value} (type: {type(value).__name__})")
    assert value == 42.5
    
    # String
    client.set_status('test_robot', 'status_message', "System operational")
    value = client.get_status('test_robot', 'status_message')
    node.get_logger().info(f"String: '{value}' (type: {type(value).__name__})")
    assert value == "System operational"
    
    # Boolean
    client.set_status('test_robot', 'is_connected', True)
    value = client.get_status('test_robot', 'is_connected')
    node.get_logger().info(f"Boolean: {value} (type: {type(value).__name__})")
    assert value is True
    
    # None
    client.set_status('test_robot', 'error_state', None)
    value = client.get_status('test_robot', 'error_state')
    node.get_logger().info(f"None: {value} (type: {type(value).__name__})")
    assert value is None
    
    node.get_logger().info("✓ All basic types passed")


def test_collections(client, node):
    """Test collection data types."""
    node.get_logger().info("\n=== Testing Collections ===")
    
    # List
    client.set_status('test_robot', 'active_sensors', ['lidar', 'camera', 'imu', 'gps'])
    value = client.get_status('test_robot', 'active_sensors')
    node.get_logger().info(f"List: {value} (type: {type(value).__name__})")
    assert value == ['lidar', 'camera', 'imu', 'gps']
    
    # Tuple
    client.set_status('test_robot', 'position_tuple', (1.5, 2.3, 0.5))
    value = client.get_status('test_robot', 'position_tuple')
    node.get_logger().info(f"Tuple: {value} (type: {type(value).__name__})")
    assert value == (1.5, 2.3, 0.5)
    
    # Dictionary
    client.set_status('test_robot', 'config_dict', {
        'ip': '192.168.1.100',
        'port': 8080,
        'timeout': 5.0,
        'retry': True
    })
    value = client.get_status('test_robot', 'config_dict')
    node.get_logger().info(f"Dict: {value} (type: {type(value).__name__})")
    assert value['ip'] == '192.168.1.100'
    
    # Set
    client.set_status('test_robot', 'enabled_features', {'navigation', 'mapping', 'collision_avoidance'})
    value = client.get_status('test_robot', 'enabled_features')
    node.get_logger().info(f"Set: {value} (type: {type(value).__name__})")
    assert 'navigation' in value
    
    # Nested list
    client.set_status('test_robot', 'path_points', [[0, 0], [1, 2], [3, 4], [5, 6]])
    value = client.get_status('test_robot', 'path_points')
    node.get_logger().info(f"Nested list: {value}")
    assert value[2] == [3, 4]
    
    node.get_logger().info("✓ All collection types passed")


def test_numpy_arrays(client, node):
    """Test NumPy arrays of various dimensions and types."""
    node.get_logger().info("\n=== Testing NumPy Arrays ===")
    
    # 1D array (vector)
    vec = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
    client.set_status('test_robot', 'velocity_vector', vec)
    value = client.get_status('test_robot', 'velocity_vector')
    node.get_logger().info(f"1D array: shape={value.shape}, dtype={value.dtype}")
    assert np.allclose(value, vec)
    
    # 2D array (matrix)
    matrix = np.array([[1, 2, 3],
                       [4, 5, 6],
                       [7, 8, 9]])
    client.set_status('test_robot', 'rotation_matrix', matrix)
    value = client.get_status('test_robot', 'rotation_matrix')
    node.get_logger().info(f"2D array: shape={value.shape}, dtype={value.dtype}")
    assert np.array_equal(value, matrix)
    
    # 3D array (tensor)
    tensor = np.random.rand(2, 3, 4)
    client.set_status('test_robot', 'sensor_data_tensor', tensor)
    value = client.get_status('test_robot', 'sensor_data_tensor')
    node.get_logger().info(f"3D array: shape={value.shape}, dtype={value.dtype}")
    assert np.allclose(value, tensor)
    
    # Different dtypes
    int_array = np.array([1, 2, 3, 4, 5], dtype=np.int32)
    client.set_status('test_robot', 'joint_indices', int_array)
    value = client.get_status('test_robot', 'joint_indices')
    node.get_logger().info(f"Int32 array: shape={value.shape}, dtype={value.dtype}")
    assert value.dtype == np.int32
    
    # Boolean array
    bool_array = np.array([True, False, True, False], dtype=bool)
    client.set_status('test_robot', 'sensor_status', bool_array)
    value = client.get_status('test_robot', 'sensor_status')
    node.get_logger().info(f"Boolean array: shape={value.shape}, dtype={value.dtype}")
    assert np.array_equal(value, bool_array)
    
    # Camera intrinsics example (common use case)
    camera_matrix = np.array([[1000.0, 0.0, 640.0],
                              [0.0, 1000.0, 480.0],
                              [0.0, 0.0, 1.0]])
    client.set_status('test_robot', 'camera_intrinsics', camera_matrix)
    value = client.get_status('test_robot', 'camera_intrinsics')
    node.get_logger().info(f"Camera matrix:\n{value}")
    assert np.allclose(value, camera_matrix)
    
    node.get_logger().info("✓ All NumPy array types passed")


def test_custom_classes(client, node):
    """Test custom class instances."""
    node.get_logger().info("\n=== Testing Custom Classes ===")
    
    # Simple class
    config = RobotConfig(max_speed=2.0, acceleration=0.5, brake_distance=1.2)
    client.set_status('test_robot', 'motion_config', config)
    value = client.get_status('test_robot', 'motion_config')
    node.get_logger().info(f"Custom class: {value}")
    assert value.max_speed == 2.0
    assert value.acceleration == 0.5
    
    # Dataclass
    waypoint = Waypoint(x=10.5, y=20.3, z=0.0, orientation=45.0, velocity=1.5)
    client.set_status('test_robot', 'target_waypoint', waypoint)
    value = client.get_status('test_robot', 'target_waypoint')
    node.get_logger().info(f"Dataclass: {value}")
    assert value.x == 10.5
    assert value.orientation == 45.0
    
    # Class with tolist() method
    pose = Pose3D(x=1.5, y=2.3, z=0.5, qx=0.0, qy=0.0, qz=0.707, qw=0.707)
    client.set_status('test_robot', 'robot_pose', pose)
    value = client.get_status('test_robot', 'robot_pose')
    node.get_logger().info(f"Class with tolist(): {value}")
    node.get_logger().info(f"  JSON representation: {value.tolist()}")
    assert value.x == 1.5
    assert value.z == 0.5
    
    # List of custom objects
    path = [
        Waypoint(0, 0, 0, 0, 1.0),
        Waypoint(5, 5, 0, 45, 1.5),
        Waypoint(10, 5, 0, 90, 1.0),
    ]
    client.set_status('test_robot', 'planned_path', path)
    value = client.get_status('test_robot', 'planned_path')
    node.get_logger().info(f"List of objects: {len(value)} waypoints")
    assert len(value) == 3
    assert value[1].x == 5
    
    node.get_logger().info("✓ All custom class types passed")


def test_complex_nested_structures(client, node):
    """Test complex nested structures with mixed types."""
    node.get_logger().info("\n=== Testing Complex Nested Structures ===")
    
    # Complex structure with everything
    complex_data = {
        'metadata': {
            'timestamp': 1234567890,
            'robot_id': 'robot_001',
            'mission_type': 'delivery',
            'priority': 5
        },
        'sensor_data': {
            'lidar': {
                'points': np.random.rand(100, 3),
                'intensity': np.random.rand(100),
                'timestamp': 1234567890.123
            },
            'camera': {
                'image_shape': (1920, 1080, 3),
                'intrinsics': np.eye(3),
                'distortion': np.array([0.1, -0.2, 0.0, 0.0, 0.0])
            }
        },
        'navigation': {
            'current_position': Waypoint(5.0, 3.2, 0.0, 30.0, 1.0),
            'path': [
                Waypoint(5, 3, 0, 30, 1.0),
                Waypoint(8, 6, 0, 45, 1.2),
                Waypoint(10, 10, 0, 90, 1.5)
            ],
            'obstacles': [(1.0, 2.0), (3.5, 4.5), (6.0, 7.0)]
        },
        'configuration': RobotConfig(2.5, 0.8, 1.5),
        'flags': {
            'is_autonomous': True,
            'collision_detection_enabled': True,
            'path_replanning_needed': False
        },
        'active_modules': ['navigation', 'perception', 'planning', 'control']
    }
    
    client.set_status('test_robot', 'complex_mission_data', complex_data)
    value = client.get_status('test_robot', 'complex_mission_data')
    
    # Verify nested structure
    node.get_logger().info(f"Complex data retrieved successfully")
    node.get_logger().info(f"  - Metadata robot_id: {value['metadata']['robot_id']}")
    node.get_logger().info(f"  - Lidar points shape: {value['sensor_data']['lidar']['points'].shape}")
    node.get_logger().info(f"  - Camera intrinsics:\n{value['sensor_data']['camera']['intrinsics']}")
    node.get_logger().info(f"  - Current position: {value['navigation']['current_position']}")
    node.get_logger().info(f"  - Path length: {len(value['navigation']['path'])} waypoints")
    node.get_logger().info(f"  - Configuration: {value['configuration']}")
    node.get_logger().info(f"  - Active modules: {value['active_modules']}")
    
    # Assertions
    assert value['metadata']['robot_id'] == 'robot_001'
    assert isinstance(value['sensor_data']['lidar']['points'], np.ndarray)
    assert value['sensor_data']['lidar']['points'].shape == (100, 3)
    assert isinstance(value['navigation']['current_position'], Waypoint)
    assert len(value['navigation']['path']) == 3
    assert isinstance(value['configuration'], RobotConfig)
    assert value['flags']['is_autonomous'] is True
    
    node.get_logger().info("✓ Complex nested structure passed")


def test_update_and_overwrite(client, node):
    """Test updating existing values."""
    node.get_logger().info("\n=== Testing Update and Overwrite ===")
    
    # Set initial value
    client.set_status('test_robot', 'counter', 0)
    value = client.get_status('test_robot', 'counter')
    node.get_logger().info(f"Initial counter: {value}")
    assert value == 0
    
    # Update value
    client.set_status('test_robot', 'counter', 42)
    value = client.get_status('test_robot', 'counter')
    node.get_logger().info(f"Updated counter: {value}")
    assert value == 42
    
    # Change type (int to array)
    client.set_status('test_robot', 'counter', np.array([1, 2, 3, 4, 5]))
    value = client.get_status('test_robot', 'counter')
    node.get_logger().info(f"Counter now array: {value}")
    assert isinstance(value, np.ndarray)
    
    node.get_logger().info("✓ Update and overwrite tests passed")


def test_direct_functions(node):
    """Test direct set_to_status and get_from_status functions."""
    node.get_logger().info("\n=== Testing Direct Functions (set_to_status/get_from_status) ===")
    
    # Test with basic types
    set_to_status(node, 'direct_test', 'message', "Hello from direct function")
    value = get_from_status(node, 'direct_test', 'message')
    node.get_logger().info(f"String via direct functions: '{value}'")
    assert value == "Hello from direct function"
    
    # Test with numpy array
    test_array = np.array([[1, 2, 3], [4, 5, 6]])
    set_to_status(node, 'direct_test', 'matrix', test_array)
    value = get_from_status(node, 'direct_test', 'matrix')
    node.get_logger().info(f"NumPy array via direct functions: shape={value.shape}")
    assert np.array_equal(value, test_array)
    
    # Test with custom class
    config = RobotConfig(max_speed=3.0, acceleration=1.0, brake_distance=2.0)
    set_to_status(node, 'direct_test', 'robot_config', config)
    value = get_from_status(node, 'direct_test', 'robot_config')
    node.get_logger().info(f"Custom class via direct functions: {value}")
    assert value.max_speed == 3.0
    
    # Test with dataclass
    waypoint = Waypoint(x=15.0, y=25.0, z=1.0, orientation=90.0, velocity=2.0)
    set_to_status(node, 'direct_test', 'waypoint', waypoint)
    value = get_from_status(node, 'direct_test', 'waypoint')
    node.get_logger().info(f"Dataclass via direct functions: {value}")
    assert value.x == 15.0
    assert value.orientation == 90.0
    
    # Test with Pose3D (class with tolist())
    pose = Pose3D(x=5.0, y=10.0, z=2.0, qx=0.0, qy=0.0, qz=0.383, qw=0.924)
    set_to_status(node, 'direct_test', 'pose', pose)
    value = get_from_status(node, 'direct_test', 'pose')
    node.get_logger().info(f"Pose3D via direct functions: {value}")
    node.get_logger().info(f"  JSON representation: {value.tolist()}")
    assert value.x == 5.0
    assert value.qw == 0.924
    
    # Test with complex dict
    complex_data = {
        'id': 'test_001',
        'values': [1, 2, 3, 4, 5],
        'matrix': np.eye(3),
        'config': RobotConfig(1.5, 0.3, 0.8),
        'nested': {
            'flag': True,
            'count': 42
        }
    }
    set_to_status(node, 'direct_test', 'complex', complex_data)
    value = get_from_status(node, 'direct_test', 'complex')
    node.get_logger().info(f"Complex dict via direct functions:")
    node.get_logger().info(f"  - id: {value['id']}")
    node.get_logger().info(f"  - values: {value['values']}")
    node.get_logger().info(f"  - matrix shape: {value['matrix'].shape}")
    node.get_logger().info(f"  - config: {value['config']}")
    node.get_logger().info(f"  - nested flag: {value['nested']['flag']}")
    assert value['id'] == 'test_001'
    assert isinstance(value['matrix'], np.ndarray)
    assert isinstance(value['config'], RobotConfig)
    
    node.get_logger().info("✓ All direct function tests passed")


def main():
    rclpy.init()
    node = Node('usage_example')
    
    try:
        # Create client
        client = RobotStatusClient(node)
        
        node.get_logger().info("="*60)
        node.get_logger().info("Starting comprehensive usage tests")
        node.get_logger().info("="*60)
        
        # Run all tests
        test_basic_types(client, node)
        test_collections(client, node)
        test_numpy_arrays(client, node)
        test_custom_classes(client, node)
        test_complex_nested_structures(client, node)
        test_update_and_overwrite(client, node)
        test_direct_functions(node)
        
        # Summary
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("✓ ALL TESTS PASSED!")
        node.get_logger().info("="*60)
        node.get_logger().info("\nKey capabilities demonstrated:")
        node.get_logger().info("  • Basic types: int, float, str, bool, None")
        node.get_logger().info("  • Collections: list, tuple, dict, set")
        node.get_logger().info("  • NumPy arrays: 1D, 2D, 3D, various dtypes")
        node.get_logger().info("  • Custom classes and dataclasses")
        node.get_logger().info("  • Complex nested structures")
        node.get_logger().info("  • Type changes on updates")
        node.get_logger().info("  • Direct functions (set_to_status/get_from_status)")
        node.get_logger().info("\nView all data at: http://localhost:8005")
        
    except Exception as e:
        node.get_logger().error(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
