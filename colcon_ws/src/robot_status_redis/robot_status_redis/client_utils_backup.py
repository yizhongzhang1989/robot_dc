#!/usr/bin/env python3
"""
Robot Status Client Utilities - Redis Backend Only

Thread-safe, high-performance status storage using Redis.
No ROS2 service dependencies - works in any context (Flask threads, timers, etc.)

Values are serialized using pickle, which supports:
- Basic types (int, float, str, bool)
- Collections (dict, list, tuple, set)
- Custom classes
- NumPy arrays
- Any picklable Python object

Usage:
    from robot_status.client_utils import RobotStatusClient
    import numpy as np
    
    # Create client (no ROS2 node required)
    client = RobotStatusClient()
    
    # Set status - supports various data types, no conversion needed
    client.set_status('robot1', 'pose', {'x': 1.5, 'y': 2.3, 'z': 0.0})
    client.set_status('robot1', 'camera_matrix', np.eye(3))  # NumPy array
    client.set_status('robot1', 'config', MyCustomClass())  # Custom objects
    
    # Get status - returns original Python objects
    pose = client.get_status('robot1', 'pose')
    matrix = client.get_status('robot1', 'camera_matrix')  # Returns np.ndarray
    
    # List all status
    all_status = client.list_status()
    
    # List specific namespace
    robot1_status = client.list_status('robot1')
    
    # Delete status
    client.delete_status('robot1', 'pose')  # Delete specific key
    client.delete_status('robot1')  # Delete entire namespace
"""

import pickle
import base64
from typing import Optional, Any, Dict

try:
    from .redis_backend import get_redis_backend, is_redis_available
except ImportError:
    # Handle case where this is imported without relative import
    from robot_status.redis_backend import get_redis_backend, is_redis_available


class RobotStatusClient:
    """
    Redis-based robot status client.
    
    Thread-safe and high-performance (<200μs per operation).
    No ROS2 dependencies - works anywhere in Python code.
    """
    
    def __init__(self, node=None, host='localhost', port=6379, db=0, password=None):
        """
        Initialize Redis-based status client.
        
        Args:
            node: Optional ROS2 node for logging (not required for operation)
            host: Redis server host (default: localhost)
            port: Redis server port (default: 6379)
            db: Redis database number (default: 0)
            password: Redis password if required (default: None)
        
        Raises:
            ConnectionError: If Redis is not available
            ImportError: If redis module is not installed
        """
        self.node = node
        
        # Check if Redis is available
        if not is_redis_available(host, port, db, password):
            error_msg = (
                "Redis is not available. Please install and start Redis:\n"
                "  sudo apt-get install redis-server\n"
                "  sudo systemctl start redis-server\n"
                "  pip3 install redis\n"
                "Verify with: redis-cli ping"
            )
            if node:
                node.get_logger().error(error_msg)
            raise ConnectionError(error_msg)
        
        # Get Redis backend
        self._redis_backend = get_redis_backend(host, port, db, password)
        if self._redis_backend is None:
            raise ConnectionError("Failed to connect to Redis")
        
        if node:
            node.get_logger().info("✓ Connected to Redis backend for robot_status")
    
    def set_status(self, namespace: str, key: str, value: Any) -> bool:
        """
        Set status value.
        
        Args:
            namespace: Robot or namespace identifier (e.g., 'robot1', 'shared')
            key: Status key (e.g., 'pose', 'battery')
            value: Value to set (any picklable Python object: dict, list, numpy array, etc.)
            
        Returns:
            bool: True if successful, False otherwise
            
        Example:
            import numpy as np
            
            client.set_status('robot1', 'pose', {'x': 1.0, 'y': 2.0, 'z': 0.0})
            client.set_status('robot1', 'battery', 85)
            client.set_status('robot1', 'camera_matrix', np.eye(3))  # No conversion needed!
        """
        try:
            return self._redis_backend.set_status(namespace, key, value)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in set_status: {e}")
            return False
    
    def get_status(self, namespace: str, key: str) -> Any:
        """
        Get status value.
        
        Args:
            namespace: Robot or namespace identifier
            key: Status key to retrieve
            
        Returns:
            Value if found (original Python object), None if not found
            
        Example:
            pose = client.get_status('robot1', 'pose')  # Returns dict
            battery = client.get_status('robot1', 'battery')  # Returns int
            matrix = client.get_status('robot1', 'camera_matrix')  # Returns np.ndarray
        """
        try:
            success, value = self._redis_backend.get_status(namespace, key)
            return value if success else None
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in get_status: {e}")
            return None
    
    def list_status(self, namespace: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
        """
        List status for all namespaces or specific namespace.
        
        Args:
            namespace: Optional namespace filter (None = list all)
            
        Returns:
            dict: Status dictionary, e.g., {'robot1': {'pose': {...}, 'battery': 85}}
                  Returns empty dict if error or no status
            
        Example:
            all_status = client.list_status()  # All namespaces
            robot1_status = client.list_status('robot1')  # Just robot1
        """
        try:
            storage_tree = self._redis_backend.list_status(namespace)
            
            # Convert storage dicts to actual values
            result = {}
            for ns, keys in storage_tree.items():
                result[ns] = {}
                for k, storage_dict in keys.items():
                    try:
                        # Extract pickle string and deserialize
                        pickle_str = storage_dict.get("pickle", "")
                        if pickle_str:
                            pickled = base64.b64decode(pickle_str.encode('ascii'))
                            result[ns][k] = pickle.loads(pickled)
                    except Exception:
                        # Keep as-is if deserialization fails
                        result[ns][k] = storage_dict
            
            return result
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in list_status: {e}")
            return {}
    
    def delete_status(self, namespace: str, key: Optional[str] = None) -> bool:
        """
        Delete status value or entire namespace.
        
        Args:
            namespace: Robot or namespace identifier
            key: Status key to delete (None = delete entire namespace)
            
        Returns:
            bool: True if successful, False otherwise
            
        Example:
            client.delete_status('robot1', 'pose')  # Delete specific key
            client.delete_status('robot1')  # Delete entire namespace
        """
        try:
            if key:
                # Delete specific key
                return self._redis_backend.delete_status(namespace, key)
            else:
                # Delete entire namespace - get all keys and delete them
                storage_tree = self._redis_backend.list_status(namespace)
                if namespace in storage_tree:
                    success = True
                    for k in storage_tree[namespace].keys():
                        if not self._redis_backend.delete_status(namespace, k):
                            success = False
                    return success
                return False
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in delete_status: {e}")
            return False
    
    def get_namespaces(self) -> list:
        """
        Get list of all namespaces.
        
        Returns:
            list: List of namespace strings, e.g., ['robot1', 'robot2', 'shared']
            
        Example:
            namespaces = client.get_namespaces()
            for ns in namespaces:
                print(f"Found namespace: {ns}")
        """
        try:
            storage_tree = self._redis_backend.list_status()
            return list(storage_tree.keys())
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in get_namespaces: {e}")
            return []


# Standalone helper functions for simple get/set operations
def get_from_status(namespace: str, key: str, host='localhost', port=6379) -> Any:
    """
    Standalone helper function to get a value from robot_status.
    
    Args:
        namespace: The namespace to query
        key: The key to retrieve
        host: Redis server host
        port: Redis server port
        
    Returns:
        The original Python object if found, None otherwise
        
    Example:
        from robot_status.client_utils import get_from_status
        
        matrix = get_from_status('ur15', 'camera_matrix')
        if matrix is not None:
            print(matrix.shape)
    """
    try:
        client = RobotStatusClient(host=host, port=port)
        return client.get_status(namespace, key)
    except Exception:
        return None


def set_to_status(namespace: str, key: str, value: Any, host='localhost', port=6379) -> bool:
    """
    Standalone helper function to set a value in robot_status.
    
    Args:
        namespace: The namespace to use
        key: The key to set
        value: The value to set (any picklable Python object)
        host: Redis server host
        port: Redis server port
        
    Returns:
        True if successful, False otherwise
        
    Example:
        from robot_status.client_utils import set_to_status
        import numpy as np
        
        camera_matrix = np.eye(3)
        if set_to_status('ur15', 'camera_matrix', camera_matrix):
            print("Saved successfully")
    """
    try:
        client = RobotStatusClient(host=host, port=port)
        return client.set_status(namespace, key, value)
    except Exception:
        return False
