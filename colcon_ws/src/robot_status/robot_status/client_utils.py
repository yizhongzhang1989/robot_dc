#!/usr/bin/env python3
"""
Robot Status Client Utilities

Helper functions for easy interaction with robot_status service from other nodes.

Values are serialized using pickle, which supports:
- Basic types (int, float, str, bool)
- Collections (dict, list, tuple, set)
- Custom classes
- NumPy arrays
- Any picklable Python object

Usage:
    from robot_status.client_utils import RobotStatusClient
    import numpy as np
    
    # In your node (auto_spin=True by default - manages executor internally)
    client = RobotStatusClient(self)
    
    # Or if your node is already spinning (e.g., in a launch file):
    client = RobotStatusClient(self, auto_spin=False)
    
    # Set status - supports various data types
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
    
    # Clean shutdown (only needed if auto_spin=True)
    client.shutdown()
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import json
import pickle
import base64
import time
import threading


class RobotStatusClient:
    """Helper class for interacting with robot_status service."""
    
    def __init__(self, node: Node, **kwargs):
        """
        Initialize client.
        
        Args:
            node: ROS2 node instance
            **kwargs: Additional parameters:
                timeout_sec: Timeout for waiting for services (default: 5.0)
                auto_spin: If True, automatically create executor and spin in background thread (default: True)
                      
                      When to use auto_spin=True (default):
                      - Standalone scripts that don't have an existing executor
                      - Quick test scripts or utilities
                      - When you want the client to be fully self-contained
                      - Example: usage_example.py, one-off data collection scripts
                      
                      When to use auto_spin=False:
                      - Node is already spinning via launch file (e.g., ur15_web_node)
                      - Node is part of a larger system with existing executor
                      - You're managing the executor lifecycle yourself
                      - Multiple nodes in same process share an executor
                      
                      Note: The timer callback (_process_pending_futures) requires spinning
                      to work. With auto_spin=False, ensure your node is spinning elsewhere.
        """
        self.node = node
        timeout_sec = kwargs.get('timeout_sec', 5.0)
        auto_spin = kwargs.get('auto_spin', True)
        
        # Cache for async access (avoids spin_until_future_complete in callbacks/threads)
        # Cache stores tuples of (value, timestamp)
        self._cache = {}
        self._pending_futures = {}
        self._cache_ttl = 1.0  # Cache expires after 1 second
        
        # Executor and spin thread for self-contained operation
        self._executor = None
        self._spin_thread = None
        self._auto_spin = auto_spin
        
        try:
            from robot_status.srv import SetStatus, GetStatus, ListStatus, DeleteStatus
            
            # Create service clients
            self.set_client = node.create_client(SetStatus, 'robot_status/set')
            self.get_client = node.create_client(GetStatus, 'robot_status/get')
            self.list_client = node.create_client(ListStatus, 'robot_status/list')
            self.delete_client = node.create_client(DeleteStatus, 'robot_status/delete')
            
            # Wait for services
            self.node.get_logger().info("Waiting for robot_status services...")
            
            if not self.set_client.wait_for_service(timeout_sec=timeout_sec):
                raise TimeoutError("robot_status/set service not available")
            if not self.get_client.wait_for_service(timeout_sec=timeout_sec):
                raise TimeoutError("robot_status/get service not available")
            if not self.list_client.wait_for_service(timeout_sec=timeout_sec):
                raise TimeoutError("robot_status/list service not available")
            if not self.delete_client.wait_for_service(timeout_sec=timeout_sec):
                raise TimeoutError("robot_status/delete service not available")
            
            self.node.get_logger().info("Connected to robot_status services")
            
            # Start timer to process pending futures (10Hz)
            self._timer = node.create_timer(0.1, self._process_pending_futures)
            
            # Start executor and spin thread if auto_spin enabled
            if self._auto_spin:
                self._start_spinning()
            
        except ImportError as e:
            raise ImportError(f"Failed to import robot_status services: {e}")
    
    def _start_spinning(self):
        """Start executor and spin in background thread."""
        if self._spin_thread is None or not self._spin_thread.is_alive():
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()
            self.node.get_logger().debug("Started background executor spinning")
    
    def shutdown(self):
        """Clean shutdown of executor and spin thread."""
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
    
    def _process_pending_futures(self):
        """Process pending async futures and update cache."""
        for key in list(self._pending_futures.keys()):
            future = self._pending_futures[key]
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        if response.value:
                            # Unpickle and cache with timestamp
                            pickled = base64.b64decode(response.value.encode('ascii'))
                            self._cache[key] = (pickle.loads(pickled), time.time())
                        else:
                            # Value doesn't exist (was deleted), remove from cache
                            if key in self._cache:
                                del self._cache[key]
                                self.node.get_logger().debug(f"Removed deleted key from cache: {key}")
                except Exception as e:
                    self.node.get_logger().debug(f"Error processing future {key}: {e}")
                finally:
                    del self._pending_futures[key]
    
    def set_status(self, namespace: str, key: str, value, timeout_sec=2.0):
        """
        Set status value. Blocks until operation completes.
        
        Args:
            namespace: Robot or namespace identifier (e.g., 'robot1', 'shared')
            key: Status key (e.g., 'pose', 'battery')
            value: Value to set (any picklable Python object: dict, list, str, int, 
                   float, numpy arrays, custom classes, etc.)
            timeout_sec: Timeout for service call
            
        Returns:
            bool: True if successful, False otherwise
            
        Example:
            import numpy as np
            
            client.set_status('robot1', 'pose', {'x': 1.0, 'y': 2.0, 'z': 0.0})
            client.set_status('robot1', 'battery', 85)
            client.set_status('robot1', 'camera_matrix', np.eye(3))
            client.set_status('shared', 'mission_id', 'mission_001')
        """
        try:
            from robot_status.srv import SetStatus
            
            request = SetStatus.Request()
            request.ns = namespace
            request.key = key
            
            # Serialize value using pickle and encode as base64 string
            pickled = pickle.dumps(value)
            request.value = base64.b64encode(pickled).decode('ascii')
            
            # Call service synchronously
            future = self.set_client.call_async(request)
            
            # Spin until future completes (works even if node is already spinning elsewhere)
            import time
            start_time = time.time()
            while not future.done():
                if (time.time() - start_time) > timeout_sec:
                    self.node.get_logger().error(f"Timeout waiting for set_status {namespace}/{key}")
                    return False
                rclpy.spin_once(self.node, timeout_sec=0.01)
            
            result = future.result()
            if result and result.success:
                self.node.get_logger().debug(f"Set {namespace}.{key}")
                # Update cache with new value and timestamp
                cache_key = f"{namespace}/{key}"
                self._cache[cache_key] = (value, time.time())
                return True
            elif result:
                self.node.get_logger().error(f"Failed to set {namespace}.{key}: {result.message}")
                return False
            else:
                self.node.get_logger().error("Service call failed for set_status")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error in set_status: {e}")
            return False
    
    def get_status(self, namespace: str, key: str, timeout_sec=2.0):
        """
        Get status value. Thread-safe with automatic caching for async contexts.
        
        When called from timer callbacks or threads, returns cached value immediately
        and initiates async fetch if not cached. Safe to call repeatedly.
        
        Args:
            namespace: Robot or namespace identifier
            key: Status key to retrieve
            timeout_sec: Timeout for service call (only used in blocking mode)
            
        Returns:
            Value if found (original Python object), None if not found
            
        Example:
            import numpy as np
            
            pose = client.get_status('robot1', 'pose')  # Returns dict
            battery = client.get_status('robot1', 'battery')  # Returns int
            matrix = client.get_status('robot1', 'camera_matrix')  # Returns np.ndarray
        """
        try:
            from robot_status.srv import GetStatus
            
            cache_key = f"{namespace}/{key}"
            
            # Check cache first, validate TTL
            if cache_key in self._cache:
                cached_value, cached_time = self._cache[cache_key]
                # Check if cache entry has expired
                if time.time() - cached_time < self._cache_ttl:
                    return cached_value
                else:
                    # Cache expired, remove it
                    del self._cache[cache_key]
                    self.node.get_logger().debug(f"Cache expired for {cache_key}")
            
            # If not in cache and not pending, initiate async request
            if cache_key not in self._pending_futures:
                request = GetStatus.Request()
                request.ns = namespace
                request.key = key
                future = self.get_client.call_async(request)
                self._pending_futures[cache_key] = future
            
            # Return None for now, will be cached once future completes
            return None
                
        except Exception as e:
            self.node.get_logger().error(f"Error in get_status: {e}")
            return None
    
    def list_status(self, namespace: str = '', timeout_sec=2.0):
        """
        List status for all namespaces or specific namespace.
        
        Args:
            namespace: Empty string = list all, otherwise specific namespace
            timeout_sec: Timeout for service call
            
        Returns:
            dict: Status dictionary, e.g., {'robot1': {'pose': {...}, 'battery': 85}}
                  Returns empty dict if error or no status
            
        Example:
            all_status = client.list_status()  # All robots
            robot1_status = client.list_status('robot1')  # Just robot1
        """
        try:
            from robot_status.srv import ListStatus
            
            request = ListStatus.Request()
            request.ns = namespace
            
            future = self.list_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            
            if future.result():
                result = future.result()
                if result.success:
                    # Parse the status dict which contains base64-encoded pickled values
                    status_dict = json.loads(result.status_dict)
                    # Decode each value
                    for namespace in status_dict:
                        for key in status_dict[namespace]:
                            try:
                                pickled = base64.b64decode(status_dict[namespace][key].encode('ascii'))
                                status_dict[namespace][key] = pickle.loads(pickled)
                            except Exception:
                                # Keep as-is if not pickle-encoded (backward compatibility)
                                try:
                                    status_dict[namespace][key] = json.loads(status_dict[namespace][key])
                                except (json.JSONDecodeError, TypeError):
                                    pass
                    return status_dict
                else:
                    self.node.get_logger().error(f"Failed to list status: {result.message}")
                    return {}
            else:
                self.node.get_logger().error(f"Service call failed for list_status")
                return {}
                
        except Exception as e:
            self.node.get_logger().error(f"Error in list_status: {e}")
            return {}
    
    def get_namespaces(self, timeout_sec=2.0):
        """
        Get list of all namespaces.
        
        Args:
            timeout_sec: Timeout for service call
            
        Returns:
            list: List of namespace strings, e.g., ['shared', 'robot1', 'robot2']
            
        Example:
            namespaces = client.get_namespaces()
            for ns in namespaces:
                print(f"Found namespace: {ns}")
        """
        try:
            from robot_status.srv import ListStatus
            
            request = ListStatus.Request()
            request.ns = ''
            
            future = self.list_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            
            if future.result():
                result = future.result()
                if result.success:
                    return result.namespaces
                else:
                    return []
            else:
                return []
                
        except Exception as e:
            self.node.get_logger().error(f"Error in get_namespaces: {e}")
            return []
    
    def delete_status(self, namespace: str, key: str = '', timeout_sec=2.0):
        """
        Delete status value or entire namespace.
        
        Args:
            namespace: Robot or namespace identifier
            key: Status key to delete (empty = delete entire namespace)
            timeout_sec: Timeout for service call
            
        Returns:
            bool: True if successful, False otherwise
            
        Example:
            client.delete_status('robot1', 'pose')  # Delete specific key
            client.delete_status('robot1')  # Delete entire namespace
        """
        try:
            from robot_status.srv import DeleteStatus
            
            request = DeleteStatus.Request()
            request.ns = namespace
            request.key = key
            
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            
            if future.result():
                result = future.result()
                if result.success:
                    # Remove from cache after successful deletion
                    if key:
                        # Delete specific key
                        cache_key = f"{namespace}/{key}"
                        if cache_key in self._cache:
                            del self._cache[cache_key]
                            self.node.get_logger().debug(f"Removed {cache_key} from cache")
                    else:
                        # Delete entire namespace
                        keys_to_remove = [k for k in self._cache.keys() if k.startswith(f"{namespace}/")]
                        for k in keys_to_remove:
                            del self._cache[k]
                        if keys_to_remove:
                            self.node.get_logger().debug(f"Removed {len(keys_to_remove)} keys from namespace {namespace} cache")
                    
                    self.node.get_logger().info(result.message)
                    return True
                else:
                    self.node.get_logger().error(f"Failed to delete: {result.message}")
                    return False
            else:
                self.node.get_logger().error(f"Service call failed for delete_status")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error in delete_status: {e}")
            return False
    
    def wait_for_services(self, timeout_sec=10.0):
        """
        Wait for robot_status services to become available.
        
        Args:
            timeout_sec: Timeout for waiting
            
        Returns:
            bool: True if all services available, False otherwise
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout_sec:
            if (self.set_client.service_is_ready() and
                self.get_client.service_is_ready() and
                self.list_client.service_is_ready() and
                self.delete_client.service_is_ready()):
                return True
            time.sleep(0.1)
        
        return False


# Standalone helper functions for simple get/set operations without client instance
def get_from_status(node: Node, namespace: str, key: str, timeout: float = 2.0):
    """
    Standalone helper function to get a value from robot_status service.
    
    Args:
        node: ROS2 node instance
        namespace: The namespace to query
        key: The key to retrieve
        timeout: Service call timeout in seconds
        
    Returns:
        The original Python object if found, None otherwise
        
    Example:
        from robot_status.client_utils import get_from_status
        import numpy as np
        
        matrix = get_from_status(self, 'ur15', 'camera_matrix')
        if matrix is not None:
            # matrix is already a numpy array, no conversion needed
            print(matrix.shape)
    """
    try:
        from robot_status.srv import GetStatus
        
        client = node.create_client(GetStatus, '/robot_status/get')
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        
        request = GetStatus.Request()
        request.ns = namespace
        request.key = key
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        
        if future.result() is not None:
            response = future.result()
            if response.success and response.value:
                # Try to unpickle (base64 encoded)
                try:
                    pickled = base64.b64decode(response.value.encode('ascii'))
                    return pickle.loads(pickled)
                except Exception:
                    # Fallback: return raw value for backward compatibility
                    return response.value
        return None
    except Exception as e:
        node.get_logger().debug(f"Error getting {namespace}/{key} from status: {e}")
        return None


def set_to_status(node: Node, namespace: str, key: str, value, timeout: float = 2.0):
    """
    Standalone helper function to set a value in robot_status service.
    
    Args:
        node: ROS2 node instance
        namespace: The namespace to use
        key: The key to set
        value: The value to set (any picklable Python object)
        timeout: Service call timeout in seconds
        
    Returns:
        True if successful, False otherwise
        
    Example:
        from robot_status.client_utils import set_to_status
        import numpy as np
        
        # No conversion needed - pass numpy array directly
        camera_matrix = np.eye(3)
        if set_to_status(self, 'ur15', 'camera_matrix', camera_matrix):
            print("Saved successfully")
    """
    try:
        from robot_status.srv import SetStatus
        
        client = node.create_client(SetStatus, '/robot_status/set')
        if not client.wait_for_service(timeout_sec=timeout):
            node.get_logger().debug(f"robot_status service not available for setting {namespace}/{key}")
            return False
        
        request = SetStatus.Request()
        request.ns = namespace
        request.key = key
        # Serialize value using pickle and encode as base64 string
        pickled = pickle.dumps(value)
        request.value = base64.b64encode(pickled).decode('ascii')
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        
        if future.result() is not None:
            response = future.result()
            return response.success
        return False
    except Exception as e:
        node.get_logger().warning(f"Error setting {namespace}/{key} to status: {e}")
        return False
