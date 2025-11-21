#!/usr/bin/env python3
"""
Robot Status Management Node

Provides centralized status storage using ROS2 parameters with hierarchical namespaces.
Supports dynamic robot addition without code changes.

Services:
    - robot_status/set: Set status for any namespace.key
    - robot_status/get: Get status for any namespace.key
    - robot_status/list: List all status or filter by namespace
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import json
import pickle
import base64
import threading
import sys
import os
from pathlib import Path

try:
    from .redis_backend import get_redis_backend, is_redis_available
except ImportError:
    from robot_status_redis.redis_backend import get_redis_backend, is_redis_available


class RobotStatusNode(Node):
    """ROS2 node for managing robot status using Redis backend with ROS2 service interface."""
    
    def __init__(self):
        super().__init__(
            'robot_status_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        # Initialize Redis backend
        if not is_redis_available():
            error_msg = (
                "Redis is not available. Please install and start Redis:\n"
                "  sudo apt-get install redis-server\n"
                "  sudo systemctl start redis-server\n"
                "  pip3 install redis\n"
                "Verify with: redis-cli ping"
            )
            self.get_logger().error(error_msg)
            raise ConnectionError(error_msg)
        
        self._redis_backend = get_redis_backend()
        if self._redis_backend is None:
            raise ConnectionError("Failed to connect to Redis")
        
        self.get_logger().info("✓ Connected to Redis backend")
        
        # Get auto-save file path (already declared by automatically_declare_parameters_from_overrides)
        if not self.has_parameter('auto_save_file_path'):
            # Default: temp/robot_status_auto_save.json in workspace root
            default_path = self._get_default_save_path()
            self.declare_parameter('auto_save_file_path', default_path)
        
        auto_save_path = self.get_parameter('auto_save_file_path').value
        # Convert to absolute path if relative
        self.auto_save_file_path = str(Path(auto_save_path).expanduser().resolve())
        
        # Thread lock for thread-safe operations
        self.lock = threading.Lock()
        
        # Clear all existing Redis data on startup
        self.get_logger().info("Clearing all existing Redis data...")
        self._redis_backend._client.flushdb()
        self.get_logger().info("✓ Redis data cleared")
        
        # Load existing status from file to Redis
        self._load_status_from_file()
        
        # Import service types (will be available after build)
        try:
            from robot_status.srv import SetStatus, GetStatus, ListStatus, DeleteStatus
            
            # Create services
            self.set_service = self.create_service(
                SetStatus,
                'robot_status/set',
                self.set_status_callback
            )
            
            self.get_service = self.create_service(
                GetStatus,
                'robot_status/get',
                self.get_status_callback
            )
            
            self.list_service = self.create_service(
                ListStatus,
                'robot_status/list',
                self.list_status_callback
            )
            
            self.delete_service = self.create_service(
                DeleteStatus,
                'robot_status/delete',
                self.delete_status_callback
            )
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("Robot Status Redis Node Ready")
            self.get_logger().info("Services available:")
            self.get_logger().info("  - /robot_status/set")
            self.get_logger().info("  - /robot_status/get")
            self.get_logger().info("  - /robot_status/list")
            self.get_logger().info("  - /robot_status/delete")
            self.get_logger().info(f"Auto-save file: {self.auto_save_file_path}")
            self.get_logger().info("=" * 60)
            
        except ImportError as e:
            self.get_logger().error(f"Failed to import service types: {e}")
            self.get_logger().error("Make sure the package is built: colcon build --packages-select robot_status")
            sys.exit(1)
    
    def _get_default_save_path(self):
        """Get the default auto-save file path in robot_dc/temp directory."""
        try:
            # Find robot_dc root directory
            current_dir = Path.cwd()
            robot_dc_root = None
            
            # Search up the directory tree for robot_dc
            for parent in [current_dir] + list(current_dir.parents):
                if parent.name == 'robot_dc':
                    robot_dc_root = parent
                    break
            
            if robot_dc_root is None:
                # If not found, use current directory
                self.get_logger().warn("Could not find robot_dc root, using current directory")
                robot_dc_root = current_dir
            
            # Create temp directory in robot_dc root if it doesn't exist
            temp_dir = robot_dc_root / 'temp'
            temp_dir.mkdir(parents=True, exist_ok=True)
            
            return str(temp_dir / 'robot_status_auto_save.json')
        except Exception as e:
            self.get_logger().warn(f"Failed to determine robot_dc root: {e}")
            return 'robot_status_auto_save.json'
    
    def _load_status_from_file(self):
        """Load status from auto-save JSON file into Redis."""
        if not os.path.exists(self.auto_save_file_path):
            self.get_logger().info(f"Auto-save file not found: {self.auto_save_file_path}")
            return
        
        try:
            with open(self.auto_save_file_path, 'r') as f:
                data = json.load(f)
            
            # Load all namespace.key pairs into Redis
            count = 0
            for namespace, keys in data.items():
                for key, value in keys.items():
                    try:
                        # Handle new format: {"pickle": "...", "json": {...}}
                        if isinstance(value, dict) and "pickle" in value:
                            pickle_str = value["pickle"]
                        else:
                            # Backward compatibility: old format (direct pickle string)
                            pickle_str = value
                        
                        # Decode and store in Redis
                        pickled = base64.b64decode(pickle_str.encode('ascii'))
                        obj = pickle.loads(pickled)
                        self._redis_backend.set_status(namespace, key, obj)
                        count += 1
                    except Exception as e:
                        self.get_logger().warn(f"Failed to load {namespace}.{key}: {e}")
            
            self.get_logger().info(f"Loaded {count} items from {self.auto_save_file_path} into Redis")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load auto-save file: {e}")
    
    def _save_status_to_file(self):
        """Save current Redis status to auto-save JSON file."""
        try:
            # Get all status from Redis
            with self.lock:
                status_tree = self._redis_backend.list_status()
            
            # Ensure the directory exists before writing
            save_path = Path(self.auto_save_file_path)
            save_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Write to file (status_tree already has pickle+json format)
            with open(self.auto_save_file_path, 'w') as f:
                json.dump(status_tree, f, indent=2)
            
            self.get_logger().debug(f"Saved status to {self.auto_save_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save auto-save file: {e}")
    
    def set_status_callback(self, request, response):
        """
        Set status callback: namespace.key = value
        
        Args:
            request.ns: Robot or namespace identifier
            request.key: Status key
            request.value: Pickled base64-encoded value
        """
        try:
            # Decode the pickled bytes but don't unpickle here (avoid __main__ class issues)
            pickled = base64.b64decode(request.value.encode('ascii'))
            
            # Try to unpickle to create JSON representation if possible
            value_dict = {"pickle": request.value}
            try:
                obj = pickle.loads(pickled)
                # Try to create JSON representation
                try:
                    json_str = json.dumps(obj)
                    value_dict["json"] = json.loads(json_str)
                except (TypeError, ValueError):
                    # Try tolist() method for numpy arrays
                    if hasattr(obj, 'tolist'):
                        try:
                            list_obj = obj.tolist()
                            json_str = json.dumps(list_obj)
                            value_dict["json"] = json.loads(json_str)
                        except (TypeError, ValueError, AttributeError):
                            pass
            except Exception as e:
                # Can't unpickle (e.g., __main__ classes), just store pickle
                self.get_logger().debug(f"Storing pickled data only (cannot unpickle): {e}")
            
            # Store the value_dict directly in Redis using the backend's client
            # Redis backend stores value_dict as JSON string
            redis_key = f"robot_status:{request.ns}:{request.key}"
            with self.lock:
                success = self._redis_backend._client.set(redis_key, json.dumps(value_dict))
            
            if success:
                response.success = True
                response.message = f"Set {request.ns}.{request.key}"
                self.get_logger().info(f"Set: {request.ns}.{request.key}")
                
                # Auto-save to file after successful set
                self._save_status_to_file()
            else:
                response.success = False
                response.message = "Failed to set status in Redis"
                self.get_logger().error(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Error setting status: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def get_status_callback(self, request, response):
        """
        Get status callback: retrieve namespace.key
        
        Args:
            request.ns: Robot or namespace identifier
            request.key: Status key to retrieve
        """
        try:
            with self.lock:
                success, obj = self._redis_backend.get_status(request.ns, request.key)
            
            if success and obj is not None:
                # Pickle and encode as base64
                pickled = pickle.dumps(obj)
                value_str = base64.b64encode(pickled).decode('ascii')
                
                response.success = True
                response.value = value_str
                response.message = "Success"
                self.get_logger().debug(f"Get: {request.ns}.{request.key}")
            else:
                response.success = False
                response.value = ""
                response.message = f"Key {request.ns}.{request.key} not found"
                self.get_logger().warn(response.message)
            
        except Exception as e:
            response.success = False
            response.value = ""
            response.message = f"Error getting status: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def list_status_callback(self, request, response):
        """
        List status callback: return all status or filter by namespace
        
        Args:
            request.ns: Empty = list all, otherwise filter by namespace
        """
        try:
            with self.lock:
                # Get status from Redis
                namespace_filter = request.ns if request.ns else None
                status_dict = self._redis_backend.list_status(namespace_filter)
            
            # Extract namespaces
            namespaces = sorted(list(status_dict.keys()))
            
            # Convert to pickle base64 format for service response
            result_tree = {}
            for namespace, keys in status_dict.items():
                result_tree[namespace] = {}
                for key, value_dict in keys.items():
                    # value_dict should have 'pickle' key with base64 string
                    if isinstance(value_dict, dict) and 'pickle' in value_dict:
                        result_tree[namespace][key] = value_dict['pickle']
                    else:
                        # Legacy format or direct value, re-pickle it
                        pickled = pickle.dumps(value_dict)
                        result_tree[namespace][key] = base64.b64encode(pickled).decode('ascii')
            
            response.success = True
            response.namespaces = namespaces
            response.status_dict = json.dumps(result_tree, indent=2)
            response.message = f"Found {len(namespaces)} namespace(s)"
            
            self.get_logger().debug(f"List: {len(namespaces)} namespaces")
            
        except Exception as e:
            response.success = False
            response.namespaces = []
            response.status_dict = "{}"
            response.message = f"Error listing status: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def delete_status_callback(self, request, response):
        """
        Delete status callback: remove namespace.key or entire namespace
        
        Args:
            request.ns: Robot or namespace identifier
            request.key: Status key to delete (empty = delete entire namespace)
        """
        try:
            with self.lock:
                if request.key:
                    # Delete specific key
                    success = self._redis_backend.delete_status(request.ns, request.key)
                    
                    if success:
                        response.success = True
                        response.message = f"Deleted {request.ns}.{request.key}"
                        self.get_logger().info(f"Deleted: {request.ns}.{request.key}")
                    else:
                        response.success = False
                        response.message = f"Key {request.ns}.{request.key} not found"
                        self.get_logger().warn(response.message)
                else:
                    # Delete entire namespace
                    success = self._redis_backend.delete_status(request.ns)
                    
                    if success:
                        response.success = True
                        response.message = f"Deleted namespace '{request.ns}'"
                        self.get_logger().info(response.message)
                    else:
                        response.success = False
                        response.message = f"Namespace '{request.ns}' not found or empty"
                        self.get_logger().warn(response.message)
            
            # Auto-save after successful delete
            if response.success:
                self._save_status_to_file()
                    
        except Exception as e:
            response.success = False
            response.message = f"Error deleting: {str(e)}"
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = RobotStatusNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
