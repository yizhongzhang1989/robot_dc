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
        
        # Get Redis configuration from parameters
        if not self.has_parameter('redis_host'):
            self.declare_parameter('redis_host', 'localhost')
        if not self.has_parameter('redis_port'):
            self.declare_parameter('redis_port', 6379)
        if not self.has_parameter('redis_db'):
            self.declare_parameter('redis_db', 0)
        if not self.has_parameter('redis_password'):
            self.declare_parameter('redis_password', '')
        
        redis_host = self.get_parameter('redis_host').value
        redis_port = self.get_parameter('redis_port').value
        redis_db = self.get_parameter('redis_db').value
        redis_password = self.get_parameter('redis_password').value
        redis_password = redis_password if redis_password else None
        
        # Initialize Redis backend with config parameters
        if not is_redis_available(redis_host, redis_port, redis_db, redis_password):
            error_msg = (
                f"Redis is not available at {redis_host}:{redis_port}. Please install and start Redis:\n"
                "  sudo apt-get install redis-server\n"
                "  sudo systemctl start redis-server\n"
                "  pip3 install redis\n"
                "Verify with: redis-cli ping"
            )
            self.get_logger().error(error_msg)
            raise ConnectionError(error_msg)
        
        self._redis_backend = get_redis_backend(redis_host, redis_port, redis_db, redis_password)
        if self._redis_backend is None:
            raise ConnectionError(f"Failed to connect to Redis at {redis_host}:{redis_port}")
        
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
            
            # Setup Redis keyspace notifications for auto-save on changes
            self._setup_redis_notifications()
            self.get_logger().info("Auto-save enabled (triggers on Redis changes)")
            
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
                        # Handle format: {"pickle": "...", "json": {...}}
                        if isinstance(value, dict) and "pickle" in value:
                            pickle_str = value["pickle"]
                        else:
                            # Backward compatibility: direct pickle string
                            pickle_str = value
                        
                        # Store pickle string directly in Redis
                        redis_key = f"robot_status:{namespace}:{key}"
                        self._redis_backend._client.set(redis_key, pickle_str)
                        count += 1
                    except Exception as e:
                        self.get_logger().warn(f"Failed to load {namespace}.{key}: {e}")
            
            self.get_logger().info(f"Loaded {count} items from {self.auto_save_file_path} into Redis")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load auto-save file: {e}")
    
    def _setup_redis_notifications(self):
        """Setup Redis keyspace notifications to trigger auto-save on changes."""
        try:
            # Enable keyspace notifications for set and delete operations
            self._redis_backend._client.config_set('notify-keyspace-events', 'KEA')
            
            # Create a separate Redis connection for pubsub (non-blocking)
            import redis
            self._pubsub_client = redis.Redis(
                host=self._redis_backend.host,
                port=self._redis_backend.port,
                db=self._redis_backend.db,
                password=self._redis_backend.password,
                decode_responses=True
            )
            self._pubsub = self._pubsub_client.pubsub()
            
            # Subscribe to keyspace notifications for robot_status keys
            pattern = f'__keyspace@{self._redis_backend.db}__:robot_status:*'
            self._pubsub.psubscribe(pattern)
            
            # Start background thread to listen for notifications
            self._notification_thread = threading.Thread(
                target=self._redis_notification_listener,
                daemon=True
            )
            self._notification_thread.start()
            self._save_pending = False
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup Redis notifications: {e}")
            self.get_logger().warn("Auto-save will only work via service API")
    
    def _redis_notification_listener(self):
        """Background thread that listens for Redis keyspace notifications."""
        try:
            for message in self._pubsub.listen():
                if message['type'] == 'pmessage':
                    # Mark that a save is needed
                    if not self._save_pending:
                        self._save_pending = True
                        # Debounce: wait a bit before saving (avoid rapid saves)
                        threading.Timer(1.0, self._trigger_save).start()
        except Exception as e:
            self.get_logger().error(f"Redis notification listener error: {e}")
    
    def _trigger_save(self):
        """Trigger auto-save after debounce period."""
        if self._save_pending:
            self._save_pending = False
            self._save_status_to_file()
    
    def _save_status_to_file(self):
        """Save current Redis status to auto-save JSON file."""
        try:
            # Get all status from Redis (returns pickle strings)
            with self.lock:
                raw_status = self._redis_backend.list_status()
            
            # Convert to format matching original robot_status: {"pickle": "...", "json": ...}
            status_tree = {}
            for namespace, keys in raw_status.items():
                status_tree[namespace] = {}
                for key, pickle_str in keys.items():
                    # Create dict with pickle string
                    value_dict = {"pickle": pickle_str}
                    
                    # Try to add JSON representation if possible
                    try:
                        # Unpickle to get the actual object
                        pickled = base64.b64decode(pickle_str.encode('ascii'))
                        obj = pickle.loads(pickled)
                        
                        # Try direct JSON serialization first
                        try:
                            json_str = json.dumps(obj)
                            value_dict["json"] = json.loads(json_str)
                        except (TypeError, ValueError):
                            # Direct serialization failed, try tolist() method
                            if hasattr(obj, 'tolist'):
                                try:
                                    list_obj = obj.tolist()
                                    json_str = json.dumps(list_obj)
                                    value_dict["json"] = json.loads(json_str)
                                except (TypeError, ValueError, AttributeError):
                                    # tolist() also failed, no JSON representation
                                    pass
                    except Exception:
                        # Unpickling failed, just keep pickle
                        pass
                    
                    status_tree[namespace][key] = value_dict
            
            # Ensure the directory exists before writing
            save_path = Path(self.auto_save_file_path)
            save_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Write to file (same format as original robot_status)
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
            # Store pickle string directly (same as original robot_status stores in ROS parameters)
            redis_key = f"robot_status:{request.ns}:{request.key}"
            with self.lock:
                success = self._redis_backend._client.set(redis_key, request.value)
            
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
            # Get pickle string directly from Redis (avoid unpickle/re-pickle cycle)
            redis_key = f"robot_status:{request.ns}:{request.key}"
            with self.lock:
                pickle_str = self._redis_backend._client.get(redis_key)
            
            if pickle_str is not None:
                response.success = True
                response.value = pickle_str
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
            
            # status_dict already contains pickle strings (same format as original robot_status)
            result_tree = status_dict
            
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
