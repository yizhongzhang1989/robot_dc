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
from common.workspace_utils import get_workspace_root, get_temp_directory


class RobotStatusNode(Node):
    """ROS2 node for managing robot status using hierarchical parameters."""
    
    def __init__(self):
        super().__init__(
            'robot_status_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        # Get auto-save file path (already declared by automatically_declare_parameters_from_overrides)
        if not self.has_parameter('auto_save_file_path'):
            # Default: temp/robot_status_auto_save.json in workspace root
            default_path = self._get_default_save_path()
            self.declare_parameter('auto_save_file_path', default_path)
        
        auto_save_path = self.get_parameter('auto_save_file_path').value
        # Convert to absolute path if relative
        self.auto_save_file_path = str(Path(auto_save_path).expanduser().resolve())
        
        # Thread lock for thread-safe parameter operations
        self.lock = threading.Lock()
        
        # Load existing status from file
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
            self.get_logger().info("Robot Status Node Ready")
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
            # Use common utility to get temp directory
            temp_dir = get_temp_directory()
            return str(Path(temp_dir) / 'robot_status_auto_save.json')
        except Exception as e:
            self.get_logger().warn(f"Failed to determine temp directory: {e}")
            return 'robot_status_auto_save.json'
    
    def _load_status_from_file(self):
        """Load status from auto-save JSON file."""
        if not os.path.exists(self.auto_save_file_path):
            self.get_logger().info(f"Auto-save file not found: {self.auto_save_file_path}")
            return
        
        try:
            with open(self.auto_save_file_path, 'r') as f:
                data = json.load(f)
            
            # Load all namespace.key pairs
            count = 0
            for namespace, keys in data.items():
                for key, value in keys.items():
                    param_name = f"{namespace}.{key}"
                    try:
                        # Handle new format: {"pickle": "...", "json": {...}}
                        if isinstance(value, dict) and "pickle" in value:
                            pickle_str = value["pickle"]
                        else:
                            # Backward compatibility: old format (direct pickle string)
                            pickle_str = value
                        
                        # Use the pickle string directly
                        self.declare_parameter(param_name, pickle_str)
                        count += 1
                    except Exception as e:
                        self.get_logger().warn(f"Failed to load {param_name}: {e}")
            
            self.get_logger().info(f"Loaded {count} parameters from {self.auto_save_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load auto-save file: {e}")
    
    def _save_status_to_file(self):
        """Save current status to auto-save JSON file."""
        try:
            # Build status tree
            status_tree = {}
            
            with self.lock:
                param_names = list(self._parameters.keys())
            
            for param_name in param_names:
                # Skip non-hierarchical parameters
                if '.' not in param_name:
                    continue
                
                # Split namespace and key
                parts = param_name.split('.', 1)
                namespace = parts[0]
                key = parts[1] if len(parts) > 1 else ""
                
                # Initialize namespace dict if needed
                if namespace not in status_tree:
                    status_tree[namespace] = {}
                
                # Get parameter value (pickled base64 string)
                try:
                    with self.lock:
                        value_str = self.get_parameter(param_name).value
                    
                    # Create dict with pickle string
                    value_dict = {"pickle": value_str}
                    
                    # Try to add JSON representation if possible
                    try:
                        # Unpickle to get the actual object
                        pickled = base64.b64decode(value_str.encode('ascii'))
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
                except Exception:
                    pass
            
            # Ensure the directory exists before writing
            save_path = Path(self.auto_save_file_path)
            save_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Write to file
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
            request.value: JSON-encoded value
        """
        param_name = f"{request.ns}.{request.key}"
        
        # Validate that the value can be decoded (pickle base64 or JSON)
        try:
            # Try pickle format first
            pickled = base64.b64decode(request.value.encode('ascii'))
            pickle.loads(pickled)
        except Exception:
            # Try JSON format for backward compatibility
            try:
                json.loads(request.value)
            except json.JSONDecodeError:
                # Allow plain strings too
                pass
        
        try:
            with self.lock:
                # Declare parameter if it doesn't exist
                if not self.has_parameter(param_name):
                    self.declare_parameter(param_name, request.value)
                else:
                    self.set_parameters([Parameter(param_name, value=request.value)])
            
            response.success = True
            response.message = f"Set {param_name}"
            
            # Don't log frequently updated parameters to reduce log spam
            # Only log other parameters for debugging
            if param_name not in ['ur15.joint_positions', 'ur15.tcp_pose']:
                self.get_logger().info(f"Set: {param_name}")
            # else: silently skip logging for joint_positions and tcp_pose
            
            # Auto-save after successful set
            self._save_status_to_file()
            
        except Exception as e:
            response.success = False
            response.message = f"Error setting parameter: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def get_status_callback(self, request, response):
        """
        Get status callback: retrieve namespace.key
        
        Args:
            request.ns: Robot or namespace identifier
            request.key: Status key to retrieve
        """
        param_name = f"{request.ns}.{request.key}"
        
        try:
            with self.lock:
                value = self.get_parameter(param_name).value
            
            response.success = True
            response.value = value
            response.message = "Success"
            self.get_logger().debug(f"Get: {param_name} = {value[:100]}...")
            
        except Exception as e:
            response.success = False
            response.value = ""
            response.message = f"Parameter {param_name} not found"
            self.get_logger().warn(response.message)
        
        return response
    
    def list_status_callback(self, request, response):
        """
        List status callback: return all status or filter by namespace
        
        Args:
            request.ns: Empty = list all, otherwise filter by namespace
        """
        try:
            # Build status tree by iterating through _parameters
            status_tree = {}
            namespaces = set()
            
            with self.lock:
                # Get all parameter names from the node's internal parameter storage
                param_names = list(self._parameters.keys())
            
            for param_name in param_names:
                # Skip non-hierarchical parameters
                if '.' not in param_name:
                    continue
                
                # Split namespace and key
                parts = param_name.split('.', 1)
                namespace = parts[0]
                key = parts[1] if len(parts) > 1 else ""
                
                # Filter by requested namespace if specified
                if request.ns and namespace != request.ns:
                    continue
                
                namespaces.add(namespace)
                
                # Initialize namespace dict if needed
                if namespace not in status_tree:
                    status_tree[namespace] = {}
                
                # Get parameter value (already in base64-encoded pickle format)
                try:
                    with self.lock:
                        value = self.get_parameter(param_name).value
                    # Keep the base64-encoded pickle string for transmission
                    status_tree[namespace][key] = value
                except Exception as e:
                    self.get_logger().warn(f"Error reading {param_name}: {e}")
            
            # Build response
            response.success = True
            response.namespaces = sorted(list(namespaces))
            response.status_dict = json.dumps(status_tree, indent=2)
            response.message = f"Found {len(namespaces)} namespace(s)"
            
            self.get_logger().debug(f"List: {len(namespaces)} namespaces, {len(param_names)} total params")
            
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
            if request.key:
                # Delete specific key
                param_name = f"{request.ns}.{request.key}"
                deleted = False
                
                with self.lock:
                    if self.has_parameter(param_name):
                        # ROS2 doesn't have undeclare_parameter, so we set it to empty
                        # and remove from internal dict
                        if param_name in self._parameters:
                            del self._parameters[param_name]
                        
                        response.success = True
                        response.message = f"Deleted {param_name}"
                        self.get_logger().info(f"Deleted: {param_name}")
                        deleted = True
                    else:
                        response.success = False
                        response.message = f"Parameter {param_name} not found"
                        self.get_logger().warn(response.message)
                
                # Auto-save after successful delete (outside lock to avoid deadlock)
                if deleted:
                    self._save_status_to_file()
            else:
                # Delete entire namespace
                deleted_count = 0
                params_to_delete = []
                
                with self.lock:
                    # Find all parameters in this namespace
                    for param_name in list(self._parameters.keys()):
                        if param_name.startswith(f"{request.ns}."):
                            params_to_delete.append(param_name)
                    
                    # Delete them
                    for param_name in params_to_delete:
                        if param_name in self._parameters:
                            del self._parameters[param_name]
                            deleted_count += 1
                
                if deleted_count > 0:
                    response.success = True
                    response.message = f"Deleted namespace '{request.ns}' ({deleted_count} parameters)"
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = f"Namespace '{request.ns}' not found or empty"
                    self.get_logger().warn(response.message)
                
                # Auto-save after successful delete (outside lock to avoid deadlock)
                if deleted_count > 0:
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
