#!/usr/bin/env python3
"""
Robot Status Client Utilities

Helper functions for easy interaction with robot_status service from other nodes.

Usage:
    from robot_status.client_utils import RobotStatusClient
    
    # In your node
    client = RobotStatusClient(self)
    
    # Set status
    client.set_status('robot1', 'pose', {'x': 1.5, 'y': 2.3, 'z': 0.0})
    
    # Get status
    pose = client.get_status('robot1', 'pose')
    
    # List all status
    all_status = client.list_status()
    
    # List specific namespace
    robot1_status = client.list_status('robot1')
"""

import rclpy
from rclpy.node import Node
import json
import time


class RobotStatusClient:
    """Helper class for interacting with robot_status service."""
    
    def __init__(self, node: Node, timeout_sec=5.0):
        """
        Initialize client.
        
        Args:
            node: ROS2 node instance
            timeout_sec: Timeout for waiting for services
        """
        self.node = node
        
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
            
        except ImportError as e:
            raise ImportError(f"Failed to import robot_status services: {e}")
    
    def set_status(self, namespace: str, key: str, value, timeout_sec=2.0):
        """
        Set status value.
        
        Args:
            namespace: Robot or namespace identifier (e.g., 'robot1', 'shared')
            key: Status key (e.g., 'pose', 'battery')
            value: Value to set (dict, list, str, int, float)
            timeout_sec: Timeout for service call
            
        Returns:
            bool: True if successful, False otherwise
            
        Example:
            client.set_status('robot1', 'pose', {'x': 1.0, 'y': 2.0, 'z': 0.0})
            client.set_status('robot1', 'battery', 85)
            client.set_status('shared', 'mission_id', 'mission_001')
        """
        try:
            from robot_status.srv import SetStatus
            
            request = SetStatus.Request()
            request.ns = namespace
            request.key = key
            
            # Convert value to JSON string
            if isinstance(value, (dict, list)):
                request.value = json.dumps(value)
            else:
                request.value = json.dumps(value)
            
            future = self.set_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            
            if future.result():
                result = future.result()
                if result.success:
                    self.node.get_logger().debug(f"Set {namespace}.{key}")
                    return True
                else:
                    self.node.get_logger().error(f"Failed to set {namespace}.{key}: {result.message}")
                    return False
            else:
                self.node.get_logger().error(f"Service call failed for set_status")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error in set_status: {e}")
            return False
    
    def get_status(self, namespace: str, key: str, timeout_sec=2.0):
        """
        Get status value.
        
        Args:
            namespace: Robot or namespace identifier
            key: Status key to retrieve
            timeout_sec: Timeout for service call
            
        Returns:
            Value if found (parsed from JSON), None if not found
            
        Example:
            pose = client.get_status('robot1', 'pose')  # Returns dict
            battery = client.get_status('robot1', 'battery')  # Returns int
        """
        try:
            from robot_status.srv import GetStatus
            
            request = GetStatus.Request()
            request.ns = namespace
            request.key = key
            
            future = self.get_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            
            if future.result():
                result = future.result()
                if result.success:
                    # Try to parse JSON
                    try:
                        return json.loads(result.value)
                    except json.JSONDecodeError:
                        # Return as string if not valid JSON
                        return result.value
                else:
                    self.node.get_logger().debug(f"{namespace}.{key} not found")
                    return None
            else:
                self.node.get_logger().error(f"Service call failed for get_status")
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
                    return json.loads(result.status_dict)
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
