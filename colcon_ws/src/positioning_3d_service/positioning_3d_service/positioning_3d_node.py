#!/usr/bin/env python3
"""
3D Positioning Service ROS2 Node

This node wraps the positioning_3d web service and manages its lifecycle.
It launches the Flask web application that handles multi-view triangulation
using FlowFormer++ for feature tracking.
"""

import rclpy
from rclpy.node import Node
import subprocess
import signal
import sys
import os
import time
from pathlib import Path


class Positioning3DServiceNode(Node):
    """ROS2 node that manages the 3D positioning web service."""
    
    def __init__(self):
        super().__init__('positioning_3d_service_node')
        
        # Declare parameters
        self.declare_parameter('ffpp_url', 'http://msraig-ubuntu-4.guest.corp.microsoft.com:8001')
        self.declare_parameter('dataset_path', './dataset')
        self.declare_parameter('port', 8004)
        self.declare_parameter('host', '0.0.0.0')
        
        # Get parameters
        self.ffpp_url = self.get_parameter('ffpp_url').value
        self.dataset_path = self.get_parameter('dataset_path').value
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        
        # Process handle (initialize early to avoid AttributeError in cleanup)
        self.process = None
        
        # Find the app.py path (in ThirdParty/robot_vision submodule)
        # Use ROS_WORKSPACE environment variable or search from common locations
        workspace_root = None
        
        # Try to get from environment
        if 'ROS_WORKSPACE' in os.environ:
            workspace_root = Path(os.environ['ROS_WORKSPACE']).parent
        elif 'COLCON_PREFIX_PATH' in os.environ:
            # COLCON_PREFIX_PATH points to install directory
            colcon_path = Path(os.environ['COLCON_PREFIX_PATH'].split(':')[0])
            workspace_root = colcon_path.parent.parent  # Go up from install/positioning_3d_service
        else:
            # Fallback: try to find from current file location
            current_file = Path(__file__).resolve()
            # Look for 'robot_dc' in the path
            for parent in current_file.parents:
                if parent.name == 'robot_dc' or (parent / 'scripts' / 'ThirdParty').exists():
                    workspace_root = parent
                    break
        
        if workspace_root is None:
            # Last resort: assume standard layout from home
            workspace_root = Path.home() / 'Documents' / 'robot_dc'
        
        # Store workspace root for resolving relative paths
        self.workspace_root = workspace_root
        
        self.app_path = workspace_root / 'scripts' / 'ThirdParty' / 'robot_vision' / 'web' / 'positioning_3d' / 'app.py'
        
        if not self.app_path.exists():
            self.get_logger().error(f"app.py not found at: {self.app_path}")
            self.get_logger().error("Please ensure the robot_vision submodule is initialized:")
            self.get_logger().error("  cd scripts/ThirdParty && git submodule update --init --recursive")
            sys.exit(1)
        
        # Working directory for the service
        self.working_dir = self.app_path.parent
        
        # Log configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("3D Positioning Service Configuration:")
        self.get_logger().info(f"  FFPP URL: {self.ffpp_url}")
        self.get_logger().info(f"  Dataset Path: {self.dataset_path}")
        self.get_logger().info(f"  Host: {self.host}")
        self.get_logger().info(f"  Port: {self.port}")
        self.get_logger().info(f"  App Path: {self.app_path}")
        self.get_logger().info(f"  Working Dir: {self.working_dir}")
        self.get_logger().info("=" * 60)
        
        # Register signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Start the web service
        self.start_service()
        
    def start_service(self):
        """Start the positioning_3d web service."""
        try:
            # Resolve dataset path - if relative, make it relative to workspace root
            dataset_path = Path(self.dataset_path)
            if not dataset_path.is_absolute():
                dataset_path = self.workspace_root / dataset_path
            
            # Build command
            cmd = [
                sys.executable,  # Use same Python interpreter
                str(self.app_path),
                '--ffpp-url', self.ffpp_url,
                '--dataset-path', str(dataset_path),
                '--port', str(self.port),
                '--host', self.host
            ]
            
            self.get_logger().info(f"Starting service: {' '.join(cmd)}")
            
            # Start the process
            self.process = subprocess.Popen(
                cmd,
                cwd=str(self.working_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            self.get_logger().info(f"Service started with PID: {self.process.pid}")
            self.get_logger().info(f"Web interface will be available at: http://{self.host}:{self.port}")
            
            # Monitor process output in a separate thread
            import threading
            self.output_thread = threading.Thread(target=self._monitor_output, daemon=True)
            self.output_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Failed to start service: {e}")
            sys.exit(1)
    
    def _monitor_output(self):
        """Monitor and log the web service output."""
        if not self.process or not self.process.stdout:
            return
        
        try:
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    # Forward service logs to ROS logger
                    line = line.rstrip()
                    if 'ERROR' in line or 'Error' in line:
                        self.get_logger().error(f"[positioning_3d] {line}")
                    elif 'WARNING' in line or 'Warning' in line:
                        self.get_logger().warn(f"[positioning_3d] {line}")
                    else:
                        self.get_logger().info(f"[positioning_3d] {line}")
        except Exception as e:
            self.get_logger().error(f"Error monitoring output: {e}")
    
    def _signal_handler(self, signum, frame):
        """Handle termination signals."""
        self.get_logger().info(f"Received signal {signum}, shutting down...")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Clean up the web service process."""
        if self.process:
            try:
                self.get_logger().info("Stopping positioning_3d service...")
                self.process.terminate()
                
                # Wait for graceful shutdown
                try:
                    self.process.wait(timeout=5)
                    self.get_logger().info("Service stopped successfully")
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("Service did not stop gracefully, killing...")
                    self.process.kill()
                    self.process.wait()
                    self.get_logger().info("Service killed")
            except Exception as e:
                self.get_logger().error(f"Error stopping service: {e}")
    
    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = Positioning3DServiceNode()
        
        # Keep the node alive
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
