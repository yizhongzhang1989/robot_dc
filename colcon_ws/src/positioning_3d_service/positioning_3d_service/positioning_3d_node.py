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
from pathlib import Path
from common.workspace_utils import get_workspace_root


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
        
        # Get workspace root using common utilities
        workspace_root = get_workspace_root()
        if workspace_root is None:
            self.get_logger().error("Could not determine workspace root directory")
            self.get_logger().error("Please ensure you're running from within the robot_dc workspace")
            sys.exit(1)
        
        # Store workspace root as Path object
        self.workspace_root = Path(workspace_root)
        
        self.app_path = self.workspace_root / 'scripts' / 'ThirdParty' / 'robot_vision' / 'web' / 'positioning_3d' / 'app.py'
        
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
        
        # Start the web service
        self.start_service()
        
    def start_service(self):
        """Start the positioning_3d web service."""
        try:
            # Resolve dataset path - if relative, make it relative to workspace root
            dataset_path = Path(self.dataset_path)
            if not dataset_path.is_absolute():
                dataset_path = Path(self.workspace_root) / dataset_path
            
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
            
            # Start the process (create new process group for proper signal handling)
            self.process = subprocess.Popen(
                cmd,
                cwd=str(self.working_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                preexec_fn=os.setsid  # Create new session to handle signals properly
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
        """Monitor and log the web service output (critical messages only)."""
        if not self.process or not self.process.stdout:
            return
        
        try:
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    line = line.rstrip()
                    
                    # Only log critical messages (errors and warnings)
                    # Filter out routine HTTP request logs from werkzeug
                    if 'ERROR' in line or 'Error' in line or 'error' in line:
                        self.get_logger().error(f"[positioning_3d] {line}")
                    elif 'WARNING' in line or 'Warning' in line or 'warning' in line:
                        self.get_logger().warn(f"[positioning_3d] {line}")
                    elif 'werkzeug' in line.lower():
                        # Skip routine werkzeug HTTP request logs (GET, POST, etc.)
                        continue
                    elif 'INFO' in line and ('GET' in line or 'POST' in line or 'PUT' in line or 'DELETE' in line):
                        # Skip HTTP method logs
                        continue
                    elif line.strip():
                        # Log other non-empty lines at debug level (only shown if ROS log level is debug)
                        self.get_logger().debug(f"[positioning_3d] {line}")
        except Exception as e:
            self.get_logger().error(f"Error monitoring output: {e}")
    
    def cleanup(self):
        """Clean up the web service process."""
        if self.process and self.process.poll() is None:
            try:
                self.get_logger().info("Stopping positioning_3d service...")
                
                # Send SIGTERM to the entire process group
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    # Process already terminated
                    return
                
                # Wait for graceful shutdown with shorter timeout
                try:
                    self.process.wait(timeout=2)
                    self.get_logger().info("Service stopped successfully")
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("Service did not stop gracefully, killing...")
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
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
    
    def signal_handler(signum, frame):
        """Handle termination signals."""
        if node:
            node.get_logger().info(f"Received signal {signum}, shutting down...")
            node.cleanup()
        sys.exit(0)
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
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
