#!/usr/bin/env python3
"""
Image Labeling Service ROS2 Node

This node wraps the image labeling web service and manages its lifecycle.
It launches the Flask web application for labeling images with keypoints.
"""

import rclpy
from rclpy.node import Node
import subprocess
import signal
import sys
import os
from pathlib import Path
from common.workspace_utils import get_workspace_root


class ImageLabelingServiceNode(Node):
    """ROS2 node that manages the image labeling web service."""
    
    def __init__(self):
        super().__init__('image_labeling_service_node')
        
        # Declare parameters
        self.declare_parameter('port', 8002)
        self.declare_parameter('host', '0.0.0.0')
        
        # Get parameters
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
        
        self.app_path = self.workspace_root / 'scripts' / 'ThirdParty' / 'robot_vision' / 'ThirdParty' / 'ImageLabelingWeb' / 'launch_server.py'
        
        if not self.app_path.exists():
            self.get_logger().error(f"launch_server.py not found at: {self.app_path}")
            self.get_logger().error("Please ensure the robot_vision submodule is initialized:")
            self.get_logger().error("  cd scripts/ThirdParty && git submodule update --init --recursive")
            sys.exit(1)
        
        # Working directory for the service
        self.working_dir = self.app_path.parent
        
        # Log configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("Image Labeling Service Configuration:")
        self.get_logger().info(f"  Host: {self.host}")
        self.get_logger().info(f"  Port: {self.port}")
        self.get_logger().info(f"  App Path: {self.app_path}")
        self.get_logger().info(f"  Working Dir: {self.working_dir}")
        self.get_logger().info("=" * 60)
        
        # Start the web service
        self.start_service()
        
    def start_service(self):
        """Start the image labeling web service."""
        try:
            # Build command
            cmd = [
                sys.executable,  # Use same Python interpreter
                str(self.app_path),
                '--port', str(self.port),
                '--host', self.host,
                '--no-browser'  # Don't open browser automatically in ROS context
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
                    # Filter out routine HTTP request logs from werkzeug/Flask
                    if 'ERROR' in line or 'Error' in line or 'error' in line:
                        self.get_logger().error(f"[image_labeling] {line}")
                    elif 'WARNING' in line or 'Warning' in line or 'warning' in line:
                        self.get_logger().warn(f"[image_labeling] {line}")
                    elif 'werkzeug' in line.lower() or 'flask' in line.lower():
                        # Skip routine werkzeug/Flask HTTP request logs (GET, POST, etc.)
                        continue
                    elif 'INFO' in line and ('GET' in line or 'POST' in line or 'PUT' in line or 'DELETE' in line):
                        # Skip HTTP method logs
                        continue
                    elif 'Server running at' in line or 'Press Ctrl+C' in line or '=' * 10 in line:
                        # Log startup messages
                        self.get_logger().info(f"[image_labeling] {line}")
                    elif line.strip():
                        # Log other non-empty lines at debug level
                        self.get_logger().debug(f"[image_labeling] {line}")
        except Exception as e:
            self.get_logger().error(f"Error monitoring output: {e}")
    
    def cleanup(self):
        """Clean up the web service process."""
        if self.process and self.process.poll() is None:
            try:
                self.get_logger().info("Stopping image_labeling service...")
                
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
    try:
        node = ImageLabelingServiceNode()
        
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
