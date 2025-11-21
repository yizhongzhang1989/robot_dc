#!/usr/bin/env python3
"""
Test Web Service ROS2 Node

A simple Flask-based web service for testing purposes.
"""

import rclpy
from rclpy.node import Node
from flask import Flask, render_template, jsonify
import threading
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class TestWebNode(Node):
    """ROS2 node that manages a simple test web service."""
    
    def __init__(self):
        super().__init__('test_web_node')
        
        # Declare parameters
        self.declare_parameter('port', 8001)
        self.declare_parameter('host', '0.0.0.0')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        
        # Create Flask app
        self.app = self.create_flask_app()
        
        # Log configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("Test Web Service Configuration:")
        self.get_logger().info(f"  Host: {self.host}")
        self.get_logger().info(f"  Port: {self.port}")
        self.get_logger().info("=" * 60)
        
        # Start Flask in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info(f"Web interface available at: http://{self.host}:{self.port}")
        
    def create_flask_app(self):
        """Create and configure Flask application."""
        # Get the template directory
        try:
            # Try to get from installed package
            package_share = get_package_share_directory('test_web')
            template_dir = os.path.join(package_share, 'templates')
        except:
            # Fallback to source directory
            template_dir = os.path.join(os.path.dirname(__file__), 'templates')
        
        self.get_logger().info(f"Template directory: {template_dir}")
        
        app = Flask(__name__, template_folder=template_dir)
        
        @app.route('/')
        def index():
            """Serve the main page."""
            return render_template('index.html')
        
        @app.route('/test', methods=['POST'])
        def test():
            """Handle test button click."""
            self.get_logger().info("Test button clicked!")
            return jsonify({
                'status': 'success',
                'message': 'Test button clicked successfully! ✓',
                'timestamp': self.get_clock().now().to_msg().sec
            })
        
        return app
    
    def run_flask(self):
        """Run Flask server in a separate thread."""
        try:
            # Suppress Flask startup messages (only show errors)
            import logging
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            
            self.app.run(
                host=self.host,
                port=self.port,
                debug=False,
                use_reloader=False
            )
        except Exception as e:
            self.get_logger().error(f"Flask server error: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = TestWebNode()
        
        # Keep the node alive
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
