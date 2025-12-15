#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string, send_from_directory, jsonify
import os
import threading
import time
from ament_index_python.packages import get_package_share_directory

# Import RTDE for robot status
try:
    from rtde import RTDE
    RTDE_AVAILABLE = True
except ImportError as e:
    print(f"Warning: RTDE library not available: {e}")
    RTDE_AVAILABLE = False

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        
        self.get_logger().info('Task Manager Node starting...')
        
        # Task statistics
        self.active_tasks = 0
        self.pending_tasks = 0
        self.completed_tasks = 0
        self.system_status = 'Ready'
        self.robot_connection = 'Disconnected'
        
        # RTDE connection for robot_mode and safety_mode
        self.rtde_connection = None
        self.rtde_connected = False
        self.rtde_lock = threading.Lock()
        self.rtde_thread_running = True  # Set to True first so reader thread can start
        self.cached_robot_mode = None
        self.cached_safety_mode = None
        self.robot_ip = '192.168.1.15'  # Default UR15 IP
        
        # Get package share directory for web files
        try:
            self.package_share_directory = get_package_share_directory('task_manager')
            self.web_dir = os.path.join(self.package_share_directory, 'web')
            self.get_logger().info(f'Web directory: {self.web_dir}')
        except Exception as e:
            self.get_logger().error(f'Failed to get package directory: {e}')
            self.web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
        
        # Create Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Initialize RTDE connection
        self._init_rtde_connection()
        
        # Start Flask in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info('Task Manager Node started successfully!')
        self.get_logger().info('Open http://localhost:8040 in your browser')
    
    def setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            """Serve the main HTML page"""
            try:
                html_path = os.path.join(self.web_dir, 'index.html')
                with open(html_path, 'r', encoding='utf-8') as f:
                    html_content = f.read()
                return render_template_string(html_content)
            except Exception as e:
                self.get_logger().error(f'Error serving index.html: {e}')
                return f"Error: {e}", 500
        
        @self.app.route('/tailwind.js')
        def serve_tailwind():
            """Serve tailwind.js"""
            return send_from_directory(self.web_dir, 'tailwind.js')
        
        @self.app.route('/static/<path:filename>')
        def serve_static(filename):
            """Serve static files"""
            return send_from_directory(self.web_dir, filename)
        
        @self.app.route('/js/<path:filename>')
        def serve_js(filename):
            """Serve JavaScript files"""
            js_dir = os.path.join(self.web_dir, 'js')
            return send_from_directory(js_dir, filename)
        
        @self.app.route('/api/status')
        def get_status():
            """API endpoint to get current system status"""
            return jsonify({
                'system_status': self.system_status,
                'active_tasks': self.active_tasks,
                'pending_tasks': self.pending_tasks,
                'completed_tasks': self.completed_tasks,
                'robot_connection': self.robot_connection
            })
        
        @self.app.route('/api/robot_status')
        def get_robot_status():
            """API endpoint to get robot_mode and safety_mode"""
            robot_mode = None
            robot_mode_str = "-"
            safety_mode = None
            safety_mode_str = "-"
            rtde_connected = False
            
            with self.rtde_lock:
                rtde_connected = self.rtde_connected
                robot_mode = self.cached_robot_mode
                safety_mode = self.cached_safety_mode
            
            if robot_mode is not None:
                robot_mode_str = self._get_robot_mode_string(robot_mode)
            
            if safety_mode is not None:
                safety_mode_str = self._get_safety_mode_string(safety_mode)
            
            return jsonify({
                'success': True,
                'rtde_connected': rtde_connected,
                'robot_mode': robot_mode,
                'robot_mode_str': robot_mode_str,
                'safety_mode': safety_mode,
                'safety_mode_str': safety_mode_str
            })
    
    def _init_rtde_connection(self):
        """Initialize RTDE connection for reading robot_mode and safety_mode"""
        if not RTDE_AVAILABLE:
            self.get_logger().warning("RTDE library not available, robot/safety mode will not be available")
            self.cached_robot_mode = -1
            self.cached_safety_mode = -1
            return
        
        # Run connection in background to not block startup
        connection_thread = threading.Thread(target=self._connect_rtde, daemon=True)
        connection_thread.start()
    
    def _connect_rtde(self):
        """Background thread to connect to RTDE"""
        try:
            self.get_logger().info(f"Connecting to RTDE interface at {self.robot_ip}:30004...")
            self.rtde_connection = RTDE(self.robot_ip, 30004)
            self.rtde_connection.connect()
            
            # Get controller version
            version = self.rtde_connection.get_controller_version()
            self.get_logger().info(f"RTDE connected! Controller version: {version[0]}.{version[1]}.{version[2]}.{version[3]}")
            
            # Setup output recipe
            variables = ['robot_mode', 'safety_mode']
            if not self.rtde_connection.send_output_setup(variables, frequency=125):
                self.get_logger().error("Failed to configure RTDE output recipe")
                self.rtde_connection.disconnect()
                self.rtde_connection = None
                return
            
            # Start data synchronization
            if not self.rtde_connection.send_start():
                self.get_logger().error("Failed to start RTDE data synchronization")
                self.rtde_connection.disconnect()
                self.rtde_connection = None
                return
            
            self.get_logger().info("RTDE configured successfully")
            
            # Set connected flag and start background thread
            with self.rtde_lock:
                self.rtde_connected = True
            
            self.rtde_thread = threading.Thread(target=self._rtde_reader_thread, daemon=True)
            self.rtde_thread.start()
            
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize RTDE connection: {e}")
            self.rtde_connection = None
            self.rtde_connected = False
    
    def _rtde_reader_thread(self):
        """Background thread to read robot_mode and safety_mode from RTDE"""
        self.get_logger().info("RTDE reader thread started")
        
        while self.rtde_thread_running:
            if not self.rtde_connected or not self.rtde_connection:
                time.sleep(0.05)
                continue
            
            try:
                # Call receive() outside the lock to avoid blocking
                state = self.rtde_connection.receive()
                
                if state is not None:
                    # Only lock when updating cache
                    with self.rtde_lock:
                        if hasattr(state, 'robot_mode'):
                            self.cached_robot_mode = state.robot_mode
                        if hasattr(state, 'safety_mode'):
                            self.cached_safety_mode = state.safety_mode
                
            except Exception as e:
                self.get_logger().warning(f"Error reading from RTDE: {e}")
                with self.rtde_lock:
                    self.rtde_connected = False
                time.sleep(0.5)
        
        self.get_logger().info("RTDE reader thread stopped")
    
    def _get_robot_mode_string(self, mode):
        """Convert robot_mode numeric value to string"""
        robot_modes = {
            -1: "NO_CONTROLLER",
            0: "DISCONNECTED",
            1: "CONFIRM_SAFETY",
            2: "BOOTING",
            3: "POWER_OFF",
            4: "POWER_ON",
            5: "IDLE",
            6: "BACKDRIVE",
            7: "RUNNING",
            8: "UPDATING_FIRMWARE"
        }
        return robot_modes.get(mode, f"Unknown({mode})")
    
    def _get_safety_mode_string(self, mode):
        """Convert safety_mode numeric value to string"""
        safety_modes = {
            1: "Normal",
            2: "Reduced",
            3: "Protective Stop",
            4: "Recovery",
            5: "Safeguard Stop",
            6: "System Emergency Stop",
            7: "Robot Emergency Stop",
            8: "Emergency Stop",
            9: "Violation",
            10: "Fault",
            11: "Validate Stop"
        }
        return safety_modes.get(mode, f"Unknown({mode})")
    
    def run_flask(self):
        """Run Flask server"""
        try:
            import logging
            # Only log errors and warnings, not normal requests
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.WARNING)  # Only show WARNING and ERROR
            
            self.get_logger().info('Starting Flask server on http://0.0.0.0:8040')
            # Suppress Flask development server banner but keep errors
            import sys
            cli = sys.modules['flask.cli']
            cli.show_server_banner = lambda *x: None
            
            self.app.run(host='0.0.0.0', port=8040, debug=False, use_reloader=False)
        except Exception as e:
            self.get_logger().error(f'Flask server error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TaskManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
