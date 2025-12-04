#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json, asyncio, threading, os, time
from ament_index_python.packages import get_package_share_directory

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
    from fastapi.responses import FileResponse, JSONResponse
    from fastapi.staticfiles import StaticFiles
    import uvicorn
    FASTAPI = True
except ImportError:
    FASTAPI = False

# Global node instance (accessed by FastAPI routes)
lift_robot_node = None

class LiftRobotWeb(Node):
    def __init__(self):
        super().__init__('lift_robot_web_node')
        self.declare_parameter('port', 8090)
        self.declare_parameter('sensor_topic', '/draw_wire_sensor/data')
        self.port = self.get_parameter('port').value
        self.sensor_topic = self.get_parameter('sensor_topic').value

        # Portable config directory resolution
        env_dir = os.environ.get('LIFT_ROBOT_CONFIG_DIR')
        if env_dir:
            self.config_dir = os.path.abspath(env_dir)
        else:
            try:
                here = os.path.abspath(os.path.dirname(__file__))
                parts = here.split(os.sep)
                if 'colcon_ws' in parts:
                    idx = parts.index('colcon_ws')
                    colcon_ws_path = os.sep.join(parts[:idx+1])
                    self.config_dir = os.path.join(colcon_ws_path, 'config')
                else:
                    self.config_dir = os.path.join(os.getcwd(), 'config')
            except Exception:
                self.config_dir = os.path.join(os.getcwd(), 'config')
        try:
            os.makedirs(self.config_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().warn(f"Cannot create config dir '{self.config_dir}': {e}")

        def get_config_path(name):
            return os.path.join(self.config_dir, name)
        self.get_config_path = get_config_path

        # State holders
        self.latest_raw = None
        self.latest_obj = None
        self.connections = []
        self.loop = None
        
        # Force sensor topic names (will be loaded from config)
        self.force_topic_right = '/force_sensor_right'
        self.force_topic_left = '/force_sensor_left'
        # Force sensor device IDs (will be loaded from config)
        self.force_device_id_right = 52  # Default
        self.force_device_id_left = 53   # Default
        
        # Dual-channel force values: right force sensor (device_id=52) and left force sensor (device_id=53)
        self.right_force_sensor = None  # calibrated
        self.left_force_sensor = None   # calibrated
        self.right_force_raw = None     # raw for calibration
        self.left_force_raw = None      # raw for calibration
        self.right_force_freq_hz = 0.0  # publish frequency
        self.left_force_freq_hz = 0.0   # publish frequency
        self.combined_force_sensor = None  # Combined force (sum of two sensors, falls back to single sensor if one missing)
        self.last_force_update = None  # Latest force sensor update timestamp (either sensor)
        # Force sensor error status
        self.right_force_error = False
        self.right_force_error_msg = None
        self.left_force_error = False
        self.left_force_error_msg = None
        self.platform_status = None
        self.pushrod_status = None

        # Calibration state (draw-wire sensor)
        self.calib_samples = []  # List of {'sensor': float, 'height': float, 'timestamp': float}
        self.calib_scale = None
        self.calib_offset = None
        self.calib_lock = threading.Lock()

        # Platform overshoot calibration state
        self.overshoot_samples_up = []  # List of {'target': float, 'actual': float, 'overshoot': float, 'timestamp': float}
        self.overshoot_samples_down = []
        self.overshoot_up = None
        self.overshoot_down = None
        self.overshoot_lock = threading.Lock()
        
        # Systematic overshoot calibration state (new workflow)
        self.systematic_samples = []  # List of {'height': float, 'overshoot': float, 'direction': 'up'|'down', 'timestamp': float}
        self.safe_range_min = None    # Safe range minimum (actual_min + 50mm)
        self.safe_range_max = None    # Safe range maximum (actual_max - 50mm)
        self.actual_range_min = None  # Detected absolute minimum height
        self.actual_range_max = None  # Detected absolute maximum height

        # Force sensor calibration state (dual-channel)
        self.force_calib_samples_right = []  # device_id=52, List of {'sensor': float, 'force': float, 'timestamp': float}
        self.force_calib_samples_left = []   # device_id=53, List of {'sensor': float, 'force': float, 'timestamp': float}
        self.force_calib_scale_right = None  # Scale for right sensor (force = sensor * scale)
        self.force_calib_scale_left = None   # Scale for left sensor
        self.force_calib_lock = threading.Lock()

        # ROS interfaces
        self.sub = self.create_subscription(String, self.sensor_topic, self.sensor_cb, 10)
        # Force sensor subscriptions (Float32)
        # Load topic names from config
        force_topic_right = '/force_sensor_right'
        force_topic_left = '/force_sensor_left'
        force_device_id_right = 52
        force_device_id_left = 53
        try:
            from common.config_manager import ConfigManager
            config = ConfigManager()
            if config.has('lift_robot.web.force_topics.right'):
                force_topic_right = config.get('lift_robot.web.force_topics.right')
            if config.has('lift_robot.web.force_topics.left'):
                force_topic_left = config.get('lift_robot.web.force_topics.left')
            if config.has('lift_robot.force_sensor_right.device_id'):
                force_device_id_right = config.get('lift_robot.force_sensor_right.device_id')
            if config.has('lift_robot.force_sensor_left.device_id'):
                force_device_id_left = config.get('lift_robot.force_sensor_left.device_id')
        except Exception as e:
            self.get_logger().warn(f"Could not load force sensor topics from config: {e}")
        
        # Store topic names for later use
        self.force_topic_right = force_topic_right
        self.force_topic_left = force_topic_left
        self.force_device_id_right = force_device_id_right
        self.force_device_id_left = force_device_id_left
        
        try:
            # Subscribe to calibrated force sensor topics (for control) - now JSON with freq_hz
            self.force_sub_right = self.create_subscription(String, force_topic_right, self.force_cb_right, 10)
            self.force_sub_left = self.create_subscription(String, force_topic_left, self.force_cb_left, 10)
            # Subscribe to raw force sensor topics (for calibration display) - also JSON
            self.force_raw_sub_right = self.create_subscription(String, f'{force_topic_right}/raw', self.force_raw_cb_right, 10)
            self.force_raw_sub_left = self.create_subscription(String, f'{force_topic_left}/raw', self.force_raw_cb_left, 10)
            self.get_logger().info(f"Subscribed to {force_topic_right} (right) and {force_topic_left} (left)")
            self.get_logger().info(f"Subscribed to {force_topic_right}/raw and {force_topic_left}/raw (for calibration)")
        except Exception as e:
            self.get_logger().warn(f"Failed to create force sensor subscriptions: {e}")
        # Status subscriptions
        self.platform_status_sub = self.create_subscription(String, '/lift_robot_platform/status', self.platform_status_cb, 10)
        
        self.cmd_pub = self.create_publisher(String, '/lift_robot_platform/command', 10)
        
        # Command publishers for different purposes:
        # 1. Action commands (stop, goto_height, force_up, etc.) -> cmd_processor
        self.action_command_pub = self.create_publisher(String, '/lift_robot/command_queue', 10)
        self.get_logger().info("Created action command publisher on /lift_robot/command_queue (for cmd_processor)")
        
        # 2. Direct commands (reset, range_scan) -> lift_robot_node
        self.direct_command_pub = self.create_publisher(String, 'lift_robot_platform/command', 10)
        self.get_logger().info("Created direct command publisher on lift_robot_platform/command (for lift_robot_node)")
        
        # Modbus service client for direct commands (e.g., force sensor tare)
        # Use modbus_manager's service instead of direct /modbus_write topic
        try:
            from modbus_driver_interfaces.srv import ModbusRequest
            self.modbus_client = self.create_client(ModbusRequest, '/modbus_request')
            self.get_logger().info("Created Modbus service client for /modbus_request")
        except Exception as e:
            self.get_logger().warn(f"Failed to create Modbus service client: {e}")

        self.get_logger().info(f"Web server subscribing: {self.sensor_topic}")
        # Note: FastAPI server will be started separately in main()

    def sensor_cb(self, msg: String):
        try:
            self.latest_raw = msg.data
            try:
                self.latest_obj = json.loads(msg.data)
            except Exception:
                self.latest_obj = None
            if self.loop and self.connections:
                # Merge force values and status if available
                outbound = msg.data
                if self.latest_obj is not None:
                    try:
                        merged = dict(self.latest_obj)
                        # Add dual-channel force values (always add, value can be None → JSON null)
                        merged['right_force_sensor'] = self.right_force_sensor
                        merged['left_force_sensor'] = self.left_force_sensor
                        merged['right_force_freq_hz'] = self.right_force_freq_hz
                        merged['left_force_freq_hz'] = self.left_force_freq_hz
                        merged['combined_force_sensor'] = self.combined_force_sensor
                        
                        # Extract draw_wire_sensor error info (from latest_obj which comes from sensor topic)
                        sensor_error = self.latest_obj.get('error', False)
                        sensor_error_msg = self.latest_obj.get('error_message')
                        merged['sensor_error'] = sensor_error
                        merged['sensor_error_message'] = sensor_error_msg
                        
                        # Add force sensor error info
                        merged['right_force_error'] = getattr(self, 'right_force_error', False)
                        merged['right_force_error_message'] = getattr(self, 'right_force_error_msg', None)
                        merged['left_force_error'] = getattr(self, 'left_force_error', False)
                        merged['left_force_error_message'] = getattr(self, 'left_force_error_msg', None)
                        
                        # Force sensor status detection
                        if self.last_force_update is None:
                            # Never received any force data
                            merged['force_sensor_status'] = 'no_data'
                            merged['force_stale'] = True
                        elif (time.time() - self.last_force_update) > 2.0:
                            # No update for >2s (connection lost or sensor failed)
                            merged['force_sensor_status'] = 'stale'
                            merged['force_stale'] = True
                        else:
                            # Normal operation
                            merged['force_sensor_status'] = 'ok'
                            merged['force_stale'] = False
                        
                        if self.platform_status is not None:
                            merged['platform_status'] = self.platform_status
                        # Note: pushrod shares same status as platform
                        outbound = json.dumps(merged)
                    except Exception as e:
                        self.get_logger().warn(f"Sensor data merge error: {e}")
                        outbound = msg.data  # Fallback to raw data
                asyncio.run_coroutine_threadsafe(self.broadcast(outbound), self.loop)
        except Exception as e:
            self.get_logger().error(f"Sensor callback error: {e}")
            # Continue operation

    def force_cb_right(self, msg):
        """Right force sensor callback (device_id=52) - calibrated value with freq_hz"""
        try:
            import math
            # Parse JSON message
            data = json.loads(msg.data)
            value = data.get('force', None)
            freq_hz = data.get('freq_hz', 0.0)
            
            # Extract error info
            has_error = data.get('error', False)
            error_msg = data.get('error_message')
            
            # Store error status
            self.right_force_error = has_error
            self.right_force_error_msg = error_msg
            
            # Check for overflow (inf/nan) - set to None if invalid
            if value is not None and (math.isinf(value) or math.isnan(value)):
                self.get_logger().warn(f"Right force sensor overflow detected: {value}")
                self.right_force_sensor = None
                self.right_force_freq_hz = 0.0
                self.right_force_error = True
                self.right_force_error_msg = "Overflow detected (inf/nan)"
            else:
                self.right_force_sensor = value
                self.right_force_freq_hz = freq_hz
            self.last_force_update = time.time()
            self._update_combined_force()
        except Exception as e:
            self.get_logger().error(f"Right force callback error: {e}")
            self.right_force_sensor = None  # Set to None on exception
            self.right_force_freq_hz = 0.0
            self.right_force_error = True
            self.right_force_error_msg = f"Callback exception: {e}"
            self.last_force_update = time.time()  # Update timestamp to avoid stale detection
    
    def force_cb_left(self, msg):
        """Left force sensor callback (device_id=53) - calibrated value with freq_hz"""
        try:
            import math
            # Parse JSON message
            data = json.loads(msg.data)
            value = data.get('force', None)
            freq_hz = data.get('freq_hz', 0.0)
            
            # Extract error info
            has_error = data.get('error', False)
            error_msg = data.get('error_message')
            
            # Store error status
            self.left_force_error = has_error
            self.left_force_error_msg = error_msg
            
            # Check for overflow (inf/nan) - set to None if invalid
            if value is not None and (math.isinf(value) or math.isnan(value)):
                self.get_logger().warn(f"Left force sensor overflow detected: {value}")
                self.left_force_sensor = None
                self.left_force_freq_hz = 0.0
                self.left_force_error = True
                self.left_force_error_msg = "Overflow detected (inf/nan)"
            else:
                self.left_force_sensor = value
                self.left_force_freq_hz = freq_hz
            self.last_force_update = time.time()
            self._update_combined_force()
        except Exception as e:
            self.get_logger().error(f"Left force callback error: {e}")
            self.left_force_sensor = None  # Set to None on exception
            self.left_force_freq_hz = 0.0
            self.left_force_error = True
            self.left_force_error_msg = f"Callback exception: {e}"
            self.last_force_update = time.time()  # Update timestamp to avoid stale detection
    
    def force_raw_cb_right(self, msg):
        """Right force sensor raw callback (device_id=52) - raw value for calibration"""
        try:
            data = json.loads(msg.data)
            self.right_force_raw = data.get('force', None)
        except Exception as e:
            self.get_logger().warn(f"Right force raw callback error: {e}")
    
    def force_raw_cb_left(self, msg):
        """Left force sensor raw callback (device_id=53) - raw value for calibration"""
        try:
            data = json.loads(msg.data)
            self.left_force_raw = data.get('force', None)
        except Exception as e:
            self.get_logger().warn(f"Left force raw callback error: {e}")

    def _update_combined_force(self):
        """Update combined force: sum if both exist; single value if only one exists; None if neither or overflow"""
        try:
            import math
            if self.right_force_sensor is not None and self.left_force_sensor is not None:
                combined = self.right_force_sensor + self.left_force_sensor
                # Check for overflow (inf/nan) - set to None if invalid
                if math.isinf(combined) or math.isnan(combined):
                    self.get_logger().warn(f"Combined force overflow detected: {combined} (right={self.right_force_sensor}, left={self.left_force_sensor})")
                    self.combined_force_sensor = None
                else:
                    self.combined_force_sensor = combined
            elif self.right_force_sensor is not None:
                self.combined_force_sensor = self.right_force_sensor
            elif self.left_force_sensor is not None:
                self.combined_force_sensor = self.left_force_sensor
            else:
                self.combined_force_sensor = None
        except Exception as e:
            self.get_logger().warn(f"Combined force update error: {e}")
    
    def platform_status_cb(self, msg: String):
        try:
            self.platform_status = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Platform status parse error: {e}")

    async def broadcast(self, text):
        drop = []
        for ws in self.connections:
            try:
                await ws.send_text(text)
            except Exception:
                drop.append(ws)
        for ws in drop:
            if ws in self.connections:
                self.connections.remove(ws)

    # ═══════════════════════════════════════════════════════════════
    # End of LiftRobotWeb Node class
    # ═══════════════════════════════════════════════════════════════


# ═══════════════════════════════════════════════════════════════
# FastAPI Server (runs in separate thread, accesses global node)
# ═══════════════════════════════════════════════════════════════

def run_fastapi_server(port):
    """Run FastAPI server in a separate thread."""
    if not FASTAPI:
        print('FastAPI / uvicorn not installed')
        return

    def run():
        app = FastAPI(title='Lift Robot Web')
        try:
            web_dir = os.path.join(get_package_share_directory('lift_robot_web'), 'web')
        except Exception:
            web_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'web'))
        
        # Serve static assets (images, css, js) from web directory
        try:
            app.mount('/static', StaticFiles(directory=web_dir), name='static')
        except Exception as e:
            lift_robot_node.get_logger().warn(f"Failed to mount static files: {e}")

        @app.get('/')
        def index():
            return FileResponse(os.path.join(web_dir, 'index.html'))

        @app.get('/api/latest')
        def latest():
            """Get latest sensor data with force sensor values merged in real-time"""
            if lift_robot_node.latest_obj is None:
                return JSONResponse({'error': 'no data'}, status_code=404)
            
            try:
                # Merge force sensor data in real-time (same logic as sensor_cb)
                merged = dict(lift_robot_node.latest_obj)
                
                # Add dual-channel force values (always add, value can be None → JSON null)
                merged['right_force_sensor'] = lift_robot_node.right_force_sensor
                merged['left_force_sensor'] = lift_robot_node.left_force_sensor
                merged['right_force_freq_hz'] = lift_robot_node.right_force_freq_hz
                merged['left_force_freq_hz'] = lift_robot_node.left_force_freq_hz
                merged['combined_force_sensor'] = lift_robot_node.combined_force_sensor
                
                # Add error info from draw_wire_sensor
                sensor_error = lift_robot_node.latest_obj.get('error', False)
                sensor_error_msg = lift_robot_node.latest_obj.get('error_message')
                merged['sensor_error'] = sensor_error
                merged['sensor_error_message'] = sensor_error_msg
                
                # Add force sensor error info
                merged['right_force_error'] = getattr(lift_robot_node, 'right_force_error', False)
                merged['right_force_error_message'] = getattr(lift_robot_node, 'right_force_error_msg', None)
                merged['left_force_error'] = getattr(lift_robot_node, 'left_force_error', False)
                merged['left_force_error_message'] = getattr(lift_robot_node, 'left_force_error_msg', None)
                
                # Add platform status (pushrod shares same status)
                if lift_robot_node.platform_status is not None:
                    merged['platform_status'] = lift_robot_node.platform_status
                
                return merged
            except Exception as e:
                lift_robot_node.get_logger().warn(f"Latest data merge error: {e}")
                # Fallback to raw data without force sensors
                return lift_robot_node.latest_obj

        @app.post('/api/cmd')
        async def send_cmd(payload: dict):
            """
            Command endpoint - routes commands to appropriate handlers:
            - Action commands (goto_height, force_up, etc.) -> cmd_processor_node
            - Direct commands (stop, reset, range_scan) -> lift_robot_node
            """
            cmd = payload.get('command')
            target = payload.get('target', 'platform')
            
            # Validate command
            allowed = {
                'up', 'down', 'stop', 'reset',
                'goto_height', 'force_up', 'force_down', 'height_force_hybrid',
                'range_scan_down', 'range_scan_up', 'range_scan_cancel'
            }
            if cmd not in allowed:
                return JSONResponse({'error': 'invalid command'}, status_code=400)
            
            # Route commands to appropriate publisher
            # Direct commands: ONLY reset and range_scan_* -> lift_robot_node (stop uses Action)
            direct_commands = {'reset', 'range_scan_down', 'range_scan_up', 'range_scan_cancel'}
            
            queue_msg = String()
            queue_msg.data = json.dumps(payload)
            
            if cmd in direct_commands:
                # Send to lift_robot_node for immediate handling (reset, range_scan)
                lift_robot_node.direct_command_pub.publish(queue_msg)
                lift_robot_node.get_logger().info(f'[CMD] Direct: {cmd} (target={target})')
                return {
                    'status': 'ok',
                    'command': cmd,
                    'target': target,
                    'method': 'direct'
                }
            else:
                # Send to cmd_processor for Action execution (including stop via StopMovement)
                lift_robot_node.action_command_pub.publish(queue_msg)
                lift_robot_node.get_logger().info(f'[CMD] Action: {cmd} (target={target})')
                return {
                    'status': 'ok',
                    'command': cmd,
                    'target': target,
                    'method': 'action'
                }


        
        @app.get('/api/status')
        def get_status():
            """Get current platform and pushrod task status"""
            response = {}
            if lift_robot_node.platform_status:
                response['platform'] = {
                    'task_state': lift_robot_node.platform_status.get('task_state', 'unknown'),
                    'task_type': lift_robot_node.platform_status.get('task_type'),
                    'task_start_time': lift_robot_node.platform_status.get('task_start_time'),
                    'task_end_time': lift_robot_node.platform_status.get('task_end_time'),
                    'task_duration': lift_robot_node.platform_status.get('task_duration'),
                    'completion_reason': lift_robot_node.platform_status.get('completion_reason'),
                    'control_mode': lift_robot_node.platform_status.get('control_mode'),
                    'movement_state': lift_robot_node.platform_status.get('movement_state'),
                    'current_height': lift_robot_node.platform_status.get('current_height'),
                    'target_height': lift_robot_node.platform_status.get('target_height'),
                    'limit_exceeded': lift_robot_node.platform_status.get('limit_exceeded', False),
                    # Overshoot calibration data
                    'last_goto_target': lift_robot_node.platform_status.get('last_goto_target'),
                    'last_goto_actual': lift_robot_node.platform_status.get('last_goto_actual'),
                    'last_goto_stop_height': lift_robot_node.platform_status.get('last_goto_stop_height'),
                    'last_goto_direction': lift_robot_node.platform_status.get('last_goto_direction'),
                    'last_goto_timestamp': lift_robot_node.platform_status.get('last_goto_timestamp'),
                    # Range scan data
                    'range_scan_active': lift_robot_node.platform_status.get('range_scan_active'),
                    'range_scan_direction': lift_robot_node.platform_status.get('range_scan_direction'),
                    'range_scan_low_height': lift_robot_node.platform_status.get('range_scan_low_height'),
                    'range_scan_high_height': lift_robot_node.platform_status.get('range_scan_high_height'),
                }
            # Pushrod status: shares same task_state/movement_state as platform
            # (both controlled by same lift_robot_node, use same state variables)
            if lift_robot_node.platform_status:
                response['pushrod'] = {
                    'task_state': lift_robot_node.platform_status.get('task_state', 'unknown'),
                    'task_type': lift_robot_node.platform_status.get('task_type'),
                    'task_start_time': lift_robot_node.platform_status.get('task_start_time'),
                    'task_end_time': lift_robot_node.platform_status.get('task_end_time'),
                    'task_duration': lift_robot_node.platform_status.get('task_duration'),
                    'completion_reason': lift_robot_node.platform_status.get('completion_reason'),
                    'control_mode': lift_robot_node.platform_status.get('control_mode'),
                    'movement_state': lift_robot_node.platform_status.get('movement_state'),
                    'current_height': lift_robot_node.platform_status.get('current_height'),
                    'target_height': lift_robot_node.platform_status.get('target_height'),
                }
            if not response:
                return JSONResponse({'error': 'no status data available'}, status_code=503)
            return response
        
        # Calibration API endpoints
        @app.post('/api/calibration/add_sample')
        async def add_calibration_sample(payload: dict):
            """Add calibration sample using latest sensor value"""
            height = payload.get('height')
            if height is None:
                return JSONResponse({'success': False, 'error': 'Missing height field'})
            
            if lift_robot_node.latest_obj is None:
                return JSONResponse({'success': False, 'error': 'No sensor data available'})
            
            # Check for register_1 (full mode) or height (compact mode as fallback)
            sensor_val = None
            if 'register_1' in lift_robot_node.latest_obj:
                sensor_val = lift_robot_node.latest_obj['register_1']
            elif 'height' in lift_robot_node.latest_obj:
                # Compact mode - using calibrated height as "raw" value
                # This is not ideal for calibration but allows functionality
                sensor_val = lift_robot_node.latest_obj['height']
                lift_robot_node.get_logger().warn('Using calibrated height as sensor value - consider disabling publish_compact for proper calibration')
            else:
                return JSONResponse({'success': False, 'error': 'No sensor raw value (register_1) or height field available'})
            
            if sensor_val is None:
                return JSONResponse({'success': False, 'error': 'Sensor value is null'})
            
            with lift_robot_node.calib_lock:
                sample = {
                    'sensor': sensor_val,
                    'height': float(height),
                    'timestamp': time.time()
                }
                lift_robot_node.calib_samples.append(sample)
                total = len(lift_robot_node.calib_samples)
            
            return JSONResponse({
                'success': True,
                'sample': sample,
                'total_samples': total
            })
        
        @app.get('/api/calibration/samples')
        async def get_calibration_samples():
            """Get all calibration samples"""
            with lift_robot_node.calib_lock:
                return JSONResponse({
                    'success': True,
                    'samples': list(lift_robot_node.calib_samples)
                })
        
        @app.delete('/api/calibration/samples/{index}')
        async def delete_calibration_sample(index: int):
            """Delete sample by index"""
            with lift_robot_node.calib_lock:
                if 0 <= index < len(lift_robot_node.calib_samples):
                    removed = lift_robot_node.calib_samples.pop(index)
                    return JSONResponse({
                        'success': True,
                        'removed': removed,
                        'total_samples': len(lift_robot_node.calib_samples)
                    })
                else:
                    return JSONResponse({
                        'success': False,
                        'error': f'Invalid index: {index}'
                    })
        
        @app.delete('/api/calibration/samples')
        async def clear_calibration_samples():
            """Clear all samples"""
            with lift_robot_node.calib_lock:
                count = len(lift_robot_node.calib_samples)
                lift_robot_node.calib_samples.clear()
                lift_robot_node.calib_scale = None
                lift_robot_node.calib_offset = None
            
            return JSONResponse({
                'success': True,
                'cleared': count
            })
        
        @app.post('/api/calibration/calculate')
        async def calculate_calibration():
            """Calculate linear calibration: height = sensor * scale + offset"""
            with lift_robot_node.calib_lock:
                if len(lift_robot_node.calib_samples) < 2:
                    return JSONResponse({
                        'success': False,
                        'error': 'Need at least 2 samples for calibration'
                    })
                
                xs = [s['sensor'] for s in lift_robot_node.calib_samples]
                ys = [s['height'] for s in lift_robot_node.calib_samples]
            
            # Linear regression
            n = len(xs)
            mx = sum(xs) / n
            my = sum(ys) / n
            
            num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
            den = sum((x - mx) ** 2 for x in xs)
            
            if den == 0:
                return JSONResponse({
                    'success': False,
                    'error': 'Zero variance in sensor values'
                })
            
            scale = num / den
            offset = my - scale * mx
            
            # Calculate R²
            y_pred = [scale * x + offset for x in xs]
            ss_tot = sum((y - my) ** 2 for y in ys)
            ss_res = sum((y - yp) ** 2 for y, yp in zip(ys, y_pred))
            r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
            
            # Calculate errors
            errors = [abs(y - yp) for y, yp in zip(ys, y_pred)]
            max_error = max(errors) if errors else 0
            avg_error = sum(errors) / len(errors) if errors else 0
            
            with lift_robot_node.calib_lock:
                lift_robot_node.calib_scale = scale
                lift_robot_node.calib_offset = offset
            
            return JSONResponse({
                'success': True,
                'scale': scale,
                'offset': offset,
                'r_squared': r_squared,
                'max_error_mm': max_error,
                'avg_error_mm': avg_error,
                'num_samples': n,
                'formula': f'height = {scale:.6f} * sensor + {offset:.6f}'
            })
        
        @app.get('/api/calibration/status')
        async def get_calibration_status():
            """Get current calibration status"""
            with lift_robot_node.calib_lock:
                # Get sensor value (prefer register_1, fallback to height)
                sensor_val = None
                if lift_robot_node.latest_obj:
                    if 'register_1' in lift_robot_node.latest_obj:
                        sensor_val = lift_robot_node.latest_obj['register_1']
                    elif 'height' in lift_robot_node.latest_obj:
                        sensor_val = lift_robot_node.latest_obj['height']
                
                # Calculate estimated height if calibrated
                estimated_height = None
                if sensor_val is not None and lift_robot_node.calib_scale is not None and lift_robot_node.calib_offset is not None:
                    estimated_height = sensor_val * lift_robot_node.calib_scale + lift_robot_node.calib_offset
                
                # Load timestamp from config file if exists
                calibrated_at = None
                config_path = lift_robot_node.get_config_path('draw_wire_calibration.json')
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            config_data = json.load(f)
                            calibrated_at = config_data.get('generated_at_iso')
                    except Exception as e:
                        lift_robot_node.get_logger().error(f"Failed to read calibration timestamp: {e}")
                
                return JSONResponse({
                    'num_samples': len(lift_robot_node.calib_samples),
                    'calibrated': lift_robot_node.calib_scale is not None and lift_robot_node.calib_offset is not None,
                    'scale': lift_robot_node.calib_scale,
                    'offset': lift_robot_node.calib_offset,
                    'latest_sensor': sensor_val,
                    'estimated_height': estimated_height,
                    'calibrated_at': calibrated_at
                })
        
        @app.post('/api/calibration/save')
        async def save_calibration():
            """Save calibration to JSON config file"""
            with lift_robot_node.calib_lock:
                if lift_robot_node.calib_scale is None or lift_robot_node.calib_offset is None:
                    return JSONResponse({
                        'success': False,
                        'error': 'No calibration calculated yet'
                    })
                
                scale = lift_robot_node.calib_scale
                offset = lift_robot_node.calib_offset
            
            # Save to colcon_ws/config directory
            config_dir = lift_robot_node.config_dir
            config_path = lift_robot_node.get_config_path('draw_wire_calibration.json')
            
            try:
                # Create config directory if it doesn't exist
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                    lift_robot_node.get_logger().info(f"Created config directory: {config_dir}")
                
                # Prepare calibration data (no samples stored)
                config_data = {
                    'scale': scale,
                    'offset': offset,
                    'enable': True,
                    'formula': f'height = register_1 * {scale} + {offset}',
                    'generated_at': time.time(),
                    'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                }
                
                # Write JSON file
                with open(config_path, 'w') as f:
                    json.dump(config_data, f, indent=2)
                
                lift_robot_node.get_logger().info(f"Calibration saved: scale={scale}, offset={offset} -> {config_path}")
                
                return JSONResponse({
                    'success': True,
                    'filepath': config_path,
                    'scale': scale,
                    'offset': offset,
                    'message': 'Config saved. Restart draw_wire_sensor to apply (no rebuild needed).'
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Failed to save calibration: {e}")
                return JSONResponse({
                    'success': False,
                    'error': str(e)
                })

        # Platform overshoot calibration API endpoints
        @app.post('/api/overshoot/add_sample')
        async def add_overshoot_sample(request: Request):
            """Add a platform overshoot sample"""
            try:
                body = await request.json()
                direction = body.get('direction')  # 'up' or 'down'
                target = body.get('target')
                actual = body.get('actual')
                stop_height = body.get('stop_height')  # Height when stop command issued
                
                if direction not in ['up', 'down']:
                    return JSONResponse({'success': False, 'error': 'Direction must be up or down'})
                if actual is None:
                    return JSONResponse({'success': False, 'error': 'Missing actual height'})
                
                # Calculate RESIDUAL overshoot (drift after stop command)
                # This is what the EMA should learn, NOT the total error from target
                if stop_height is not None:
                    # Correct calculation: residual drift after stop
                    if direction == 'up':
                        overshoot = actual - stop_height  # How much drifted up after stop
                    else:
                        overshoot = stop_height - actual  # How much drifted down after stop
                else:
                    # Fallback: if stop_height not provided, use target (legacy behavior)
                    # This is INCORRECT for EMA learning but maintains backward compatibility
                    overshoot = actual - target if target is not None else 0.0
                    lift_robot_node.get_logger().warn(f"No stop_height provided for {direction} sample - using target (incorrect)")
                
                sample = {
                    'target': float(target) if target is not None else None,
                    'actual': float(actual),
                    'stop_height': float(stop_height) if stop_height is not None else None,
                    'overshoot': overshoot,
                    'timestamp': time.time()
                }
                
                with lift_robot_node.overshoot_lock:
                    if direction == 'up':
                        lift_robot_node.overshoot_samples_up.append(sample)
                    else:
                        lift_robot_node.overshoot_samples_down.append(sample)
                
                # Display normalization: for DOWN direction ensure positive heights in response (do not alter stored raw sample)
                if direction == 'down':
                    sample_display = dict(sample)
                    if sample_display['actual'] is not None:
                        sample_display['actual'] = abs(sample_display['actual'])
                    if sample_display['target'] is not None:
                        sample_display['target'] = abs(sample_display['target'])
                    if sample_display['stop_height'] is not None:
                        sample_display['stop_height'] = abs(sample_display['stop_height'])
                    # Normalize overshoot for DOWN so UI scatter & fit treat it as positive magnitude
                    if sample_display.get('overshoot') is not None:
                        sample_display['overshoot'] = abs(sample_display['overshoot'])
                else:
                    sample_display = sample

                return JSONResponse({
                    'success': True,
                    'sample': sample_display,
                    'direction': direction,
                    'count_up': len(lift_robot_node.overshoot_samples_up),
                    'count_down': len(lift_robot_node.overshoot_samples_down)
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Add overshoot sample error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.get('/api/overshoot/samples')
        async def get_overshoot_samples():
            """Get all overshoot samples"""
            with lift_robot_node.overshoot_lock:
                # Display normalization: ensure DOWN direction heights are positive for plotting
                # We do NOT modify stored raw samples; only transform in response.
                samples_up = lift_robot_node.overshoot_samples_up.copy()
                raw_down = lift_robot_node.overshoot_samples_down.copy()
                samples_down = []
                for s in raw_down:
                    # Clone and normalize height-like fields to positive values
                    nd = dict(s)
                    if 'actual' in nd and nd['actual'] is not None:
                        nd['actual'] = abs(nd['actual'])
                    if 'target' in nd and nd['target'] is not None:
                        nd['target'] = abs(nd['target'])
                    if 'stop_height' in nd and nd['stop_height'] is not None:
                        nd['stop_height'] = abs(nd['stop_height'])
                    if 'overshoot' in nd and nd['overshoot'] is not None:
                        nd['overshoot'] = abs(nd['overshoot'])
                    samples_down.append(nd)
                return JSONResponse({
                    'samples_up': samples_up,
                    'samples_down': samples_down
                })
        
        @app.delete('/api/overshoot/samples/{direction}/{index}')
        async def delete_overshoot_sample(direction: str, index: int):
            """Delete a specific overshoot sample"""
            try:
                if direction not in ['up', 'down']:
                    return JSONResponse({'success': False, 'error': 'Invalid direction'})
                
                with lift_robot_node.overshoot_lock:
                    samples = lift_robot_node.overshoot_samples_up if direction == 'up' else lift_robot_node.overshoot_samples_down
                    if 0 <= index < len(samples):
                        samples.pop(index)
                        return JSONResponse({
                            'success': True,
                            'count_up': len(lift_robot_node.overshoot_samples_up),
                            'count_down': len(lift_robot_node.overshoot_samples_down)
                        })
                    return JSONResponse({'success': False, 'error': 'Index out of range'})
            except Exception as e:
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.delete('/api/overshoot/samples')
        async def clear_overshoot_samples():
            """Clear all overshoot samples (both up and down)"""
            try:
                with lift_robot_node.overshoot_lock:
                    lift_robot_node.overshoot_samples_up.clear()
                    lift_robot_node.overshoot_samples_down.clear()
                return JSONResponse({
                    'success': True,
                    'count_up': len(lift_robot_node.overshoot_samples_up),
                    'count_down': len(lift_robot_node.overshoot_samples_down)
                })
            except Exception as e:
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.post('/api/overshoot/calculate')
        async def calculate_overshoot():
            """Calculate overshoot using EMA (Exponential Moving Average) for fast convergence"""
            try:
                with lift_robot_node.overshoot_lock:
                    samples_up = lift_robot_node.overshoot_samples_up.copy()
                    samples_down = lift_robot_node.overshoot_samples_down.copy()
                
                result = {
                    'success': True,
                    'overshoot_up': None,
                    'overshoot_down': None,
                    'count_up': len(samples_up),
                    'count_down': len(samples_down),
                    'avg_up': None,
                    'avg_down': None,
                    'std_up': None,
                    'std_down': None,
                    'ema_up': None,
                    'ema_down': None
                }
                
                # EMA parameters
                ALPHA = 0.3  # Weight for new samples (0.3 = 30% new, 70% old)
                BOOTSTRAP_COUNT = 3  # Use median of first N samples as EMA seed
                
                # Calculate EMA for upward movement
                if len(samples_up) >= 2:
                    overshoots = [s['overshoot'] for s in samples_up]
                    
                    # Calculate simple average and std for reference
                    avg_up = sum(overshoots) / len(overshoots)
                    std_up = (sum((x - avg_up)**2 for x in overshoots) / len(overshoots)) ** 0.5
                    
                    # EMA calculation with median bootstrap
                    if len(overshoots) <= BOOTSTRAP_COUNT:
                        # Bootstrap phase: use median as initial EMA
                        sorted_vals = sorted(overshoots)
                        ema_up = sorted_vals[len(sorted_vals) // 2]
                    else:
                        # Apply EMA: start with median of first BOOTSTRAP_COUNT samples
                        bootstrap_samples = overshoots[:BOOTSTRAP_COUNT]
                        sorted_bootstrap = sorted(bootstrap_samples)
                        ema_up = sorted_bootstrap[len(sorted_bootstrap) // 2]
                        
                        # Apply EMA to remaining samples
                        for val in overshoots[BOOTSTRAP_COUNT:]:
                            ema_up = (1 - ALPHA) * ema_up + ALPHA * val
                    
                    with lift_robot_node.overshoot_lock:
                        lift_robot_node.overshoot_up = ema_up
                    
                    result['overshoot_up'] = ema_up
                    result['ema_up'] = ema_up
                    result['avg_up'] = avg_up
                    result['std_up'] = std_up
                
                # Calculate EMA for downward movement
                if len(samples_down) >= 2:
                    overshoots = [abs(s['overshoot']) for s in samples_down]  # Use absolute value for down
                    
                    # Calculate simple average and std for reference
                    avg_down = sum(overshoots) / len(overshoots)
                    std_down = (sum((x - avg_down)**2 for x in overshoots) / len(overshoots)) ** 0.5
                    
                    # EMA calculation with median bootstrap
                    if len(overshoots) <= BOOTSTRAP_COUNT:
                        # Bootstrap phase: use median as initial EMA
                        sorted_vals = sorted(overshoots)
                        ema_down = sorted_vals[len(sorted_vals) // 2]
                    else:
                        # Apply EMA: start with median of first BOOTSTRAP_COUNT samples
                        bootstrap_samples = overshoots[:BOOTSTRAP_COUNT]
                        sorted_bootstrap = sorted(bootstrap_samples)
                        ema_down = sorted_bootstrap[len(sorted_bootstrap) // 2]
                        
                        # Apply EMA to remaining samples
                        for val in overshoots[BOOTSTRAP_COUNT:]:
                            ema_down = (1 - ALPHA) * ema_down + ALPHA * val
                    
                    with lift_robot_node.overshoot_lock:
                        lift_robot_node.overshoot_down = ema_down
                    
                    result['overshoot_down'] = ema_down
                    result['ema_down'] = ema_down
                    result['avg_down'] = avg_down
                    result['std_down'] = std_down
                
                return JSONResponse(result)
                
            except Exception as e:
                lift_robot_node.get_logger().error(f"Overshoot calculation error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.get('/api/overshoot/status')
        async def overshoot_status():
            """Get current overshoot calibration status.
            Extended: include region list from config file if present.
            """
            try:
                with lift_robot_node.overshoot_lock:
                    result = {
                        'overshoot_up': lift_robot_node.overshoot_up,
                        'overshoot_down': lift_robot_node.overshoot_down,
                        'count_up': len(lift_robot_node.overshoot_samples_up),
                        'count_down': len(lift_robot_node.overshoot_samples_down),
                        'calibrated_up': lift_robot_node.overshoot_up is not None,
                        'calibrated_down': lift_robot_node.overshoot_down is not None
                    }

                    # Add last goto_height measurement from platform status
                    if lift_robot_node.platform_status:
                        if 'last_goto_target' in lift_robot_node.platform_status:
                            result['last_goto_target'] = lift_robot_node.platform_status['last_goto_target']
                        if 'last_goto_actual' in lift_robot_node.platform_status:
                            result['last_goto_actual'] = lift_robot_node.platform_status['last_goto_actual']
                            result['last_goto_direction'] = lift_robot_node.platform_status.get('last_goto_direction')
                            result['last_goto_timestamp'] = lift_robot_node.platform_status.get('last_goto_timestamp')
                        if 'last_goto_stop_height' in lift_robot_node.platform_status:
                            result['last_goto_stop_height'] = lift_robot_node.platform_status['last_goto_stop_height']

                    # Load timestamp & regions from config file if exists
                    config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config_data = json.load(f)
                                result['calibrated_at'] = config_data.get('generated_at_iso')
                                if 'regions' in config_data:
                                    result['regions'] = config_data['regions']
                                if 'default' in config_data:
                                    # Backward compatible default block
                                    result['default'] = config_data['default']
                        except Exception as e:
                            lift_robot_node.get_logger().error(f"Failed to read overshoot config: {e}")

                    return JSONResponse(result)
            except Exception as e:
                return JSONResponse({'success': False, 'error': str(e)})

        @app.post('/api/overshoot/normalize_file')
        async def normalize_overshoot_file():
            """Normalize stored JSON file: make all DOWN direction overshoot values positive (absolute).

            This rewrites platform_overshoot_calibration.json in-place so plotting tools reading
            directly from the file will see positive Y values. In-memory samples are ALSO updated
            to stay consistent (their overshoot for DOWN set to abs). Heights remain unchanged.
            """
            try:
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                changed = 0
                total_down = 0
                # Load file
                with open(config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                samples = data.get('samples', [])
                for s in samples:
                    if s.get('direction') == 'down':
                        total_down += 1
                        ov = s.get('overshoot')
                        if isinstance(ov, (int, float)) and ov < 0:
                            s['overshoot'] = abs(ov)
                            changed += 1
                # Write back
                data['normalized_at'] = time.time()
                data['normalized_note'] = 'DOWN overshoot values converted to absolute'
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                # Update in-memory lists
                with lift_robot_node.overshoot_lock:
                    for s in lift_robot_node.overshoot_samples_down:
                        ov = s.get('overshoot')
                        if isinstance(ov, (int, float)) and ov < 0:
                            s['overshoot'] = abs(ov)
                return JSONResponse({
                    'success': True,
                    'file': config_path,
                    'total_down_samples': total_down,
                    'changed_count': changed,
                    'message': 'Normalization complete'
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Normalize overshoot file error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.delete('/api/overshoot/config')
        async def clear_overshoot_config():
            """Clear (reset) overshoot calibration config file and in-memory values.
            Removes regions and default values; file is deleted if exists.
            Safe to call before a new full-auto multi-region calibration.
            """
            try:
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                with lift_robot_node.overshoot_lock:
                    lift_robot_node.overshoot_up = None
                    lift_robot_node.overshoot_down = None
                if os.path.exists(config_path):
                    try:
                        os.remove(config_path)
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to delete overshoot config (will overwrite on save): {e}")
                return JSONResponse({'success': True, 'message': 'Overshoot config cleared'})
            except Exception as e:
                return JSONResponse({'success': False, 'error': str(e)})

        @app.post('/api/overshoot/fit')
        async def overshoot_fit(payload: dict = None):
            """Compute polynomial fit (default degree=2 or auto-select) for overshoot_up/down vs height.
            x: region midpoints; y: overshoot values. Saves coefficients to config and
            renders a plot image using OpenCV into web_dir.
            Returns JSON with coeffs and plot URL.
            Supports auto=true to automatically select optimal degree based on R² and AIC.
            """
            try:
                import numpy as np
                import cv2
            except Exception as e:
                return JSONResponse({'success': False, 'error': f'OpenCV/Numpy not available: {e}'})
            try:
                auto_select = payload and payload.get('auto', False)
                degree = 2
                if not auto_select and payload and 'degree' in payload:
                    try:
                        degree = int(payload['degree'])
                        degree = max(1, min(degree, 5))
                    except Exception:
                        degree = 2
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                if not os.path.exists(config_path):
                    return JSONResponse({'success': False, 'error': 'No calibration config found'})
                with open(config_path, 'r') as f:
                    cfg = json.load(f)
                regions = cfg.get('regions') or []
                if not regions:
                    return JSONResponse({'success': False, 'error': 'No regions to fit'})
                # Prepare data
                xs = []
                yu = []
                yd = []
                for r in regions:
                    try:
                        lb = float(r['lower']); ub = float(r['upper'])
                        mid = 0.5*(lb+ub)
                        xs.append(mid)
                        yu.append(float(r.get('overshoot_up', 0.0)))
                        yd.append(float(r.get('overshoot_down', 0.0)))
                    except Exception:
                        continue
                if len(xs) < 2:
                    return JSONResponse({'success': False, 'error': 'Insufficient points for fitting'})
                x = np.array(xs)
                y_up = np.array(yu)
                y_dn = np.array(yd)
                
                # Auto-select optimal degree if requested
                if auto_select:
                    def compute_r2(y_true, y_pred):
                        ss_res = np.sum((y_true - y_pred) ** 2)
                        ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
                        return 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
                    
                    def compute_aic(n, mse, k):
                        # AIC = n*ln(MSE) + 2*k
                        return n * np.log(mse + 1e-10) + 2 * (k + 1)
                    
                    best_degree = 1
                    best_score = -np.inf
                    n = len(xs)
                    
                    for d in range(1, min(6, n)):  # Test degrees 1 to min(5, n-1)
                        try:
                            # Fit both curves (using absolute values)
                            cu_test = np.polyfit(x, np.abs(y_up), d)
                            cd_test = np.polyfit(x, np.abs(y_dn), d)
                            # Evaluate
                            y_up_pred = np.polyval(cu_test, x)
                            y_dn_pred = np.polyval(cd_test, x)
                            # R² for both (compare with absolute values)
                            r2_up = compute_r2(np.abs(y_up), y_up_pred)
                            r2_dn = compute_r2(np.abs(y_dn), y_dn_pred)
                            avg_r2 = (r2_up + r2_dn) / 2
                            # MSE for AIC
                            mse_up = np.mean((np.abs(y_up) - y_up_pred) ** 2)
                            mse_dn = np.mean((np.abs(y_dn) - y_dn_pred) ** 2)
                            avg_mse = (mse_up + mse_dn) / 2
                            # AIC penalty (lower is better)
                            aic = compute_aic(n, avg_mse, d)
                            # Combined score: R² - normalized AIC penalty
                            # Normalize AIC by dividing by n to scale similarly to R²
                            score = avg_r2 - (aic / (n * 10))
                            
                            lift_robot_node.get_logger().info(f"Degree {d}: R²={avg_r2:.4f}, AIC={aic:.2f}, Score={score:.4f}")
                            
                            if score > best_score:
                                best_score = score
                                best_degree = d
                        except Exception as e:
                            lift_robot_node.get_logger().warn(f"Degree {d} fit failed: {e}")
                            continue
                    
                    degree = best_degree
                    lift_robot_node.get_logger().info(f"Auto-selected degree: {degree} (score: {best_score:.4f})")
                
                # Final fit with selected degree
                if len(xs) < (degree + 1):
                    degree = len(xs) - 1
                    lift_robot_node.get_logger().warn(f"Reduced degree to {degree} due to insufficient points")
                
                # CRITICAL: Fit to absolute values to ensure positive overshoot
                cu = np.polyfit(x, np.abs(y_up), degree).tolist()  # highest power first
                cd = np.polyfit(x, np.abs(y_dn), degree).tolist()
                # Save into config
                fit_block = {
                    'type': 'poly',
                    'degree': degree,
                    'coeffs_up': cu,
                    'coeffs_down': cd,
                    'x_min': float(min(xs)),
                    'x_max': float(max(xs)),
                    'generated_at': time.time(),
                    'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                }
                cfg['fit'] = fit_block
                with open(config_path, 'w') as f:
                    json.dump(cfg, f, indent=2)
                # Render plot to web_dir
                try:
                    W, H = 800, 400
                    img = np.ones((H, W, 3), dtype=np.uint8) * 255
                    margin_left = 70
                    margin_right = 40
                    margin_top = 50
                    margin_bottom = 60
                    x_min, x_max = fit_block['x_min'], fit_block['x_max']
                    # y range from points
                    y_min = float(min(min(yu), min(y_dn)))
                    y_max = float(max(max(yu), max(y_dn)))
                    # Padding
                    pad_y = (y_max - y_min) * 0.1 if (y_max>y_min) else 1.0
                    y_min -= pad_y; y_max += pad_y
                    
                    plot_x0 = margin_left
                    plot_x1 = W - margin_right
                    plot_y0 = margin_top
                    plot_y1 = H - margin_bottom
                    
                    def x_to_px(xx):
                        return int(plot_x0 + (xx - x_min) / (x_max - x_min) * (plot_x1 - plot_x0))
                    def y_to_px(yy):
                        return int(plot_y1 - (yy - y_min) / (y_max - y_min) * (plot_y1 - plot_y0))
                    
                    # Draw axes box
                    cv2.rectangle(img, (plot_x0, plot_y0), (plot_x1, plot_y1), (180,180,180), 1)
                    
                    # X-axis ticks and labels (height)
                    num_x_ticks = 6
                    for i in range(num_x_ticks):
                        x_val = x_min + (x_max - x_min) * i / (num_x_ticks - 1)
                        px = x_to_px(x_val)
                        # Tick mark
                        cv2.line(img, (px, plot_y1), (px, plot_y1 + 5), (100,100,100), 1)
                        # Label
                        label = f'{x_val:.0f}'
                        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0]
                        cv2.putText(img, label, (px - text_size[0]//2, plot_y1 + 20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60,60,60), 1, cv2.LINE_AA)
                    
                    # X-axis label
                    cv2.putText(img, 'Height (mm)', (W//2 - 30, H - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40,40,40), 1, cv2.LINE_AA)
                    
                    # Y-axis ticks and labels (overshoot)
                    num_y_ticks = 6
                    for i in range(num_y_ticks):
                        y_val = y_min + (y_max - y_min) * i / (num_y_ticks - 1)
                        py = y_to_px(y_val)
                        # Tick mark
                        cv2.line(img, (plot_x0 - 5, py), (plot_x0, py), (100,100,100), 1)
                        # Label
                        label = f'{y_val:.1f}'
                        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0]
                        cv2.putText(img, label, (plot_x0 - text_size[0] - 8, py + 4), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60,60,60), 1, cv2.LINE_AA)
                    
                    # Y-axis label (rotated text simulation with individual chars)
                    y_label = 'Overshoot (mm)'
                    for idx, ch in enumerate(y_label):
                        cv2.putText(img, ch, (10, plot_y0 + 80 + idx*12), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40,40,40), 1, cv2.LINE_AA)
                    
                    # points
                    for i in range(len(xs)):
                        px = x_to_px(xs[i])
                        pyu = y_to_px(yu[i]); pyd = y_to_px(yd[i])
                        cv2.circle(img, (px, pyu), 3, (0,0,255), -1)
                        cv2.circle(img, (px, pyd), 3, (255,0,0), -1)
                    # fitted curves
                    def eval_poly(coeffs, xx):
                        yy = 0.0
                        deg = len(coeffs)-1
                        for k,c in enumerate(coeffs):
                            powr = deg - k
                            yy += c * (xx ** powr)
                        return yy
                    pts_up = []
                    pts_dn = []
                    for t in np.linspace(x_min, x_max, 200):
                        yu_t = eval_poly(cu, t)
                        yd_t = eval_poly(cd, t)
                        pts_up.append((x_to_px(t), y_to_px(yu_t)))
                        pts_dn.append((x_to_px(t), y_to_px(yd_t)))
                    cv2.polylines(img, [np.array(pts_up, dtype=np.int32)], False, (0,0,255), 2)
                    cv2.polylines(img, [np.array(pts_dn, dtype=np.int32)], False, (255,0,0), 2)
                    # legends
                    cv2.putText(img, 'Up (red) / Down (blue) overshoot vs height', (plot_x0, margin_top-20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80,80,80), 1, cv2.LINE_AA)
                    # Save
                    plot_path = os.path.join(web_dir, 'overshoot_fit.png')
                    cv2.imwrite(plot_path, img)
                    plot_url = '/static/overshoot_fit.png'
                except Exception as e:
                    plot_url = None
                    lift_robot_node.get_logger().warn(f"Plot render error: {e}")
                return JSONResponse({'success': True, 'fit': fit_block, 'plot_url': plot_url})
            except Exception as e:
                lift_robot_node.get_logger().error(f"Overshoot fit error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})

        @app.post('/api/overshoot/save')
        async def save_overshoot(request: Request):
            """Save overshoot calibration to JSON config file.

            Extended: if request body contains lower_bound & upper_bound, store
            region-specific overshoot values. Otherwise update global default.

            New JSON format:
            {
              "enable": true,
              "generated_at": ..., "generated_at_iso": "...",
              "default": {"overshoot_up": X, "overshoot_down": Y},
              "regions": [
                 {"lower": L, "upper": U, "overshoot_up": Xr, "overshoot_down": Yr, "generated_at": ts, "generated_at_iso": "..."}
              ]
            }
            Backward compatibility: existing single-value file will be migrated on first region save.
            """
            try:
                body = {}
                try:
                    body = await request.json()
                except Exception:
                    body = {}
                lower_bound = body.get('lower_bound')
                upper_bound = body.get('upper_bound')
                overwrite_region = body.get('overwrite', False)

                with lift_robot_node.overshoot_lock:
                    overshoot_up = lift_robot_node.overshoot_up
                    overshoot_down = lift_robot_node.overshoot_down

                if overshoot_up is None and overshoot_down is None:
                    return JSONResponse({'success': False, 'error': 'No calibration calculated yet. Please calculate first.'})

                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                config_dir = lift_robot_node.config_dir
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)

                # Load existing config (if any) to preserve regions
                existing = {}
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            existing = json.load(f)
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to read existing overshoot config (will recreate): {e}")
                        existing = {}

                # Migrate legacy format (flat overshoot_up/down at top-level)
                regions = existing.get('regions', [])
                default_block = existing.get('default')
                if default_block is None:
                    # Legacy file uses top-level overshoot_*; capture as default
                    legacy_up = existing.get('overshoot_up')
                    legacy_down = existing.get('overshoot_down')
                    if legacy_up is not None or legacy_down is not None:
                        default_block = {
                            'overshoot_up': legacy_up,
                            'overshoot_down': legacy_down
                        }

                # If region bounds provided, append/update region entry
                region_saved = None
                if lower_bound is not None and upper_bound is not None:
                    try:
                        lb = float(lower_bound)
                        ub = float(upper_bound)
                        if lb >= ub:
                            return JSONResponse({'success': False, 'error': 'lower_bound must be < upper_bound'})
                    except Exception:
                        return JSONResponse({'success': False, 'error': 'Invalid bounds'})

                    # Search existing region that overlaps exactly (same bounds)
                    existing_index = None
                    for i, r in enumerate(regions):
                        if r.get('lower') == lb and r.get('upper') == ub:
                            existing_index = i
                            break
                    region_entry = {
                        'lower': lb,
                        'upper': ub,
                        'overshoot_up': overshoot_up if overshoot_up is not None else 0.0,
                        'overshoot_down': overshoot_down if overshoot_down is not None else 0.0,
                        'generated_at': time.time(),
                        'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                    }
                    if existing_index is not None and overwrite_region:
                        regions[existing_index] = region_entry
                    elif existing_index is None:
                        regions.append(region_entry)
                    region_saved = region_entry

                # Update default block if not present or if no bounds provided
                if default_block is None or (lower_bound is None and upper_bound is None):
                    default_block = {
                        'overshoot_up': overshoot_up if overshoot_up is not None else existing.get('overshoot_up', 0.0),
                        'overshoot_down': overshoot_down if overshoot_down is not None else existing.get('overshoot_down', 0.0)
                    }

                new_config = {
                    'enable': True,
                    'generated_at': time.time(),
                    'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                    'default': default_block,
                    'regions': regions,
                    'format_version': 2
                }

                with open(config_path, 'w') as f:
                    json.dump(new_config, f, indent=2)

                lift_robot_node.get_logger().info(
                    f"Saved overshoot calibration: default(up={default_block['overshoot_up']}, down={default_block['overshoot_down']}), regions={len(regions)}"
                )

                resp = {
                    'success': True,
                    'filepath': config_path,
                    'overshoot_up': overshoot_up,
                    'overshoot_down': overshoot_down,
                    'message': 'Config saved. Restart lift_robot_platform to apply.',
                    'regions_count': len(regions),
                    'region_saved': region_saved
                }
                if region_saved is not None:
                    resp['saved_region_bounds'] = [region_saved['lower'], region_saved['upper']]
                return JSONResponse(resp)
            except Exception as e:
                lift_robot_node.get_logger().error(f"Failed to save overshoot calibration: {e}")
                return JSONResponse({'success': False, 'error': str(e)})

        # ═══════════════════════════════════════════════════════════════
        # Force Sensor Calibration API Endpoints (Dual-Channel)
        # ═══════════════════════════════════════════════════════════════
        @app.post('/api/force_calib/add_sample')
        async def add_force_calib_sample(request: Request):
            """Add a force sensor calibration sample (right or left channel)"""
            try:
                body = await request.json()
                channel = body.get('channel')  # 'right' or 'left'
                actual_force = body.get('force')  # Actual applied force (N)
                
                if channel not in ['right', 'left']:
                    return JSONResponse({'success': False, 'error': 'Channel must be right or left'})
                if actual_force is None:
                    return JSONResponse({'success': False, 'error': 'Missing force value'})
                
                # Get current RAW sensor reading (not calibrated)
                sensor_reading = lift_robot_node.right_force_raw if channel == 'right' else lift_robot_node.left_force_raw
                if sensor_reading is None:
                    return JSONResponse({'success': False, 'error': f'No raw sensor data for {channel} channel'})
                
                sample = {
                    'sensor': float(sensor_reading),
                    'force': float(actual_force),
                    'timestamp': time.time()
                }
                
                with lift_robot_node.force_calib_lock:
                    if channel == 'right':
                        lift_robot_node.force_calib_samples_right.append(sample)
                    else:
                        lift_robot_node.force_calib_samples_left.append(sample)
                
                return JSONResponse({
                    'success': True,
                    'sample': sample,
                    'channel': channel,
                    'count_right': len(lift_robot_node.force_calib_samples_right),
                    'count_left': len(lift_robot_node.force_calib_samples_left)
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Add force calib sample error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.get('/api/force_calib/samples')
        async def get_force_calib_samples():
            """Get all force calibration samples"""
            with lift_robot_node.force_calib_lock:
                return JSONResponse({
                    'samples_right': lift_robot_node.force_calib_samples_right.copy(),
                    'samples_left': lift_robot_node.force_calib_samples_left.copy()
                })
        
        @app.delete('/api/force_calib/samples/{channel}/{index}')
        async def delete_force_calib_sample(channel: str, index: int):
            """Delete a specific force calibration sample"""
            try:
                if channel not in ['right', 'left']:
                    return JSONResponse({'success': False, 'error': 'Invalid channel'})
                
                with lift_robot_node.force_calib_lock:
                    samples = lift_robot_node.force_calib_samples_right if channel == 'right' else lift_robot_node.force_calib_samples_left
                    if 0 <= index < len(samples):
                        removed = samples.pop(index)
                        return JSONResponse({
                            'success': True,
                            'removed': removed,
                            'count_right': len(lift_robot_node.force_calib_samples_right),
                            'count_left': len(lift_robot_node.force_calib_samples_left)
                        })
                    else:
                        return JSONResponse({'success': False, 'error': f'Invalid index: {index}'})
            except Exception as e:
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.delete('/api/force_calib/samples')
        async def clear_force_calib_samples():
            """Clear all force calibration samples"""
            with lift_robot_node.force_calib_lock:
                count_right = len(lift_robot_node.force_calib_samples_right)
                count_left = len(lift_robot_node.force_calib_samples_left)
                lift_robot_node.force_calib_samples_right.clear()
                lift_robot_node.force_calib_samples_left.clear()
                lift_robot_node.force_calib_scale_right = None
                lift_robot_node.force_calib_scale_left = None
            
            return JSONResponse({
                'success': True,
                'cleared_right': count_right,
                'cleared_left': count_left
            })
        
        @app.post('/api/force_calib/calculate')
        async def calculate_force_calibration():
            """Calculate force sensor calibration (zero-intercept linear fit)
            Formula: actual_force = sensor_reading * scale
            """
            try:
                result = {'success': True}
                
                # Calculate right channel
                with lift_robot_node.force_calib_lock:
                    samples_right = lift_robot_node.force_calib_samples_right.copy()
                
                if len(samples_right) >= 1:
                    xs = [s['sensor'] for s in samples_right]
                    ys = [s['force'] for s in samples_right]
                    
                    # Zero-intercept linear fit: y = k*x, k = Σ(x*y) / Σ(x²)
                    numerator = sum(x * y for x, y in zip(xs, ys))
                    denominator = sum(x * x for x in xs)
                    
                    if denominator == 0:
                        result['error_right'] = 'Zero variance in sensor values'
                    else:
                        scale_right = numerator / denominator
                        
                        # Calculate errors
                        errors = [abs(y - x * scale_right) for x, y in zip(xs, ys)]
                        max_error = max(errors) if errors else 0
                        avg_error = sum(errors) / len(errors) if errors else 0
                        
                        with lift_robot_node.force_calib_lock:
                            lift_robot_node.force_calib_scale_right = scale_right
                        
                        result['scale_right'] = scale_right
                        result['max_error_right'] = max_error
                        result['avg_error_right'] = avg_error
                        result['num_samples_right'] = len(samples_right)
                
                # Calculate left channel
                with lift_robot_node.force_calib_lock:
                    samples_left = lift_robot_node.force_calib_samples_left.copy()
                
                if len(samples_left) >= 1:
                    xs = [s['sensor'] for s in samples_left]
                    ys = [s['force'] for s in samples_left]
                    
                    numerator = sum(x * y for x, y in zip(xs, ys))
                    denominator = sum(x * x for x in xs)
                    
                    if denominator == 0:
                        result['error_left'] = 'Zero variance in sensor values'
                    else:
                        scale_left = numerator / denominator
                        
                        errors = [abs(y - x * scale_left) for x, y in zip(xs, ys)]
                        max_error = max(errors) if errors else 0
                        avg_error = sum(errors) / len(errors) if errors else 0
                        
                        with lift_robot_node.force_calib_lock:
                            lift_robot_node.force_calib_scale_left = scale_left
                        
                        result['scale_left'] = scale_left
                        result['max_error_left'] = max_error
                        result['avg_error_left'] = avg_error
                        result['num_samples_left'] = len(samples_left)
                
                if 'scale_right' not in result and 'scale_left' not in result:
                    return JSONResponse({'success': False, 'error': 'Need at least 1 sample per channel'})
                
                return JSONResponse(result)
                
            except Exception as e:
                lift_robot_node.get_logger().error(f"Force calibration calculation error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.get('/api/force_calib/status')
        async def force_calib_status():
            """Get current force calibration status"""
            try:
                with lift_robot_node.force_calib_lock:
                    result = {
                        'scale_right': lift_robot_node.force_calib_scale_right,
                        'scale_left': lift_robot_node.force_calib_scale_left,
                        'count_right': len(lift_robot_node.force_calib_samples_right),
                        'count_left': len(lift_robot_node.force_calib_samples_left),
                        'calibrated_right': lift_robot_node.force_calib_scale_right is not None,
                        'calibrated_left': lift_robot_node.force_calib_scale_left is not None,
                        # Use raw values for calibration display
                        'current_sensor_right': lift_robot_node.right_force_raw,
                        'current_sensor_left': lift_robot_node.left_force_raw
                    }
                    
                    # Load timestamp from config files if exist
                    config_dir = lift_robot_node.config_dir
                    config_path_right = os.path.join(config_dir, 'force_sensor_calibration_right.json')
                    config_path_left = os.path.join(config_dir, 'force_sensor_calibration_left.json')
                    
                    # Load right channel timestamp
                    if os.path.exists(config_path_right):
                        try:
                            with open(config_path_right, 'r') as f:
                                config_data = json.load(f)
                                result['calibrated_at_right'] = config_data.get('generated_at_iso')
                        except Exception as e:
                            lift_robot_node.get_logger().error(f"Failed to read right force timestamp: {e}")
                    
                    # Load left channel timestamp
                    if os.path.exists(config_path_left):
                        try:
                            with open(config_path_left, 'r') as f:
                                config_data = json.load(f)
                                result['calibrated_at_left'] = config_data.get('generated_at_iso')
                        except Exception as e:
                            lift_robot_node.get_logger().error(f"Failed to read left force timestamp: {e}")
                    
                    return JSONResponse(result)
            except Exception as e:
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.post('/api/force_calib/save')
        async def save_force_calibration():
            """Save force sensor calibration to JSON config files"""
            try:
                with lift_robot_node.force_calib_lock:
                    scale_right = lift_robot_node.force_calib_scale_right
                    scale_left = lift_robot_node.force_calib_scale_left
                
                if scale_right is None and scale_left is None:
                    return JSONResponse({
                        'success': False,
                        'error': 'No calibration calculated yet. Please calculate first.'
                    })
                
                config_dir = lift_robot_node.config_dir
                saved_files = []
                
                # Save right channel
                if scale_right is not None:
                    config_path_right = os.path.join(config_dir, 'force_sensor_calibration_right.json')
                    config_data_right = {
                        'device_id': lift_robot_node.force_device_id_right,
                        'topic': lift_robot_node.force_topic_right,
                        'scale': scale_right,
                        'offset': 0.0,  # Zero-intercept
                        'formula': f'actual_force = sensor_reading × {scale_right}',
                        'generated_at': time.time(),
                        'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                    }
                    
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                    
                    with open(config_path_right, 'w') as f:
                        json.dump(config_data_right, f, indent=2)
                    
                    saved_files.append(config_path_right)
                    lift_robot_node.get_logger().info(f"Saved right force calibration: scale={scale_right}")
                
                # Save left channel
                if scale_left is not None:
                    config_path_left = os.path.join(config_dir, 'force_sensor_calibration_left.json')
                    config_data_left = {
                        'device_id': lift_robot_node.force_device_id_left,
                        'topic': lift_robot_node.force_topic_left,
                        'scale': scale_left,
                        'offset': 0.0,
                        'formula': f'actual_force = sensor_reading × {scale_left}',
                        'generated_at': time.time(),
                        'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                    }
                    
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                    
                    with open(config_path_left, 'w') as f:
                        json.dump(config_data_left, f, indent=2)
                    
                    saved_files.append(config_path_left)
                    lift_robot_node.get_logger().info(f"Saved left force calibration: scale={scale_left}")
                
                return JSONResponse({
                    'success': True,
                    'saved_files': saved_files,
                    'scale_right': scale_right,
                    'scale_left': scale_left,
                    'message': 'Config saved. Restart force sensor nodes to apply.'
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Failed to save force calibration: {e}")
                return JSONResponse({
                    'success': False,
                    'error': str(e)
                })

        @app.post('/api/force_calib/tare')
        async def tare_force_sensor(request: Request):
            """
            Tare (zero) force sensor by calling modbus_manager service.
            Fire-and-forget mode: sends command without waiting for response.
            
            Request body:
            {
                "channel": "right" or "left",
                "device_id": 52 or 53
            }
            
            Calls ModbusRequest service with FC06 (Write Single Register) to address 0x0011 with value 0x0001.
            """
            try:
                from modbus_driver_interfaces.srv import ModbusRequest
                
                body = await request.json()
                channel = body.get('channel', 'right')
                
                # Get device_id from config based on channel
                device_id = None
                try:
                    from common.config_manager import ConfigManager
                    config = ConfigManager()
                    config_key = f'lift_robot.force_sensor_{channel}.device_id'
                    if config.has(config_key):
                        device_id = config.get(config_key)
                except Exception:
                    pass
                
                # Fallback to body parameter or error
                if device_id is None:
                    device_id = body.get('device_id')
                    if device_id is None:
                        return JSONResponse({
                            'success': False,
                            'error': f'Could not determine device_id for channel "{channel}"'
                        })
                
                # Check service availability (quick check, no wait)
                if not lift_robot_node.modbus_client.service_is_ready():
                    return JSONResponse({
                        'success': False,
                        'error': 'Modbus service not available'
                    })
                
                # Prepare tare request (FC06, register 0x0011, value 0x0001)
                req = ModbusRequest.Request()
                req.function_code = 0x06  # Write Single Register
                req.slave_id = device_id
                req.address = 0x0011  # Tare command register
                req.count = 1  # Write 1 register
                req.values = [0x0001]  # Trigger tare
                
                lift_robot_node.get_logger().info(
                    f'Sending tare command (fire-and-forget): device_id={device_id} (0x{device_id:02X}), '
                    f'func=0x06, reg=0x0011, value=0x0001'
                )
                
                # Fire-and-forget: call service without waiting for response
                # This avoids blocking and prevents deadlock
                future = lift_robot_node.modbus_client.call_async(req)
                
                # Optional: Add a lightweight callback to log result (non-blocking)
                def log_result(f):
                    try:
                        result = f.result()
                        if result.success:
                            lift_robot_node.get_logger().info(
                                f'Tare command completed: device_id={device_id}, channel={channel}'
                            )
                        else:
                            lift_robot_node.get_logger().warn(
                                f'Tare command failed: device_id={device_id}, error={result.error_message}'
                            )
                    except Exception as e:
                        lift_robot_node.get_logger().error(f'Tare callback error: {e}')
                
                future.add_done_callback(log_result)
                
                # Return immediately without waiting
                return JSONResponse({
                    'success': True,
                    'message': f'Tare command sent to {channel} sensor (device_id={device_id}). Processing in background.',
                    'device_id': device_id,
                    'channel': channel
                })
                    
            except Exception as e:
                lift_robot_node.get_logger().error(f"Tare force sensor error: {e}")
                import traceback
                traceback.print_exc()
                return JSONResponse({
                    'success': False,
                    'error': f'Exception: {str(e)}'
                })

        # ═══════════════════════════════════════════════════════════════
        # Platform Range Detection API (Separate from Overshoot Calibration)
        # ═══════════════════════════════════════════════════════════════
        
        @app.get('/api/range/get')
        async def get_platform_range():
            """Get platform range from config file.
            Returns: {has_range, actual_min, actual_max, safe_min, safe_max, detected_at_iso}
            """
            try:
                config_path = lift_robot_node.get_config_path('platform_range.json')
                
                if not os.path.exists(config_path):
                    # Create default file
                    config_dir = os.path.dirname(config_path)
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                    default_config = {
                        'actual_min': None,
                        'actual_max': None,
                        'safe_min': None,
                        'safe_max': None,
                        'detected_at': None,
                        'detected_at_iso': None
                    }
                    with open(config_path, 'w') as f:
                        json.dump(default_config, f, indent=2)
                    return JSONResponse({
                        'has_range': False,
                        'actual_min': None,
                        'actual_max': None,
                        'safe_min': None,
                        'safe_max': None,
                        'detected_at_iso': None
                    })
                
                # Read existing config
                with open(config_path, 'r') as f:
                    config = json.load(f)
                
                has_range = (config.get('actual_min') is not None and 
                             config.get('actual_max') is not None and
                             config.get('safe_min') is not None and
                             config.get('safe_max') is not None)
                
                return JSONResponse({
                    'has_range': has_range,
                    'actual_min': config.get('actual_min'),
                    'actual_max': config.get('actual_max'),
                    'safe_min': config.get('safe_min'),
                    'safe_max': config.get('safe_max'),
                    'detected_at_iso': config.get('detected_at_iso')
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Get platform range error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.post('/api/range/set')
        async def set_platform_range(payload: dict):
            """Set platform range to config file.
            Expects: {actual_min, actual_max, safe_min, safe_max}
            """
            try:
                actual_min = payload.get('actual_min')
                actual_max = payload.get('actual_max')
                safe_min = payload.get('safe_min')
                safe_max = payload.get('safe_max')
                
                if any(v is None for v in [actual_min, actual_max, safe_min, safe_max]):
                    return JSONResponse({
                        'success': False,
                        'error': 'Missing required fields: actual_min, actual_max, safe_min, safe_max'
                    })
                
                config_path = lift_robot_node.get_config_path('platform_range.json')
                config_dir = lift_robot_node.config_dir
                
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                
                config = {
                    'actual_min': float(actual_min),
                    'actual_max': float(actual_max),
                    'safe_min': float(safe_min),
                    'safe_max': float(safe_max),
                    'detected_at': time.time(),
                    'detected_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                }
                
                with open(config_path, 'w') as f:
                    json.dump(config, f, indent=2)
                
                lift_robot_node.get_logger().info(f"Saved platform range: [{actual_min:.2f}, {actual_max:.2f}]mm, safe: [{safe_min:.2f}, {safe_max:.2f}]mm")
                
                return JSONResponse({
                    'success': True,
                    'actual_min': config['actual_min'],
                    'actual_max': config['actual_max'],
                    'safe_min': config['safe_min'],
                    'safe_max': config['safe_max'],
                    'detected_at_iso': config['detected_at_iso']
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Set platform range error: {e}")
                import traceback
                traceback.print_exc()
                return JSONResponse({'success': False, 'error': str(e)})

        # ═══════════════════════════════════════════════════════════════
        # Systematic Overshoot Calibration API (New Workflow)
        # ═══════════════════════════════════════════════════════════════
        
        @app.post('/api/systematic/detect_safe_range')
        async def detect_safe_range():
            """Detect platform range and compute safe range (min+50mm to max-50mm).
            Returns: {success, actual_min, actual_max, safe_min, safe_max}
            """
            try:
                # This should call the same range detection logic as before
                # But here we just set the ranges - actual detection happens in frontend
                # For now, return structure to be filled by frontend
                return JSONResponse({
                    'success': True,
                    'message': 'Range detection should be called from frontend'
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Detect safe range error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.post('/api/systematic/set_range')
        async def set_safe_range(payload: dict):
            """Set detected range values and save to config immediately.
            Expects: {actual_min, actual_max, safe_min, safe_max}
            """
            try:
                with lift_robot_node.overshoot_lock:
                    lift_robot_node.actual_range_min = payload.get('actual_min')
                    lift_robot_node.actual_range_max = payload.get('actual_max')
                    lift_robot_node.safe_range_min = payload.get('safe_min')
                    lift_robot_node.safe_range_max = payload.get('safe_max')
                
                # Save range to config file immediately
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                config_dir = lift_robot_node.config_dir
                
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                
                # Load existing config or create new
                config = {}
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            config = json.load(f)
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to load existing config: {e}")
                
                # Ensure basic fields exist
                if 'enable' not in config:
                    config['enable'] = True
                if 'format_version' not in config:
                    config['format_version'] = 2
                
                # Update range block (only updates range, preserves other fields)
                config['range'] = {
                    'actual_min': lift_robot_node.actual_range_min,
                    'actual_max': lift_robot_node.actual_range_max,
                    'safe_min': lift_robot_node.safe_range_min,
                    'safe_max': lift_robot_node.safe_range_max,
                    'detected_at': time.time(),
                    'detected_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                }
                
                # Write config
                with open(config_path, 'w') as f:
                    json.dump(config, f, indent=2)
                
                lift_robot_node.get_logger().info(f"Saved range to config: [{lift_robot_node.actual_range_min:.2f}, {lift_robot_node.actual_range_max:.2f}]mm")
                
                return JSONResponse({
                    'success': True,
                    'actual_min': lift_robot_node.actual_range_min,
                    'actual_max': lift_robot_node.actual_range_max,
                    'safe_min': lift_robot_node.safe_range_min,
                    'safe_max': lift_robot_node.safe_range_max
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Set safe range error: {e}")
                import traceback
                traceback.print_exc()
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.get('/api/systematic/range')
        async def get_range_status():
            """Get current range values."""
            with lift_robot_node.overshoot_lock:
                return JSONResponse({
                    'actual_min': lift_robot_node.actual_range_min,
                    'actual_max': lift_robot_node.actual_range_max,
                    'safe_min': lift_robot_node.safe_range_min,
                    'safe_max': lift_robot_node.safe_range_max,
                    'has_range': lift_robot_node.safe_range_min is not None and lift_robot_node.safe_range_max is not None
                })
        
        @app.post('/api/systematic/start_calibration')
        async def start_systematic_calibration():
            """Prepare for calibration by clearing samples in memory and config file."""
            try:
                with lift_robot_node.overshoot_lock:
                    lift_robot_node.systematic_samples.clear()
                
                # Clear samples from config file
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                config_dir = lift_robot_node.config_dir
                
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                
                # Load existing config or create new with defaults
                config = {}
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            config = json.load(f)
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to load existing config: {e}")
                
                # Ensure all required fields exist with proper defaults
                if 'enable' not in config:
                    config['enable'] = True
                if 'format_version' not in config:
                    config['format_version'] = 2
                if 'default' not in config:
                    config['default'] = {
                        'overshoot_up': 2.7999999999999545,
                        'overshoot_down': 3.0139999999999985
                    }
                if 'regions' not in config:
                    config['regions'] = []
                if 'generated_at' not in config:
                    config['generated_at'] = time.time()
                    config['generated_at_iso'] = time.strftime('%Y-%m-%d %H:%M:%S')
                
                # Clear samples array (only updates samples, preserves other fields)
                config['samples'] = []
                
                # Write config
                with open(config_path, 'w') as f:
                    json.dump(config, f, indent=2)
                
                lift_robot_node.get_logger().info("Cleared samples, ready for new calibration")
                
                return JSONResponse({'success': True, 'message': 'Calibration started, samples cleared'})
            except Exception as e:
                lift_robot_node.get_logger().error(f"Start calibration error: {e}")
                import traceback
                traceback.print_exc()
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.post('/api/systematic/add_sample')
        async def add_systematic_sample(payload: dict):
            """Add a systematic calibration sample.
            Expects: {height: float, overshoot: float, direction: 'up'|'down'}
            """
            try:
                height = payload.get('height')
                overshoot = payload.get('overshoot')
                direction = payload.get('direction')
                
                if height is None or overshoot is None or direction is None:
                    return JSONResponse({
                        'success': False,
                        'error': 'Missing required fields: height, overshoot, direction'
                    })
                
                if direction not in ['up', 'down']:
                    return JSONResponse({
                        'success': False,
                        'error': 'Direction must be "up" or "down"'
                    })
                
                sample = {
                    'height': float(height),
                    'overshoot': float(overshoot),
                    'direction': direction,
                    'timestamp': time.time()
                }
                
                with lift_robot_node.overshoot_lock:
                    lift_robot_node.systematic_samples.append(sample)
                    samples_copy = lift_robot_node.systematic_samples.copy()
                
                # Save sample to config file immediately
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                config_dir = lift_robot_node.config_dir
                
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                
                # Load existing config or create new with defaults
                config = {}
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            config = json.load(f)
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to load existing config: {e}")
                
                # Ensure all required fields exist with proper defaults
                if 'enable' not in config:
                    config['enable'] = True
                if 'format_version' not in config:
                    config['format_version'] = 2
                if 'default' not in config:
                    config['default'] = {
                        'overshoot_up': 2.7999999999999545,
                        'overshoot_down': 3.0139999999999985
                    }
                if 'regions' not in config:
                    config['regions'] = []
                if 'generated_at' not in config:
                    config['generated_at'] = time.time()
                    config['generated_at_iso'] = time.strftime('%Y-%m-%d %H:%M:%S')
                
                # Update samples array (only updates samples, preserves other fields)
                config['samples'] = samples_copy
                
                # Write config
                with open(config_path, 'w') as f:
                    json.dump(config, f, indent=2)
                
                lift_robot_node.get_logger().info(
                    f"Added systematic sample: height={height:.2f}mm, "
                    f"overshoot={overshoot:.2f}mm, direction={direction}"
                )
                
                return JSONResponse({
                    'success': True,
                    'sample': sample,
                    'total_count': len(lift_robot_node.systematic_samples)
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Add systematic sample error: {e}")
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.get('/api/systematic/samples')
        async def get_systematic_samples():
            """Get all systematic calibration samples from config file."""
            config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
            
            samples = []
            generated_at = None
            generated_at_iso = None
            fit_generated_at = None
            fit_generated_at_iso = None
            
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r') as f:
                        config = json.load(f)
                        samples = config.get('samples', [])
                        generated_at = config.get('generated_at')
                        generated_at_iso = config.get('generated_at_iso')
                        
                        # Get fit timestamp if available (higher priority)
                        fit = config.get('fit')
                        if fit and isinstance(fit, dict):
                            fit_generated_at = fit.get('generated_at')
                            fit_generated_at_iso = fit.get('generated_at_iso')
                except Exception as e:
                    lift_robot_node.get_logger().warn(f"Failed to load samples from config: {e}")
            
            # Separate by direction for convenience
            samples_up = [s for s in samples if s.get('direction') == 'up']
            samples_down = [s for s in samples if s.get('direction') == 'down']
            
            return JSONResponse({
                'samples_all': samples,
                'samples_up': samples_up,
                'samples_down': samples_down,
                'count_total': len(samples),
                'count_up': len(samples_up),
                'count_down': len(samples_down),
                'generated_at': generated_at,
                'generated_at_iso': generated_at_iso,
                'fit_generated_at': fit_generated_at,
                'fit_generated_at_iso': fit_generated_at_iso
            })
        
        @app.delete('/api/systematic/samples/{index}')
        async def delete_systematic_sample(index: int):
            """Delete a specific systematic calibration sample by index."""
            try:
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                
                # Load samples from config file
                if not os.path.exists(config_path):
                    return JSONResponse({
                        'success': False,
                        'error': 'No calibration data found'
                    })
                
                try:
                    with open(config_path, 'r') as f:
                        config = json.load(f)
                except Exception as e:
                    return JSONResponse({
                        'success': False,
                        'error': f'Failed to load config: {e}'
                    })
                
                samples = config.get('samples', [])
                
                # Validate index
                if not (0 <= index < len(samples)):
                    return JSONResponse({
                        'success': False,
                        'error': f'Invalid index: {index} (total samples in file: {len(samples)})'
                    })
                
                # Delete sample
                deleted_sample = samples.pop(index)
                
                # Also update memory if applicable
                with lift_robot_node.overshoot_lock:
                    if 0 <= index < len(lift_robot_node.systematic_samples):
                        lift_robot_node.systematic_samples.pop(index)
                
                # Ensure all required fields exist
                if 'enable' not in config:
                    config['enable'] = True
                if 'format_version' not in config:
                    config['format_version'] = 2
                if 'default' not in config:
                    config['default'] = {
                        'overshoot_up': 2.7999999999999545,
                        'overshoot_down': 3.0139999999999985
                    }
                if 'regions' not in config:
                    config['regions'] = []
                if 'generated_at' not in config:
                    config['generated_at'] = time.time()
                    config['generated_at_iso'] = time.strftime('%Y-%m-%d %H:%M:%S')
                
                # Update samples array
                config['samples'] = samples
                
                # Write config
                with open(config_path, 'w') as f:
                    json.dump(config, f, indent=2)
                
                lift_robot_node.get_logger().info(f"Deleted sample #{index}: {deleted_sample}")
                
                return JSONResponse({
                    'success': True,
                    'deleted_sample': deleted_sample,
                    'remaining_count': len(samples)
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Delete systematic sample error: {e}")
                import traceback
                traceback.print_exc()
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.delete('/api/systematic/samples')
        async def clear_systematic_samples():
            """Clear all systematic calibration samples from memory and config file."""
            with lift_robot_node.overshoot_lock:
                count = len(lift_robot_node.systematic_samples)
                lift_robot_node.systematic_samples.clear()
            
            # Clear samples from config file
            config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r') as f:
                        config = json.load(f)
                    
                    # Ensure all required fields exist with proper defaults before clearing
                    if 'enable' not in config:
                        config['enable'] = True
                    if 'format_version' not in config:
                        config['format_version'] = 2
                    if 'default' not in config:
                        config['default'] = {
                            'overshoot_up': 2.7999999999999545,
                            'overshoot_down': 3.0139999999999985
                        }
                    if 'regions' not in config:
                        config['regions'] = []
                    if 'generated_at' not in config:
                        config['generated_at'] = time.time()
                        config['generated_at_iso'] = time.strftime('%Y-%m-%d %H:%M:%S')
                    
                    # Clear samples array (preserves all other fields)
                    config['samples'] = []
                    
                    with open(config_path, 'w') as f:
                        json.dump(config, f, indent=2)
                    
                    lift_robot_node.get_logger().info(f"Cleared {count} systematic samples from config")
                except Exception as e:
                    lift_robot_node.get_logger().warn(f"Failed to clear samples from config: {e}")
            
            return JSONResponse({
                'success': True,
                'cleared_count': count
            })
        
        @app.post('/api/systematic/plot')
        async def plot_systematic_data(payload: dict = None):
            """Generate scatter plot of systematic calibration data with optional polynomial fit.
            Payload: {degree: int (optional, 1-6)}
            Returns: {success, plot_url, fit_coeffs_up, fit_coeffs_down}
            """
            try:
                import numpy as np
                import cv2
            except Exception as e:
                return JSONResponse({'success': False, 'error': f'OpenCV/Numpy not available: {e}'})
            
            try:
                degree = None
                if payload and 'degree' in payload:
                    try:
                        degree = int(payload['degree'])
                        degree = max(1, min(degree, 6))
                    except Exception:
                        degree = None
                
                # Load samples from config file
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                samples = []
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            config = json.load(f)
                            samples = config.get('samples', [])
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to load samples from config: {e}")
                
                if not samples:
                    return JSONResponse({'success': False, 'error': 'No samples to plot'})
                
                # Separate samples by direction
                samples_up = [s for s in samples if s['direction'] == 'up']
                samples_down = [s for s in samples if s['direction'] == 'down']
                
                if not samples_up and not samples_down:
                    return JSONResponse({'success': False, 'error': 'No valid samples'})
                
                # Extract data arrays
                heights_up = np.array([s['height'] for s in samples_up])
                overshoots_up = np.array([abs(s['overshoot']) for s in samples_up])  # ensure positive
                heights_down = np.array([s['height'] for s in samples_down])
                overshoots_down = np.array([abs(s['overshoot']) for s in samples_down])  # ensure positive
                
                # Determine plot ranges
                all_heights = np.concatenate([heights_up, heights_down]) if len(heights_up) > 0 and len(heights_down) > 0 else (heights_up if len(heights_up) > 0 else heights_down)
                all_overshoots = np.concatenate([overshoots_up, overshoots_down]) if len(overshoots_up) > 0 and len(overshoots_down) > 0 else (overshoots_up if len(overshoots_up) > 0 else overshoots_down)
                
                x_min = float(np.min(all_heights))
                x_max = float(np.max(all_heights))
                y_min = float(np.min(all_overshoots))
                y_max = float(np.max(all_overshoots))
                # If all positive, anchor y_min at 0 for clearer axis
                if y_min > 0:
                    y_min = 0.0
                
                # Add margins
                x_range = x_max - x_min
                y_range = y_max - y_min
                x_min -= x_range * 0.1
                x_max += x_range * 0.1
                y_min -= y_range * 0.1
                y_max += y_range * 0.1
                
                # Create plot image
                img_width = 1000
                img_height = 700
                margin_left = 70
                margin_right = 30
                margin_top = 50
                margin_bottom = 60
                
                plot_width = img_width - margin_left - margin_right
                plot_height = img_height - margin_top - margin_bottom
                
                img = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255
                
                # Helper function to map data coordinates to pixel coordinates
                def to_pixel(h, o):
                    px = int(margin_left + (h - x_min) / (x_max - x_min) * plot_width)
                    py = int(margin_top + plot_height - (o - y_min) / (y_max - y_min) * plot_height)
                    return (px, py)
                
                # Draw axes
                plot_x0 = margin_left
                plot_x1 = margin_left + plot_width
                plot_y0 = margin_top
                plot_y1 = margin_top + plot_height
                
                cv2.line(img, (plot_x0, plot_y1), (plot_x1, plot_y1), (0, 0, 0), 2)  # X-axis
                cv2.line(img, (plot_x0, plot_y0), (plot_x0, plot_y1), (0, 0, 0), 2)  # Y-axis
                
                # Draw X-axis ticks and labels
                num_x_ticks = 6
                for i in range(num_x_ticks):
                    val = x_min + (x_max - x_min) * i / (num_x_ticks - 1)
                    px = int(margin_left + (val - x_min) / (x_max - x_min) * plot_width)
                    cv2.line(img, (px, plot_y1), (px, plot_y1 + 5), (0, 0, 0), 1)
                    label = f'{val:.1f}'
                    cv2.putText(img, label, (px - 20, plot_y1 + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                
                # Draw Y-axis ticks and labels
                num_y_ticks = 6
                for i in range(num_y_ticks):
                    val = y_min + (y_max - y_min) * i / (num_y_ticks - 1)
                    py = int(margin_top + plot_height - (val - y_min) / (y_max - y_min) * plot_height)
                    cv2.line(img, (plot_x0 - 5, py), (plot_x0, py), (0, 0, 0), 1)
                    label = f'{val:.2f}'
                    cv2.putText(img, label, (plot_x0 - 60, py + 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                
                # Axis labels
                cv2.putText(img, 'Height (mm)', (img_width // 2 - 50, img_height - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                
                # Y-axis label (vertical text simulated with multiple characters)
                y_label = 'Overshoot (mm)'
                for idx, char in enumerate(y_label):
                    cv2.putText(img, char, (10, margin_top + 20 + idx * 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                
                # Plot scatter points
                for h, o in zip(heights_up, overshoots_up):
                    px, py = to_pixel(h, o)
                    cv2.circle(img, (px, py), 4, (0, 0, 255), -1)  # Red for UP
                
                for h, o in zip(heights_down, overshoots_down):
                    px, py = to_pixel(h, o)
                    cv2.circle(img, (px, py), 4, (255, 0, 0), -1)  # Blue for DOWN
                
                # Polynomial fit if degree specified
                fit_coeffs_up = None
                fit_coeffs_down = None
                
                if degree is not None:
                    # Fit UP samples
                    if len(heights_up) >= degree + 1:
                        # CRITICAL: Fit to absolute values to ensure positive overshoot
                        coeffs_up = np.polyfit(heights_up, np.abs(overshoots_up), degree)
                        fit_coeffs_up = coeffs_up.tolist()
                        
                        # Draw fitted curve
                        h_curve = np.linspace(x_min, x_max, 200)
                        o_curve = np.polyval(coeffs_up, h_curve)
                        
                        points = []
                        for h, o in zip(h_curve, o_curve):
                            if y_min <= o <= y_max:  # Only draw within range
                                points.append(to_pixel(h, o))
                        
                        if len(points) > 1:
                            points = np.array(points, dtype=np.int32)
                            cv2.polylines(img, [points], False, (200, 0, 0), 2)  # Dark red for UP fit
                    
                    # Fit DOWN samples
                    if len(heights_down) >= degree + 1:
                        # CRITICAL: Fit to absolute values to ensure positive overshoot
                        coeffs_down = np.polyfit(heights_down, np.abs(overshoots_down), degree)
                        fit_coeffs_down = coeffs_down.tolist()
                        
                        # Draw fitted curve
                        h_curve = np.linspace(x_min, x_max, 200)
                        o_curve = np.polyval(coeffs_down, h_curve)
                        
                        points = []
                        for h, o in zip(h_curve, o_curve):
                            if y_min <= o <= y_max:  # Only draw within range
                                points.append(to_pixel(h, o))
                        
                        if len(points) > 1:
                            points = np.array(points, dtype=np.int32)
                            cv2.polylines(img, [points], False, (150, 0, 0), 2)  # Dark blue for DOWN fit
                
                # Title and legend
                title = 'Systematic Overshoot Calibration'
                if degree is not None:
                    title += f' (Degree {degree} fit)'
                cv2.putText(img, title, (margin_left, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
                
                # Legend
                legend_x = plot_x1 - 200
                legend_y = plot_y0 + 20
                cv2.circle(img, (legend_x, legend_y), 4, (0, 0, 255), -1)
                cv2.putText(img, 'UP samples', (legend_x + 10, legend_y + 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                
                cv2.circle(img, (legend_x, legend_y + 20), 4, (255, 0, 0), -1)
                cv2.putText(img, 'DOWN samples', (legend_x + 10, legend_y + 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                
                if degree is not None:
                    cv2.line(img, (legend_x, legend_y + 40), (legend_x + 30, legend_y + 40), (200, 0, 0), 2)
                    cv2.putText(img, 'UP fit', (legend_x + 35, legend_y + 45), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                    
                    cv2.line(img, (legend_x, legend_y + 55), (legend_x + 30, legend_y + 55), (150, 0, 0), 2)
                    cv2.putText(img, 'DOWN fit', (legend_x + 35, legend_y + 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
                
                # Save plot
                try:
                    web_dir = os.path.join(get_package_share_directory('lift_robot_web'), 'web')
                except Exception:
                    web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
                
                plot_path = os.path.join(web_dir, 'systematic_overshoot_plot.png')
                cv2.imwrite(plot_path, img)
                plot_url = '/static/systematic_overshoot_plot.png'
                
                return JSONResponse({
                    'success': True,
                    'plot_url': plot_url,
                    'fit_coeffs_up': fit_coeffs_up,
                    'fit_coeffs_down': fit_coeffs_down,
                    'degree': degree,
                    'sample_count_up': len(heights_up),
                    'sample_count_down': len(heights_down),
                    'x_range': [x_min, x_max],
                    'y_range': [y_min, y_max]
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Plot systematic data error: {e}")
                import traceback
                traceback.print_exc()
                return JSONResponse({'success': False, 'error': str(e)})
        
        @app.post('/api/systematic/save_fit')
        async def save_systematic_fit(payload: dict):
            """Save polynomial fit coefficients to config file.
            Payload: {degree: int, coeffs_up: list, coeffs_down: list}
            """
            try:
                degree = payload.get('degree')
                coeffs_up = payload.get('coeffs_up')
                coeffs_down = payload.get('coeffs_down')
                
                if degree is None or coeffs_up is None or coeffs_down is None:
                    return JSONResponse({
                        'success': False,
                        'error': 'Missing required fields: degree, coeffs_up, coeffs_down'
                    })
                
                # Load samples from config file
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                samples = []
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            config = json.load(f)
                            samples = config.get('samples', [])
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to load samples from config: {e}")
                
                if not samples:
                    return JSONResponse({'success': False, 'error': 'No samples available'})
                
                heights = [s['height'] for s in samples]
                x_min = min(heights)
                x_max = max(heights)
                
                # Create fit block
                fit_block = {
                    'type': 'poly',
                    'degree': int(degree),
                    'coeffs_up': coeffs_up,
                    'coeffs_down': coeffs_down,
                    'x_min': x_min,
                    'x_max': x_max,
                    'generated_at': time.time(),
                    'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                }
                
                # Create range block with detected range information
                range_block = None
                with lift_robot_node.overshoot_lock:
                    if lift_robot_node.actual_range_min is not None and lift_robot_node.actual_range_max is not None:
                        range_block = {
                            'actual_min': lift_robot_node.actual_range_min,
                            'actual_max': lift_robot_node.actual_range_max,
                            'safe_min': lift_robot_node.safe_range_min,
                            'safe_max': lift_robot_node.safe_range_max,
                            'detected_at': time.time(),
                            'detected_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                        }
                
                # Save to config file
                config_path = lift_robot_node.get_config_path('platform_overshoot_calibration.json')
                config_dir = lift_robot_node.config_dir
                
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                
                # Load existing config or create new
                config = {
                    'enable': True,
                    'generated_at': time.time(),
                    'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'format_version': 2
                }
                
                if os.path.exists(config_path):
                    try:
                        with open(config_path, 'r') as f:
                            existing = json.load(f)
                            # Preserve existing fields
                            config.update(existing)
                    except Exception as e:
                        lift_robot_node.get_logger().warn(f"Failed to load existing config: {e}")
                
                # Ensure regions field exists
                if 'regions' not in config:
                    config['regions'] = []
                
                # Update fit block and range block
                config['fit'] = fit_block
                if range_block is not None:
                    config['range'] = range_block
                
                # Remove 'default' when fit is saved (fit takes priority)
                # This forces the system to use fitted polynomial instead of default values
                if 'default' in config:
                    lift_robot_node.get_logger().info("Removing 'default' from config - using fitted polynomial")
                    del config['default']
                
                # Write config
                with open(config_path, 'w') as f:
                    json.dump(config, f, indent=2)
                
                lift_robot_node.get_logger().info(f"Saved systematic fit: degree={degree}, range=[{x_min:.2f}, {x_max:.2f}]mm")
                
                return JSONResponse({
                    'success': True,
                    'config_path': config_path,
                    'fit': fit_block
                })
            except Exception as e:
                lift_robot_node.get_logger().error(f"Save systematic fit error: {e}")
                import traceback
                traceback.print_exc()
                return JSONResponse({'success': False, 'error': str(e)})

        # ═══════════════════════════════════════════════════════════════
        # ROS2 Action API Endpoints (Testing Action vs Topic)
        # ═══════════════════════════════════════════════════════════════
        @app.websocket('/ws')
        async def ws_endpoint(ws: WebSocket):
            await ws.accept()
            lift_robot_node.connections.append(ws)
            if lift_robot_node.latest_raw:
                await ws.send_text(lift_robot_node.latest_raw)
            try:
                while True:
                    try:
                        _ = await asyncio.wait_for(ws.receive_text(), timeout=30)
                    except asyncio.TimeoutError:
                        await ws.send_text('ping')
            except WebSocketDisconnect:
                pass
            finally:
                if ws in lift_robot_node.connections:
                    lift_robot_node.connections.remove(ws)

        app.mount('/static', StaticFiles(directory=web_dir), name='static')

        # Create and own event loop explicitly; run uvicorn server inside
        lift_robot_node.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(lift_robot_node.loop)

        # Use 'warning' log level to suppress access logs (INFO level logs every request)
        config = uvicorn.Config(app, host='0.0.0.0', port=port, loop='asyncio', log_level='warning')
        server = uvicorn.Server(config)

        async def serve():
            try:
                lift_robot_node.get_logger().info(f"Web server started on 0.0.0.0:{port}")
                await server.serve()
            except Exception as e:
                lift_robot_node.get_logger().error(f"Web server serve error: {e}")

        lift_robot_node.loop.create_task(serve())
        try:
            lift_robot_node.loop.run_forever()
        except Exception as e:
            lift_robot_node.get_logger().error(f"Web server loop error: {e}")
        finally:
            try:
                lift_robot_node.loop.close()
            except Exception as e:
                lift_robot_node.get_logger().error(f"Web server loop close error: {e}")

    threading.Thread(target=run, daemon=True).start()


def main(args=None):
    global lift_robot_node
    
    if not FASTAPI:
        print('Install fastapi uvicorn first: pip install fastapi uvicorn')
        return
    
    rclpy.init(args=args)
    lift_robot_node = LiftRobotWeb()
    
    # Start FastAPI server in separate thread
    port = lift_robot_node.port
    lift_robot_node.get_logger().info(f'Starting FastAPI server on port {port}')
    fastapi_thread = threading.Thread(target=run_fastapi_server, args=(port,), daemon=True)
    fastapi_thread.start()
    
    # Use MultiThreadedExecutor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(lift_robot_node)
    
    try:
        lift_robot_node.get_logger().info('Spinning node with MultiThreadedExecutor')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        lift_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
