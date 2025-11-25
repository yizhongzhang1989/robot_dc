#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
        # Dual-channel force values: right force sensor (device_id=52) and left force sensor (device_id=53)
        self.right_force_sensor = None  # /force_sensor (device_id=52)
        self.left_force_sensor = None   # /force_sensor_2 (device_id=53)
        self.combined_force_sensor = None  # Combined force (sum of two sensors, falls back to single sensor if one missing)
        self.last_force_update = None  # Latest force sensor update timestamp (either sensor)
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
        try:
            from std_msgs.msg import Float32
            # Subscribe to two force sensor topics
            self.force_sub_right = self.create_subscription(Float32, '/force_sensor', self.force_cb_right, 10)
            self.force_sub_left = self.create_subscription(Float32, '/force_sensor_2', self.force_cb_left, 10)
            self.get_logger().info("Subscribed to /force_sensor (right) and /force_sensor_2 (left)")
        except Exception as e:
            self.get_logger().warn(f"Failed to create force sensor subscriptions: {e}")
        # Status subscriptions
        self.platform_status_sub = self.create_subscription(String, '/lift_robot_platform/status', self.platform_status_cb, 10)
        
        self.cmd_pub = self.create_publisher(String, '/lift_robot_platform/command', 10)

        # ═══════════════════════════════════════════════════════════════
        # ROS2 Action Clients: Unified Platform+Pushrod (target parameter)
        # ═══════════════════════════════════════════════════════════════
        self.action_clients = {}  # Dict of action_name -> ActionClient
        self.action_goal_handles = {}  # Dict of action_name -> goal_handle
        self.action_feedback = {}  # Dict of action_name -> feedback
        self.action_result = {}  # Dict of action_name -> result
        self.action_status = {}  # Dict of action_name -> status
        self.action_lock = threading.Lock()
        
        try:
            from rclpy.action import ActionClient
            from lift_robot_interfaces.action import GotoHeight, ForceControl, HybridControl, ManualMove
            
            # Create Action Clients (4 types - all support target parameter)
            # Platform and Pushrod share the same action servers with target='platform'/'pushrod'
            self.action_clients['goto_height'] = ActionClient(self, GotoHeight, '/lift_robot/goto_height')
            self.action_clients['force_control'] = ActionClient(self, ForceControl, '/lift_robot/force_control')
            self.action_clients['hybrid_control'] = ActionClient(self, HybridControl, '/lift_robot/hybrid_control')
            self.action_clients['manual_move'] = ActionClient(self, ManualMove, '/lift_robot/manual_move')
            
            # Initialize status for all actions
            for action_name in self.action_clients.keys():
                self.action_status[action_name] = 'idle'
                self.action_goal_handles[action_name] = None
                self.action_feedback[action_name] = None
                self.action_result[action_name] = None
            
            self.get_logger().info("[Action] Platform+Pushrod unified Action Clients created - waiting for servers...")
            
            # Wait for action servers (non-blocking, 2s timeout each)
            for action_name, client in self.action_clients.items():
                if client.wait_for_server(timeout_sec=2.0):
                    self.get_logger().info(f"[Action] ✅ {action_name} Server available")
                else:
                    self.get_logger().warn(f"[Action] ⚠️ {action_name} Server not available yet")
                    
        except ImportError as e:
            self.get_logger().warn(f"[Action] Cannot import actions - interface not built or installed: {e}")
        except Exception as e:
            self.get_logger().error(f"[Action] Failed to create Action Clients: {e}")

        self.get_logger().info(f"Web server subscribing: {self.sensor_topic}")
        self.start_server()

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
                        # Add dual-channel force values
                        if self.right_force_sensor is not None:
                            merged['right_force_sensor'] = self.right_force_sensor
                        if self.left_force_sensor is not None:
                            merged['left_force_sensor'] = self.left_force_sensor
                        if self.combined_force_sensor is not None:
                            merged['combined_force_sensor'] = self.combined_force_sensor
                        # Force sensor stale detection (no update for >2s)
                        if self.last_force_update is not None:
                            if (time.time() - self.last_force_update) > 2.0:
                                merged['force_stale'] = True
                        if self.platform_status is not None:
                            merged['platform_status'] = self.platform_status
                        if self.pushrod_status is not None:
                            merged['pushrod_status'] = self.pushrod_status
                        outbound = json.dumps(merged)
                    except Exception as e:
                        self.get_logger().warn(f"Sensor data merge error: {e}")
                        outbound = msg.data  # Fallback to raw data
                asyncio.run_coroutine_threadsafe(self.broadcast(outbound), self.loop)
        except Exception as e:
            self.get_logger().error(f"Sensor callback error: {e}")
            # Continue operation

    def force_cb_right(self, msg):
        """Right force sensor callback (device_id=52, /force_sensor)"""
        try:
            if 0 <= msg.data <= 2000:
                self.right_force_sensor = msg.data
                self.last_force_update = time.time()
                self._update_combined_force()
            else:
                self.get_logger().warn(f"Right force out of range: {msg.data}")
        except Exception as e:
            self.get_logger().warn(f"Right force callback error: {e}")
    
    def force_cb_left(self, msg):
        """Left force sensor callback (device_id=53, /force_sensor_2)"""
        try:
            if 0 <= msg.data <= 2000:
                self.left_force_sensor = msg.data
                self.last_force_update = time.time()
                self._update_combined_force()
            else:
                self.get_logger().warn(f"Left force out of range: {msg.data}")
        except Exception as e:
            self.get_logger().warn(f"Left force callback error: {e}")

    def _update_combined_force(self):
        """Update combined force: sum if both exist; single value if only one exists; None if neither; prevent inf overflow"""
        try:
            if self.right_force_sensor is not None and self.left_force_sensor is not None:
                combined = self.right_force_sensor + self.left_force_sensor
                # Prevent overflow to inf
                if combined > 4000 or combined < -1000:
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
    
    # ═══════════════════════════════════════════════════════════════
    # Action Client Callback Methods (Generic for all 4 Actions)
    # ═══════════════════════════════════════════════════════════════
    
    def _create_goal_response_callback(self, action_name):
        """Create goal response callback for specific action"""
        def callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f'[Action:{action_name}] ❌ Goal rejected')
                with self.action_lock:
                    self.action_status[action_name] = 'aborted'
                return
            
            self.get_logger().info(f'[Action:{action_name}] ✅ Goal accepted, executing...')
            with self.action_lock:
                self.action_goal_handles[action_name] = goal_handle
                self.action_status[action_name] = 'executing'
                self.get_logger().debug(f'[Action:{action_name}] Stored goal_handle id={id(goal_handle)}')
            
            # Register result callback
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self._create_result_callback(action_name, goal_handle))
        return callback
    
    def _create_feedback_callback(self, action_name):
        """Create feedback callback for specific action"""
        def callback(feedback_msg):
            feedback = feedback_msg.feedback
            with self.action_lock:
                # Store all feedback fields (different Actions have different fields)
                self.action_feedback[action_name] = feedback
        return callback
    
    def _create_result_callback(self, action_name, goal_handle):
        """Create result callback for specific action"""
        def callback(future):
            result = future.result().result
            status = future.result().status
            
            with self.action_lock:
                # Only update status if this is still the active goal_handle
                # This prevents old Action results from overwriting new Action status
                if self.action_goal_handles.get(action_name) is goal_handle:
                    self.action_result[action_name] = result
                    
                    # Map action status codes to string
                    if status == 4:  # SUCCEEDED
                        self.action_status[action_name] = 'succeeded'
                        self.get_logger().info(f'[Action:{action_name}] ✅ SUCCEEDED - reason={result.completion_reason}')
                    elif status == 6:  # ABORTED
                        self.action_status[action_name] = 'aborted'
                        self.get_logger().warn(f'[Action:{action_name}] ⚠️ ABORTED - reason={result.completion_reason}')
                    elif status == 5:  # CANCELED
                        self.action_status[action_name] = 'cancelled'
                        self.get_logger().info(f'[Action:{action_name}] ⏹️ CANCELLED - reason={result.completion_reason}')
                    else:
                        self.action_status[action_name] = 'unknown'
                        self.get_logger().warn(f'[Action:{action_name}] ❓ Unknown status: {status}')
                else:
                    # Old Action result - ignore to prevent overwriting new Action status
                    self.get_logger().debug(
                        f'[Action:{action_name}] Ignoring old result (handle_id={id(goal_handle)}, '
                        f'current_handle_id={id(self.action_goal_handles.get(action_name))})'
                    )
        return callback
    
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

    def start_server(self):
        if not FASTAPI:
            self.get_logger().error('FastAPI / uvicorn not installed')
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
                self.get_logger().warn(f"Failed to mount static files: {e}")

            @app.get('/')
            def index():
                return FileResponse(os.path.join(web_dir, 'index.html'))

            @app.get('/api/latest')
            def latest():
                if self.latest_obj:
                    return self.latest_obj
                return JSONResponse({'error': 'no data'}, status_code=404)

            @app.post('/api/cmd')
            async def send_cmd(payload: dict):
                cmd = payload.get('command')
                target = payload.get('target','platform')
                duration = payload.get('duration')
                allowed = {'up','down','stop','timed_up','timed_down','stop_timed','goto_point','goto_height','force_up','force_down','height_force_hybrid','reset','range_scan_down','range_scan_up','range_scan_cancel'}
                if cmd not in allowed:
                    return JSONResponse({'error':'invalid command'}, status_code=400)
                
                # ═══════════════════════════════════════════════════════════════
                # PLATFORM COMMANDS: Route to Actions (pure Action architecture)
                # ═══════════════════════════════════════════════════════════════
                if target == 'platform':
                    # Platform now uses Action-only control (no topic commands)
                    
                    # --- ManualMove Action: up/down/stop/reset ---
                    if cmd in ('up', 'down', 'stop', 'reset'):
                        if cmd == 'stop':
                            # Cancel all running actions
                            self.get_logger().info('[CMD] STOP requested')
                            cancelled_any = False
                            with self.action_lock:
                                self.get_logger().info(f'[CMD] Current goal_handles: {list(self.action_goal_handles.keys())}')
                                self.get_logger().info(f'[CMD] Current statuses: {self.action_status}')
                                for action_name, goal_handle in self.action_goal_handles.items():
                                    if goal_handle and self.action_status.get(action_name) == 'executing':
                                        self.get_logger().info(f'[CMD] Cancelling {action_name} action (handle_id={id(goal_handle)})...')
                                        goal_handle.cancel_goal_async()
                                        cancelled_any = True
                                    else:
                                        self.get_logger().info(f'[CMD] Skip {action_name}: handle={goal_handle is not None}, status={self.action_status.get(action_name)}')
                            self.get_logger().info(f'[CMD] STOP complete: cancelled_any={cancelled_any}')
                            return {'status':'ok','command':'stop','cancelled':cancelled_any}
                        
                        elif cmd == 'reset':
                            # Emergency reset: cancel all Actions + send reset topic
                            self.get_logger().warn('[CMD] 🔴 EMERGENCY RESET requested')
                            cancelled_count = 0
                            with self.action_lock:
                                for action_name, goal_handle in self.action_goal_handles.items():
                                    if goal_handle and self.action_status.get(action_name) == 'executing':
                                        self.get_logger().info(f'[CMD] Cancelling {action_name} for reset...')
                                        goal_handle.cancel_goal_async()
                                        cancelled_count += 1
                            
                            # Send reset topic for hardware cleanup
                            reset_msg = String()
                            reset_msg.data = json.dumps({'command': 'reset'})
                            self.cmd_pub.publish(reset_msg)
                            
                            # Don't manually set platform_status - let ROS topic update it
                            # The lift_robot_node will publish emergency_reset state via /lift_robot_platform/status
                            
                            self.get_logger().warn(f'[CMD] Reset complete: cancelled {cancelled_count} Actions, waiting for status update...')
                            return {'status':'ok','command':'reset','cancelled':cancelled_count}
                        
                        else:  # up or down
                            # Create ManualMove goal
                            from lift_robot_interfaces.action import ManualMove
                            goal_msg = ManualMove.Goal()
                            goal_msg.target = 'platform'
                            goal_msg.direction = cmd
                            
                            # Send action goal
                            send_goal_future = self.action_clients['manual_move'].send_goal_async(
                                goal_msg,
                                feedback_callback=self._create_feedback_callback('manual_move')
                            )
                            send_goal_future.add_done_callback(
                                self._create_goal_response_callback('manual_move')
                            )
                            
                            with self.action_lock:
                                self.action_status['manual_move'] = 'sending'
                            
                            return {'status':'ok','command':cmd,'action':'manual_move','action_status':'sending'}
                    
                    # --- GotoHeight Action ---
                    elif cmd == 'goto_height':
                        target_height = payload.get('target_height')
                        mode = payload.get('mode', 'absolute')  # Default to 'absolute' if not specified
                        
                        if target_height is None:
                            return JSONResponse({'error':'target_height required for goto_height'}, status_code=400)
                        
                        try:
                            target_height = float(target_height)
                        except:
                            return JSONResponse({'error':'invalid target_height'}, status_code=400)
                        
                        # Validate mode
                        if mode not in ['absolute', 'relative']:
                            return JSONResponse({'error':'mode must be "absolute" or "relative"'}, status_code=400)
                        
                        from lift_robot_interfaces.action import GotoHeight
                        goal_msg = GotoHeight.Goal()
                        goal_msg.target = 'platform'
                        goal_msg.target_height = target_height
                        goal_msg.mode = mode
                        
                        send_goal_future = self.action_clients['goto_height'].send_goal_async(
                            goal_msg,
                            feedback_callback=self._create_feedback_callback('goto_height')
                        )
                        send_goal_future.add_done_callback(
                            self._create_goal_response_callback('goto_height')
                        )
                        
                        with self.action_lock:
                            self.action_status['goto_height'] = 'sending'
                        
                        return {'status':'ok','command':cmd,'action':'goto_height','target_height':target_height,'mode':mode,'action_status':'sending'}
                    
                    # --- ForceControl Action: force_up/force_down ---
                    elif cmd in ('force_up', 'force_down'):
                        target_force = payload.get('target_force')
                        if target_force is None:
                            return JSONResponse({'error':'target_force required for force control'}, status_code=400)
                        
                        try:
                            target_force = float(target_force)
                            if target_force <= 0:
                                return JSONResponse({'error':'target_force must be > 0'}, status_code=400)
                        except:
                            return JSONResponse({'error':'invalid target_force'}, status_code=400)
                        
                        from lift_robot_interfaces.action import ForceControl
                        goal_msg = ForceControl.Goal()
                        goal_msg.target_force = target_force
                        goal_msg.direction = cmd.replace('force_', '')  # 'force_up' -> 'up'
                        
                        send_goal_future = self.action_clients['force_control'].send_goal_async(
                            goal_msg,
                            feedback_callback=self._create_feedback_callback('force_control')
                        )
                        send_goal_future.add_done_callback(
                            self._create_goal_response_callback('force_control')
                        )
                        
                        with self.action_lock:
                            self.action_status['force_control'] = 'sending'
                        
                        return {'status':'ok','command':cmd,'action':'force_control','target_force':target_force,'action_status':'sending'}
                    
                    # --- HybridControl Action ---
                    elif cmd == 'height_force_hybrid':
                        target_height = payload.get('target_height')
                        target_force = payload.get('target_force')
                        
                        if target_height is None or target_force is None:
                            return JSONResponse({'error':'height_force_hybrid requires both target_height and target_force'}, status_code=400)
                        
                        try:
                            target_height = float(target_height)
                            target_force = float(target_force)
                            if target_force <= 0:
                                return JSONResponse({'error':'target_force must be > 0'}, status_code=400)
                        except:
                            return JSONResponse({'error':'invalid target_height or target_force'}, status_code=400)
                        
                        # Direction is automatically determined by Action server based on height_error
                        from lift_robot_interfaces.action import HybridControl
                        goal_msg = HybridControl.Goal()
                        goal_msg.target_height = target_height
                        goal_msg.target_force = target_force
                        
                        send_goal_future = self.action_clients['hybrid_control'].send_goal_async(
                            goal_msg,
                            feedback_callback=self._create_feedback_callback('hybrid_control')
                        )
                        send_goal_future.add_done_callback(
                            self._create_goal_response_callback('hybrid_control')
                        )
                        
                        with self.action_lock:
                            self.action_status['hybrid_control'] = 'sending'
                        
                        return {'status':'ok','command':cmd,'action':'hybrid_control','target_height':target_height,'target_force':target_force,'action_status':'sending'}
                    
                    # --- Range Scan Commands (Topic-based) ---
                    elif cmd in ('range_scan_down', 'range_scan_up', 'range_scan_cancel'):
                        # Range scan uses topic commands (not Actions)
                        msg = String()
                        msg.data = json.dumps({'command': cmd})
                        self.cmd_pub.publish(msg)
                        self.get_logger().info(f'[CMD] Range scan command sent: {cmd}')
                        return {'status':'ok','command':cmd,'method':'topic'}
                    
                    else:
                        return JSONResponse({'error':f'unknown platform command: {cmd}'}, status_code=400)                # ═══════════════════════════════════════════════════════════════
                # PUSHROD COMMANDS: Use Action architecture (same as Platform)
                # ═══════════════════════════════════════════════════════════════
                elif target == 'pushrod':
                    # --- ManualMove Action: up/down/stop ---
                    if cmd in ('up', 'down'):
                        # Create ManualMove goal (unified platform action with target='pushrod')
                        from lift_robot_interfaces.action import ManualMove
                        goal_msg = ManualMove.Goal()
                        goal_msg.target = 'pushrod'
                        goal_msg.direction = cmd
                        
                        # Send action goal (use unified manual_move action)
                        send_goal_future = self.action_clients['manual_move'].send_goal_async(
                            goal_msg,
                            feedback_callback=self._create_feedback_callback('manual_move')
                        )
                        send_goal_future.add_done_callback(
                            self._create_goal_response_callback('manual_move')
                        )
                        
                        with self.action_lock:
                            self.action_status['manual_move'] = 'sending'
                        
                        return {'status':'ok','command':cmd,'target':'pushrod','action':'manual_move','action_status':'sending'}
                    
                    elif cmd == 'stop':
                        # Cancel running ManualMove action (unified action)
                        self.get_logger().info('[CMD] Pushrod STOP requested')
                        cancelled_any = False
                        with self.action_lock:
                            goal_handle = self.action_goal_handles.get('manual_move')
                            if goal_handle and self.action_status.get('manual_move') == 'executing':
                                self.get_logger().info(f'[CMD] Cancelling manual_move action (pushrod)...')
                                goal_handle.cancel_goal_async()
                                cancelled_any = True
                        self.get_logger().info(f'[CMD] Pushrod STOP complete: cancelled_any={cancelled_any}')
                        return {'status':'ok','command':'stop','target':'pushrod','cancelled':cancelled_any}
                    
                    elif cmd == 'goto_height':
                        # Extract target_height and mode from payload
                        target_height = payload.get('target_height')
                        mode = payload.get('mode', 'absolute')  # Default to 'absolute' if not specified
                        
                        if target_height is None:
                            return JSONResponse({'error':'target_height field required for goto_height'}, status_code=400)
                        
                        try:
                            target_height = float(target_height)
                        except:
                            return JSONResponse({'error':'invalid target_height'}, status_code=400)
                        
                        # Validate mode
                        if mode not in ['absolute', 'relative']:
                            return JSONResponse({'error':'mode must be "absolute" or "relative"'}, status_code=400)
                        
                        # Create GotoHeight goal (unified platform action with target='pushrod')
                        from lift_robot_interfaces.action import GotoHeight
                        goal_msg = GotoHeight.Goal()
                        goal_msg.target = 'pushrod'
                        goal_msg.target_height = target_height
                        goal_msg.mode = mode
                        
                        # Send action goal (use unified goto_height action)
                        send_goal_future = self.action_clients['goto_height'].send_goal_async(
                            goal_msg,
                            feedback_callback=self._create_feedback_callback('goto_height')
                        )
                        send_goal_future.add_done_callback(
                            self._create_goal_response_callback('goto_height')
                        )
                        
                        with self.action_lock:
                            self.action_status['goto_height'] = 'sending'
                        
                        return {'status':'ok','command':cmd,'target':'pushrod','target_height':target_height,'mode':mode,'action':'goto_height','action_status':'sending'}
                    
                    else:
                        return JSONResponse({'error':f'unknown pushrod command: {cmd}'}, status_code=400)
                
                else:
                    return JSONResponse({'error':f'invalid target: {target}'}, status_code=400)
            
            @app.get('/api/status')
            def get_status():
                """Get current platform and pushrod task status"""
                response = {}
                if self.platform_status:
                    response['platform'] = {
                        'task_state': self.platform_status.get('task_state', 'unknown'),
                        'task_type': self.platform_status.get('task_type'),
                        'task_start_time': self.platform_status.get('task_start_time'),
                        'task_end_time': self.platform_status.get('task_end_time'),
                        'task_duration': self.platform_status.get('task_duration'),
                        'completion_reason': self.platform_status.get('completion_reason'),
                        'control_mode': self.platform_status.get('control_mode'),
                        'movement_state': self.platform_status.get('movement_state'),
                        'current_height': self.platform_status.get('current_height'),
                        'target_height': self.platform_status.get('target_height'),
                        'limit_exceeded': self.platform_status.get('limit_exceeded', False),
                        # Overshoot calibration data
                        'last_goto_target': self.platform_status.get('last_goto_target'),
                        'last_goto_actual': self.platform_status.get('last_goto_actual'),
                        'last_goto_stop_height': self.platform_status.get('last_goto_stop_height'),
                        'last_goto_direction': self.platform_status.get('last_goto_direction'),
                        'last_goto_timestamp': self.platform_status.get('last_goto_timestamp'),
                        # Range scan data
                        'range_scan_active': self.platform_status.get('range_scan_active'),
                        'range_scan_direction': self.platform_status.get('range_scan_direction'),
                        'range_scan_low_height': self.platform_status.get('range_scan_low_height'),
                        'range_scan_high_height': self.platform_status.get('range_scan_high_height'),
                    }
                if self.pushrod_status:
                    response['pushrod'] = {
                        'task_state': self.pushrod_status.get('task_state', 'unknown'),
                        'task_type': self.pushrod_status.get('task_type'),
                        'task_start_time': self.pushrod_status.get('task_start_time'),
                        'task_end_time': self.pushrod_status.get('task_end_time'),
                        'task_duration': self.pushrod_status.get('task_duration'),
                        'completion_reason': self.pushrod_status.get('completion_reason'),
                        'control_mode': self.pushrod_status.get('control_mode'),
                        'movement_state': self.pushrod_status.get('movement_state'),
                        'current_height': self.pushrod_status.get('current_height'),
                        'target_height': self.pushrod_status.get('target_height'),
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
                
                if self.latest_obj is None:
                    return JSONResponse({'success': False, 'error': 'No sensor data available'})
                
                # Check for register_1 (full mode) or height (compact mode as fallback)
                sensor_val = None
                if 'register_1' in self.latest_obj:
                    sensor_val = self.latest_obj['register_1']
                elif 'height' in self.latest_obj:
                    # Compact mode - using calibrated height as "raw" value
                    # This is not ideal for calibration but allows functionality
                    sensor_val = self.latest_obj['height']
                    self.get_logger().warn('Using calibrated height as sensor value - consider disabling publish_compact for proper calibration')
                else:
                    return JSONResponse({'success': False, 'error': 'No sensor raw value (register_1) or height field available'})
                
                if sensor_val is None:
                    return JSONResponse({'success': False, 'error': 'Sensor value is null'})
                
                with self.calib_lock:
                    sample = {
                        'sensor': sensor_val,
                        'height': float(height),
                        'timestamp': time.time()
                    }
                    self.calib_samples.append(sample)
                    total = len(self.calib_samples)
                
                return JSONResponse({
                    'success': True,
                    'sample': sample,
                    'total_samples': total
                })
            
            @app.get('/api/calibration/samples')
            async def get_calibration_samples():
                """Get all calibration samples"""
                with self.calib_lock:
                    return JSONResponse({
                        'success': True,
                        'samples': list(self.calib_samples)
                    })
            
            @app.delete('/api/calibration/samples/{index}')
            async def delete_calibration_sample(index: int):
                """Delete sample by index"""
                with self.calib_lock:
                    if 0 <= index < len(self.calib_samples):
                        removed = self.calib_samples.pop(index)
                        return JSONResponse({
                            'success': True,
                            'removed': removed,
                            'total_samples': len(self.calib_samples)
                        })
                    else:
                        return JSONResponse({
                            'success': False,
                            'error': f'Invalid index: {index}'
                        })
            
            @app.delete('/api/calibration/samples')
            async def clear_calibration_samples():
                """Clear all samples"""
                with self.calib_lock:
                    count = len(self.calib_samples)
                    self.calib_samples.clear()
                    self.calib_scale = None
                    self.calib_offset = None
                
                return JSONResponse({
                    'success': True,
                    'cleared': count
                })
            
            @app.post('/api/calibration/calculate')
            async def calculate_calibration():
                """Calculate linear calibration: height = sensor * scale + offset"""
                with self.calib_lock:
                    if len(self.calib_samples) < 2:
                        return JSONResponse({
                            'success': False,
                            'error': 'Need at least 2 samples for calibration'
                        })
                    
                    xs = [s['sensor'] for s in self.calib_samples]
                    ys = [s['height'] for s in self.calib_samples]
                
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
                
                with self.calib_lock:
                    self.calib_scale = scale
                    self.calib_offset = offset
                
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
                with self.calib_lock:
                    # Get sensor value (prefer register_1, fallback to height)
                    sensor_val = None
                    if self.latest_obj:
                        if 'register_1' in self.latest_obj:
                            sensor_val = self.latest_obj['register_1']
                        elif 'height' in self.latest_obj:
                            sensor_val = self.latest_obj['height']
                    
                    # Calculate estimated height if calibrated
                    estimated_height = None
                    if sensor_val is not None and self.calib_scale is not None and self.calib_offset is not None:
                        estimated_height = sensor_val * self.calib_scale + self.calib_offset
                    
                    # Load timestamp from config file if exists
                    calibrated_at = None
                    config_path = self.get_config_path('draw_wire_calibration.json')
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config_data = json.load(f)
                                calibrated_at = config_data.get('generated_at_iso')
                        except Exception as e:
                            self.get_logger().error(f"Failed to read calibration timestamp: {e}")
                    
                    return JSONResponse({
                        'num_samples': len(self.calib_samples),
                        'calibrated': self.calib_scale is not None and self.calib_offset is not None,
                        'scale': self.calib_scale,
                        'offset': self.calib_offset,
                        'latest_sensor': sensor_val,
                        'estimated_height': estimated_height,
                        'calibrated_at': calibrated_at
                    })
            
            @app.post('/api/calibration/save')
            async def save_calibration():
                """Save calibration to JSON config file"""
                with self.calib_lock:
                    if self.calib_scale is None or self.calib_offset is None:
                        return JSONResponse({
                            'success': False,
                            'error': 'No calibration calculated yet'
                        })
                    
                    scale = self.calib_scale
                    offset = self.calib_offset
                
                # Save to colcon_ws/config directory
                config_dir = self.config_dir
                config_path = self.get_config_path('draw_wire_calibration.json')
                
                try:
                    # Create config directory if it doesn't exist
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                        self.get_logger().info(f"Created config directory: {config_dir}")
                    
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
                    
                    self.get_logger().info(f"Calibration saved: scale={scale}, offset={offset} -> {config_path}")
                    
                    return JSONResponse({
                        'success': True,
                        'filepath': config_path,
                        'scale': scale,
                        'offset': offset,
                        'message': 'Config saved. Restart draw_wire_sensor to apply (no rebuild needed).'
                    })
                except Exception as e:
                    self.get_logger().error(f"Failed to save calibration: {e}")
                    return JSONResponse({
                        'success': False,
                        'error': str(e)
                    })

            # Platform overshoot calibration API endpoints
            @app.post('/api/overshoot/add_sample')
            async def add_overshoot_sample(request: dict):
                """Add a platform overshoot sample"""
                try:
                    direction = request.get('direction')  # 'up' or 'down'
                    target = request.get('target')
                    actual = request.get('actual')
                    stop_height = request.get('stop_height')  # Height when stop command issued
                    
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
                        self.get_logger().warn(f"No stop_height provided for {direction} sample - using target (incorrect)")
                    
                    sample = {
                        'target': float(target) if target is not None else None,
                        'actual': float(actual),
                        'stop_height': float(stop_height) if stop_height is not None else None,
                        'overshoot': overshoot,
                        'timestamp': time.time()
                    }
                    
                    with self.overshoot_lock:
                        if direction == 'up':
                            self.overshoot_samples_up.append(sample)
                        else:
                            self.overshoot_samples_down.append(sample)
                    
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
                        'count_up': len(self.overshoot_samples_up),
                        'count_down': len(self.overshoot_samples_down)
                    })
                except Exception as e:
                    self.get_logger().error(f"Add overshoot sample error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/overshoot/samples')
            async def get_overshoot_samples():
                """Get all overshoot samples"""
                with self.overshoot_lock:
                    # Display normalization: ensure DOWN direction heights are positive for plotting
                    # We do NOT modify stored raw samples; only transform in response.
                    samples_up = self.overshoot_samples_up.copy()
                    raw_down = self.overshoot_samples_down.copy()
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
                    
                    with self.overshoot_lock:
                        samples = self.overshoot_samples_up if direction == 'up' else self.overshoot_samples_down
                        if 0 <= index < len(samples):
                            samples.pop(index)
                            return JSONResponse({
                                'success': True,
                                'count_up': len(self.overshoot_samples_up),
                                'count_down': len(self.overshoot_samples_down)
                            })
                        return JSONResponse({'success': False, 'error': 'Index out of range'})
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.delete('/api/overshoot/samples')
            async def clear_overshoot_samples():
                """Clear all overshoot samples (both up and down)"""
                try:
                    with self.overshoot_lock:
                        self.overshoot_samples_up.clear()
                        self.overshoot_samples_down.clear()
                    return JSONResponse({
                        'success': True,
                        'count_up': len(self.overshoot_samples_up),
                        'count_down': len(self.overshoot_samples_down)
                    })
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.post('/api/overshoot/calculate')
            async def calculate_overshoot():
                """Calculate overshoot using EMA (Exponential Moving Average) for fast convergence"""
                try:
                    with self.overshoot_lock:
                        samples_up = self.overshoot_samples_up.copy()
                        samples_down = self.overshoot_samples_down.copy()
                    
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
                        
                        with self.overshoot_lock:
                            self.overshoot_up = ema_up
                        
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
                        
                        with self.overshoot_lock:
                            self.overshoot_down = ema_down
                        
                        result['overshoot_down'] = ema_down
                        result['ema_down'] = ema_down
                        result['avg_down'] = avg_down
                        result['std_down'] = std_down
                    
                    return JSONResponse(result)
                    
                except Exception as e:
                    self.get_logger().error(f"Overshoot calculation error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/overshoot/status')
            async def overshoot_status():
                """Get current overshoot calibration status.
                Extended: include region list from config file if present.
                """
                try:
                    with self.overshoot_lock:
                        result = {
                            'overshoot_up': self.overshoot_up,
                            'overshoot_down': self.overshoot_down,
                            'count_up': len(self.overshoot_samples_up),
                            'count_down': len(self.overshoot_samples_down),
                            'calibrated_up': self.overshoot_up is not None,
                            'calibrated_down': self.overshoot_down is not None
                        }

                        # Add last goto_height measurement from platform status
                        if self.platform_status:
                            if 'last_goto_target' in self.platform_status:
                                result['last_goto_target'] = self.platform_status['last_goto_target']
                            if 'last_goto_actual' in self.platform_status:
                                result['last_goto_actual'] = self.platform_status['last_goto_actual']
                                result['last_goto_direction'] = self.platform_status.get('last_goto_direction')
                                result['last_goto_timestamp'] = self.platform_status.get('last_goto_timestamp')
                            if 'last_goto_stop_height' in self.platform_status:
                                result['last_goto_stop_height'] = self.platform_status['last_goto_stop_height']

                        # Load timestamp & regions from config file if exists
                        config_path = self.get_config_path('platform_overshoot_calibration.json')
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
                                self.get_logger().error(f"Failed to read overshoot config: {e}")

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
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
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
                    with self.overshoot_lock:
                        for s in self.overshoot_samples_down:
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
                    self.get_logger().error(f"Normalize overshoot file error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.delete('/api/overshoot/config')
            async def clear_overshoot_config():
                """Clear (reset) overshoot calibration config file and in-memory values.
                Removes regions and default values; file is deleted if exists.
                Safe to call before a new full-auto multi-region calibration.
                """
                try:
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    with self.overshoot_lock:
                        self.overshoot_up = None
                        self.overshoot_down = None
                    if os.path.exists(config_path):
                        try:
                            os.remove(config_path)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to delete overshoot config (will overwrite on save): {e}")
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
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
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
                                
                                self.get_logger().info(f"Degree {d}: R²={avg_r2:.4f}, AIC={aic:.2f}, Score={score:.4f}")
                                
                                if score > best_score:
                                    best_score = score
                                    best_degree = d
                            except Exception as e:
                                self.get_logger().warn(f"Degree {d} fit failed: {e}")
                                continue
                        
                        degree = best_degree
                        self.get_logger().info(f"Auto-selected degree: {degree} (score: {best_score:.4f})")
                    
                    # Final fit with selected degree
                    if len(xs) < (degree + 1):
                        degree = len(xs) - 1
                        self.get_logger().warn(f"Reduced degree to {degree} due to insufficient points")
                    
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
                        self.get_logger().warn(f"Plot render error: {e}")
                    return JSONResponse({'success': True, 'fit': fit_block, 'plot_url': plot_url})
                except Exception as e:
                    self.get_logger().error(f"Overshoot fit error: {e}")
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

                    with self.overshoot_lock:
                        overshoot_up = self.overshoot_up
                        overshoot_down = self.overshoot_down

                    if overshoot_up is None and overshoot_down is None:
                        return JSONResponse({'success': False, 'error': 'No calibration calculated yet. Please calculate first.'})

                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    config_dir = self.config_dir
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)

                    # Load existing config (if any) to preserve regions
                    existing = {}
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                existing = json.load(f)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to read existing overshoot config (will recreate): {e}")
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

                    self.get_logger().info(
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
                    self.get_logger().error(f"Failed to save overshoot calibration: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})

            # ═══════════════════════════════════════════════════════════════
            # Force Sensor Calibration API Endpoints (Dual-Channel)
            # ═══════════════════════════════════════════════════════════════
            @app.post('/api/force_calib/add_sample')
            async def add_force_calib_sample(request: dict):
                """Add a force sensor calibration sample (right or left channel)"""
                try:
                    channel = request.get('channel')  # 'right' or 'left'
                    actual_force = request.get('force')  # Actual applied force (N)
                    
                    if channel not in ['right', 'left']:
                        return JSONResponse({'success': False, 'error': 'Channel must be right or left'})
                    if actual_force is None:
                        return JSONResponse({'success': False, 'error': 'Missing force value'})
                    
                    # Get current sensor reading
                    sensor_reading = self.right_force_sensor if channel == 'right' else self.left_force_sensor
                    if sensor_reading is None:
                        return JSONResponse({'success': False, 'error': f'No sensor data for {channel} channel'})
                    
                    sample = {
                        'sensor': float(sensor_reading),
                        'force': float(actual_force),
                        'timestamp': time.time()
                    }
                    
                    with self.force_calib_lock:
                        if channel == 'right':
                            self.force_calib_samples_right.append(sample)
                        else:
                            self.force_calib_samples_left.append(sample)
                    
                    return JSONResponse({
                        'success': True,
                        'sample': sample,
                        'channel': channel,
                        'count_right': len(self.force_calib_samples_right),
                        'count_left': len(self.force_calib_samples_left)
                    })
                except Exception as e:
                    self.get_logger().error(f"Add force calib sample error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/force_calib/samples')
            async def get_force_calib_samples():
                """Get all force calibration samples"""
                with self.force_calib_lock:
                    return JSONResponse({
                        'samples_right': self.force_calib_samples_right.copy(),
                        'samples_left': self.force_calib_samples_left.copy()
                    })
            
            @app.delete('/api/force_calib/samples/{channel}/{index}')
            async def delete_force_calib_sample(channel: str, index: int):
                """Delete a specific force calibration sample"""
                try:
                    if channel not in ['right', 'left']:
                        return JSONResponse({'success': False, 'error': 'Invalid channel'})
                    
                    with self.force_calib_lock:
                        samples = self.force_calib_samples_right if channel == 'right' else self.force_calib_samples_left
                        if 0 <= index < len(samples):
                            removed = samples.pop(index)
                            return JSONResponse({
                                'success': True,
                                'removed': removed,
                                'count_right': len(self.force_calib_samples_right),
                                'count_left': len(self.force_calib_samples_left)
                            })
                        else:
                            return JSONResponse({'success': False, 'error': f'Invalid index: {index}'})
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.delete('/api/force_calib/samples')
            async def clear_force_calib_samples():
                """Clear all force calibration samples"""
                with self.force_calib_lock:
                    count_right = len(self.force_calib_samples_right)
                    count_left = len(self.force_calib_samples_left)
                    self.force_calib_samples_right.clear()
                    self.force_calib_samples_left.clear()
                    self.force_calib_scale_right = None
                    self.force_calib_scale_left = None
                
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
                    with self.force_calib_lock:
                        samples_right = self.force_calib_samples_right.copy()
                    
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
                            
                            with self.force_calib_lock:
                                self.force_calib_scale_right = scale_right
                            
                            result['scale_right'] = scale_right
                            result['max_error_right'] = max_error
                            result['avg_error_right'] = avg_error
                            result['num_samples_right'] = len(samples_right)
                    
                    # Calculate left channel
                    with self.force_calib_lock:
                        samples_left = self.force_calib_samples_left.copy()
                    
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
                            
                            with self.force_calib_lock:
                                self.force_calib_scale_left = scale_left
                            
                            result['scale_left'] = scale_left
                            result['max_error_left'] = max_error
                            result['avg_error_left'] = avg_error
                            result['num_samples_left'] = len(samples_left)
                    
                    if 'scale_right' not in result and 'scale_left' not in result:
                        return JSONResponse({'success': False, 'error': 'Need at least 1 sample per channel'})
                    
                    return JSONResponse(result)
                    
                except Exception as e:
                    self.get_logger().error(f"Force calibration calculation error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/force_calib/status')
            async def force_calib_status():
                """Get current force calibration status"""
                try:
                    with self.force_calib_lock:
                        result = {
                            'scale_right': self.force_calib_scale_right,
                            'scale_left': self.force_calib_scale_left,
                            'count_right': len(self.force_calib_samples_right),
                            'count_left': len(self.force_calib_samples_left),
                            'calibrated_right': self.force_calib_scale_right is not None,
                            'calibrated_left': self.force_calib_scale_left is not None,
                            'current_sensor_right': self.right_force_sensor,
                            'current_sensor_left': self.left_force_sensor
                        }
                        
                        # Load timestamp from config files if exist
                        config_dir = self.config_dir
                        config_path_right = os.path.join(config_dir, 'force_sensor_calibration_52.json')
                        config_path_left = os.path.join(config_dir, 'force_sensor_calibration_53.json')
                        
                        # Load right channel timestamp
                        if os.path.exists(config_path_right):
                            try:
                                with open(config_path_right, 'r') as f:
                                    config_data = json.load(f)
                                    result['calibrated_at_right'] = config_data.get('generated_at_iso')
                            except Exception as e:
                                self.get_logger().error(f"Failed to read right force timestamp: {e}")
                        
                        # Load left channel timestamp
                        if os.path.exists(config_path_left):
                            try:
                                with open(config_path_left, 'r') as f:
                                    config_data = json.load(f)
                                    result['calibrated_at_left'] = config_data.get('generated_at_iso')
                            except Exception as e:
                                self.get_logger().error(f"Failed to read left force timestamp: {e}")
                        
                        return JSONResponse(result)
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.post('/api/force_calib/save')
            async def save_force_calibration():
                """Save force sensor calibration to JSON config files"""
                try:
                    with self.force_calib_lock:
                        scale_right = self.force_calib_scale_right
                        scale_left = self.force_calib_scale_left
                    
                    if scale_right is None and scale_left is None:
                        return JSONResponse({
                            'success': False,
                            'error': 'No calibration calculated yet. Please calculate first.'
                        })
                    
                    config_dir = self.config_dir
                    saved_files = []
                    
                    # Save right channel (device_id=52)
                    if scale_right is not None:
                        config_path_right = os.path.join(config_dir, 'force_sensor_calibration_52.json')
                        config_data_right = {
                            'device_id': 52,
                            'topic': '/force_sensor',
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
                        self.get_logger().info(f"Saved right force calibration: scale={scale_right}")
                    
                    # Save left channel (device_id=53)
                    if scale_left is not None:
                        config_path_left = os.path.join(config_dir, 'force_sensor_calibration_53.json')
                        config_data_left = {
                            'device_id': 53,
                            'topic': '/force_sensor_2',
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
                        self.get_logger().info(f"Saved left force calibration: scale={scale_left}")
                    
                    return JSONResponse({
                        'success': True,
                        'saved_files': saved_files,
                        'scale_right': scale_right,
                        'scale_left': scale_left,
                        'message': 'Config saved. Restart force sensor nodes to apply.'
                    })
                except Exception as e:
                    self.get_logger().error(f"Failed to save force calibration: {e}")
                    return JSONResponse({
                        'success': False,
                        'error': str(e)
                    })

            @app.post('/api/force_calib/tare')
            async def tare_force_sensor(request: dict):
                """
                Tare (zero) force sensor by sending Modbus command.
                
                Request body:
                {
                    "channel": "right" or "left",
                    "device_id": 52 or 53
                }
                
                Sends Modbus function code 06 (Write Single Register) to address 0x0011 with value 0x0001.
                """
                try:
                    from modbus_driver_interfaces.srv import ModbusRequest
                    
                    body = await request.json()
                    channel = body.get('channel', 'right')
                    device_id = body.get('device_id', 52)
                    
                    # Validate device_id matches channel
                    if channel == 'right' and device_id != 52:
                        return JSONResponse({
                            'success': False,
                            'error': f'Channel "right" requires device_id=52, got {device_id}'
                        })
                    if channel == 'left' and device_id != 53:
                        return JSONResponse({
                            'success': False,
                            'error': f'Channel "left" requires device_id=53, got {device_id}'
                        })
                    
                    # Create Modbus request client
                    tare_client = self.create_client(ModbusRequest, '/modbus_request')
                    
                    # Wait for service with timeout
                    if not tare_client.wait_for_service(timeout_sec=2.0):
                        return JSONResponse({
                            'success': False,
                            'error': 'Modbus service not available'
                        })
                    
                    # Prepare tare command (FC06, register 0x0011, value 0x0001)
                    request_msg = ModbusRequest.Request()
                    request_msg.function_code = 0x06  # Write Single Register
                    request_msg.slave_id = device_id
                    request_msg.address = 0x0011  # Tare command register
                    request_msg.count = 0  # Not used for write operations
                    request_msg.values = [0x0001]  # Trigger tare
                    request_msg.seq_id = int(time.time() * 1000) % 65536
                    
                    self.get_logger().info(
                        f'Sending tare command: device_id={device_id} (0x{device_id:02X}), '
                        f'func=0x06, reg=0x0011, value=0x0001'
                    )
                    
                    # Send request
                    future = tare_client.call_async(request_msg)
                    
                    # Wait for response with timeout
                    start_time = time.time()
                    while not future.done():
                        if time.time() - start_time > 3.0:
                            return JSONResponse({
                                'success': False,
                                'error': 'Tare command timeout (3s)'
                            })
                        await asyncio.sleep(0.05)
                    
                    response = future.result()
                    
                    if response.success:
                        self.get_logger().info(f'Tare successful for device_id={device_id}')
                        return JSONResponse({
                            'success': True,
                            'message': f'Tared {channel} sensor (device_id={device_id}). Zero point reset.',
                            'device_id': device_id,
                            'channel': channel
                        })
                    else:
                        return JSONResponse({
                            'success': False,
                            'error': f'Modbus tare command failed: {response.message}'
                        })
                        
                except Exception as e:
                    self.get_logger().error(f"Tare force sensor error: {e}")
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
                    config_path = self.get_config_path('platform_range.json')
                    
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
                    self.get_logger().error(f"Get platform range error: {e}")
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
                    
                    config_path = self.get_config_path('platform_range.json')
                    config_dir = self.config_dir
                    
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
                    
                    self.get_logger().info(f"Saved platform range: [{actual_min:.2f}, {actual_max:.2f}]mm, safe: [{safe_min:.2f}, {safe_max:.2f}]mm")
                    
                    return JSONResponse({
                        'success': True,
                        'actual_min': config['actual_min'],
                        'actual_max': config['actual_max'],
                        'safe_min': config['safe_min'],
                        'safe_max': config['safe_max'],
                        'detected_at_iso': config['detected_at_iso']
                    })
                except Exception as e:
                    self.get_logger().error(f"Set platform range error: {e}")
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
                    self.get_logger().error(f"Detect safe range error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.post('/api/systematic/set_range')
            async def set_safe_range(payload: dict):
                """Set detected range values and save to config immediately.
                Expects: {actual_min, actual_max, safe_min, safe_max}
                """
                try:
                    with self.overshoot_lock:
                        self.actual_range_min = payload.get('actual_min')
                        self.actual_range_max = payload.get('actual_max')
                        self.safe_range_min = payload.get('safe_min')
                        self.safe_range_max = payload.get('safe_max')
                    
                    # Save range to config file immediately
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    config_dir = self.config_dir
                    
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                    
                    # Load existing config or create new
                    config = {}
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config = json.load(f)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to load existing config: {e}")
                    
                    # Ensure basic fields exist
                    if 'enable' not in config:
                        config['enable'] = True
                    if 'format_version' not in config:
                        config['format_version'] = 2
                    
                    # Update range block (only updates range, preserves other fields)
                    config['range'] = {
                        'actual_min': self.actual_range_min,
                        'actual_max': self.actual_range_max,
                        'safe_min': self.safe_range_min,
                        'safe_max': self.safe_range_max,
                        'detected_at': time.time(),
                        'detected_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                    }
                    
                    # Write config
                    with open(config_path, 'w') as f:
                        json.dump(config, f, indent=2)
                    
                    self.get_logger().info(f"Saved range to config: [{self.actual_range_min:.2f}, {self.actual_range_max:.2f}]mm")
                    
                    return JSONResponse({
                        'success': True,
                        'actual_min': self.actual_range_min,
                        'actual_max': self.actual_range_max,
                        'safe_min': self.safe_range_min,
                        'safe_max': self.safe_range_max
                    })
                except Exception as e:
                    self.get_logger().error(f"Set safe range error: {e}")
                    import traceback
                    traceback.print_exc()
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/systematic/range')
            async def get_range_status():
                """Get current range values."""
                with self.overshoot_lock:
                    return JSONResponse({
                        'actual_min': self.actual_range_min,
                        'actual_max': self.actual_range_max,
                        'safe_min': self.safe_range_min,
                        'safe_max': self.safe_range_max,
                        'has_range': self.safe_range_min is not None and self.safe_range_max is not None
                    })
            
            @app.post('/api/systematic/start_calibration')
            async def start_systematic_calibration():
                """Prepare for calibration by clearing samples in memory and config file."""
                try:
                    with self.overshoot_lock:
                        self.systematic_samples.clear()
                    
                    # Clear samples from config file
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    config_dir = self.config_dir
                    
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                    
                    # Load existing config or create new with defaults
                    config = {}
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config = json.load(f)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to load existing config: {e}")
                    
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
                    
                    self.get_logger().info("Cleared samples, ready for new calibration")
                    
                    return JSONResponse({'success': True, 'message': 'Calibration started, samples cleared'})
                except Exception as e:
                    self.get_logger().error(f"Start calibration error: {e}")
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
                    
                    with self.overshoot_lock:
                        self.systematic_samples.append(sample)
                        samples_copy = self.systematic_samples.copy()
                    
                    # Save sample to config file immediately
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    config_dir = self.config_dir
                    
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                    
                    # Load existing config or create new with defaults
                    config = {}
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config = json.load(f)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to load existing config: {e}")
                    
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
                    
                    self.get_logger().info(
                        f"Added systematic sample: height={height:.2f}mm, "
                        f"overshoot={overshoot:.2f}mm, direction={direction}"
                    )
                    
                    return JSONResponse({
                        'success': True,
                        'sample': sample,
                        'total_count': len(self.systematic_samples)
                    })
                except Exception as e:
                    self.get_logger().error(f"Add systematic sample error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/systematic/samples')
            async def get_systematic_samples():
                """Get all systematic calibration samples from config file."""
                config_path = self.get_config_path('platform_overshoot_calibration.json')
                
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
                        self.get_logger().warn(f"Failed to load samples from config: {e}")
                
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
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    
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
                    with self.overshoot_lock:
                        if 0 <= index < len(self.systematic_samples):
                            self.systematic_samples.pop(index)
                    
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
                    
                    self.get_logger().info(f"Deleted sample #{index}: {deleted_sample}")
                    
                    return JSONResponse({
                        'success': True,
                        'deleted_sample': deleted_sample,
                        'remaining_count': len(samples)
                    })
                except Exception as e:
                    self.get_logger().error(f"Delete systematic sample error: {e}")
                    import traceback
                    traceback.print_exc()
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.delete('/api/systematic/samples')
            async def clear_systematic_samples():
                """Clear all systematic calibration samples from memory and config file."""
                with self.overshoot_lock:
                    count = len(self.systematic_samples)
                    self.systematic_samples.clear()
                
                # Clear samples from config file
                config_path = self.get_config_path('platform_overshoot_calibration.json')
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
                        
                        self.get_logger().info(f"Cleared {count} systematic samples from config")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to clear samples from config: {e}")
                
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
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    samples = []
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config = json.load(f)
                                samples = config.get('samples', [])
                        except Exception as e:
                            self.get_logger().warn(f"Failed to load samples from config: {e}")
                    
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
                    self.get_logger().error(f"Plot systematic data error: {e}")
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
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    samples = []
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config = json.load(f)
                                samples = config.get('samples', [])
                        except Exception as e:
                            self.get_logger().warn(f"Failed to load samples from config: {e}")
                    
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
                    with self.overshoot_lock:
                        if self.actual_range_min is not None and self.actual_range_max is not None:
                            range_block = {
                                'actual_min': self.actual_range_min,
                                'actual_max': self.actual_range_max,
                                'safe_min': self.safe_range_min,
                                'safe_max': self.safe_range_max,
                                'detected_at': time.time(),
                                'detected_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                            }
                    
                    # Save to config file
                    config_path = self.get_config_path('platform_overshoot_calibration.json')
                    config_dir = self.config_dir
                    
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
                            self.get_logger().warn(f"Failed to load existing config: {e}")
                    
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
                        self.get_logger().info("Removing 'default' from config - using fitted polynomial")
                        del config['default']
                    
                    # Write config
                    with open(config_path, 'w') as f:
                        json.dump(config, f, indent=2)
                    
                    self.get_logger().info(f"Saved systematic fit: degree={degree}, range=[{x_min:.2f}, {x_max:.2f}]mm")
                    
                    return JSONResponse({
                        'success': True,
                        'config_path': config_path,
                        'fit': fit_block
                    })
                except Exception as e:
                    self.get_logger().error(f"Save systematic fit error: {e}")
                    import traceback
                    traceback.print_exc()
                    return JSONResponse({'success': False, 'error': str(e)})

            # ═══════════════════════════════════════════════════════════════
            # ROS2 Action API Endpoints (Testing Action vs Topic)
            # ═══════════════════════════════════════════════════════════════
            
            @app.post('/api/action/goto_height')
            async def send_action_goal(payload: dict):
                """Send GotoHeight Action goal"""
                if not self.action_client:
                    return JSONResponse({'error': 'Action client not available'}, status_code=503)
                
                target_height = payload.get('target_height')
                if target_height is None:
                    return JSONResponse({'error': 'target_height required'}, status_code=400)
                
                try:
                    from lift_robot_interfaces.action import GotoHeight
                    
                    # Create goal
                    goal_msg = GotoHeight.Goal()
                    goal_msg.target_height = float(target_height)
                    
                    self.get_logger().info(f'[Action] Sending goal: target_height={target_height}mm')
                    
                    # Reset state
                    with self.action_lock:
                        self.action_status = 'sending'
                        self.action_feedback = None
                        self.action_result = None
                        self.action_goal_handle = None
                    
                    # Send goal (non-blocking)
                    send_goal_future = self.action_client.send_goal_async(
                        goal_msg, 
                        feedback_callback=self.action_feedback_callback
                    )
                    send_goal_future.add_done_callback(self.action_goal_response_callback)
                    
                    return JSONResponse({
                        'status': 'goal_sent',
                        'target_height': target_height
                    })
                    
                except ImportError:
                    return JSONResponse({'error': 'GotoHeight action not available - interface not built'}, status_code=503)
                except Exception as e:
                    self.get_logger().error(f'[Action] Send goal error: {e}')
                    return JSONResponse({'error': str(e)}, status_code=500)
            
            @app.post('/api/action/cancel_goto_height')
            async def cancel_action():
                """Cancel active Action goal"""
                with self.action_lock:
                    if not self.action_goal_handle:
                        return JSONResponse({'error': 'No active goal to cancel'}, status_code=400)
                    
                    goal_handle = self.action_goal_handle
                
                try:
                    self.get_logger().info('[Action] Cancelling goal...')
                    cancel_future = goal_handle.cancel_goal_async()
                    # Note: Result will be handled by result callback
                    
                    return JSONResponse({'status': 'cancel_sent'})
                    
                except Exception as e:
                    self.get_logger().error(f'[Action] Cancel error: {e}')
                    return JSONResponse({'error': str(e)}, status_code=500)
            
            @app.get('/api/action/status')
            async def get_action_status():
                """Get current Action status and feedback (polled by frontend)"""
                with self.action_lock:
                    return JSONResponse({
                        'status': self.action_status,
                        'feedback': self.action_feedback,
                        'result': self.action_result
                    })

            @app.websocket('/ws')
            async def ws_endpoint(ws: WebSocket):
                await ws.accept()
                self.connections.append(ws)
                if self.latest_raw:
                    await ws.send_text(self.latest_raw)
                try:
                    while True:
                        try:
                            _ = await asyncio.wait_for(ws.receive_text(), timeout=30)
                        except asyncio.TimeoutError:
                            await ws.send_text('ping')
                except WebSocketDisconnect:
                    pass
                finally:
                    if ws in self.connections:
                        self.connections.remove(ws)

            app.mount('/static', StaticFiles(directory=web_dir), name='static')

            # Create and own event loop explicitly; run uvicorn server inside
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            # Use 'warning' log level to suppress access logs (INFO level logs every request)
            config = uvicorn.Config(app, host='0.0.0.0', port=self.port, loop='asyncio', log_level='warning')
            server = uvicorn.Server(config)

            async def serve():
                try:
                    self.get_logger().info(f"Web server started on 0.0.0.0:{self.port}")
                    await server.serve()
                except Exception as e:
                    self.get_logger().error(f"Web server serve error: {e}")

            self.loop.create_task(serve())
            try:
                self.loop.run_forever()
            except Exception as e:
                self.get_logger().error(f"Web server loop error: {e}")
            finally:
                try:
                    self.loop.close()
                except Exception as e:
                    self.get_logger().error(f"Web server loop close error: {e}")

        threading.Thread(target=run, daemon=True).start()

def main(args=None):
    if not FASTAPI:
        print('Install fastapi uvicorn first: pip install fastapi uvicorn')
        return
    rclpy.init(args=args)
    node = LiftRobotWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
