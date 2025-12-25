#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string, send_from_directory, jsonify
import os
import threading
import time
import socket
import concurrent.futures
from ament_index_python.packages import get_package_share_directory
from common.config_manager import ConfigManager

# Import RTDE for robot status
try:
    from rtde import RTDE
    RTDE_AVAILABLE = True
except ImportError as e:
    print(f"Warning: RTDE library not available: {e}")
    RTDE_AVAILABLE = False

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')
        
        self.get_logger().info('System Monitor Node starting...')
        
        # Task statistics
        self.active_tasks = 0
        self.pending_tasks = 0
        self.completed_tasks = 0
        self.system_status = 'Ready'
        self.robot_connection = 'Disconnected'
        
        # Load configuration
        self.config = ConfigManager()
        ur15_config = self.config.get_robot('ur15')
        amr_config = self.config.get('amr')
        lift_robot_config = self.config.get('lift_robot')
        services_config = self.config.get('services')
        shared_config = self.config.get('shared')
        
        self.ur15_web_port = ur15_config.get('web.port', 8030)
        self.amr_web_port = amr_config.get('web.port', 5000)
        self.courier_web_host = lift_robot_config.get('web.host', '192.168.1.3')
        self.courier_web_port = lift_robot_config.get('web.port', 8090)
        
        # Device IP addresses from config
        self.ur15_ip = ur15_config.get('robot.ip', '192.168.1.15')
        self.courier_ip = shared_config.get('network.courier.ip', '192.168.1.3')  
        self.amr_ip = amr_config.get('ip', '192.168.1.123')
        
        # Service URLs
        self.robot_status_port = services_config.get('robot_status_redis', {}).get('web', {}).get('port', 8005)
        self.positioning_3d_port = services_config.get('positioning_3d', {}).get('port', 8004)
        self.camcalib_web_port = services_config.get('camcalib_web', {}).get('port', 8006)
        self.image_labeling_port = services_config.get('image_labeling', {}).get('port', 8007)
        self.workflow_config_port = services_config.get('workflow_config_center', {}).get('port', 8008)
        self.ffpp_server_url = shared_config.get('network', {}).get('ffpp_server', {}).get('url', 'http://msraig-ubuntu-4.guest.corp.microsoft.com:8001')
        
        # RTDE connection for robot_mode and safety_mode
        self.rtde_connection = None
        self.rtde_connected = False
        self.rtde_lock = threading.Lock()
        self.rtde_thread_running = True  # Set to True first so reader thread can start
        self.cached_robot_mode = None
        self.cached_safety_mode = None
        self.cached_rtde_data = {}  # Cache for all RTDE data
        self.robot_ip = '192.168.1.15'  # Default UR15 IP
        
        # Task manager for demo operations (lazy initialization)
        self.task_manager = None
        
        # Get package share directory for web files
        try:
            self.package_share_directory = get_package_share_directory('system_monitor')
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
        
        @self.app.route('/api/web_urls')
        def get_web_urls():
            """API endpoint to get web interface URLs"""
            # Use localhost for services running on same machine
            localhost = 'localhost'
            return jsonify({
                'success': True,
                'ur15_web_url': f'http://msra-yizhong.guest.corp.microsoft.com:{self.ur15_web_port}/',
                'amr_web_url': f'http://msra-yizhong.guest.corp.microsoft.com:{self.amr_web_port}/',
                'courier_web_url': f'http://{self.courier_web_host}:{self.courier_web_port}',
                'robot_status_url': f'http://{localhost}:{self.robot_status_port}',
                'positioning_3d_url': f'http://{localhost}:{self.positioning_3d_port}',
                'camcalib_web_url': f'http://{localhost}:{self.camcalib_web_port}',
                'image_labeling_url': f'http://{localhost}:{self.image_labeling_port}',
                'workflow_config_url': f'http://{localhost}:{self.workflow_config_port}',
                'ffpp_server_url': self.ffpp_server_url
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
        
        @self.app.route('/api/rtde_data')
        def get_rtde_data():
            """API endpoint to get comprehensive RTDE data for robot monitor"""
            rtde_data = {}
            rtde_connected = False
            
            with self.rtde_lock:
                rtde_connected = self.rtde_connected
                rtde_data = self.cached_rtde_data.copy() if hasattr(self, 'cached_rtde_data') else {}
            
            return jsonify({
                'success': rtde_connected,
                'rtde_connected': rtde_connected,
                'rtde_data': rtde_data if rtde_connected else {}
            })
        
        @self.app.route('/api/courier_robot_status')
        def get_courier_robot_status():
            """API endpoint to get courier robot status from courier_robot_webapi.py"""
            try:
                # Import the CourierRobotWebAPI class using common workspace utils
                import sys
                from common.workspace_utils import get_scripts_directory
                
                # Get the scripts directory path
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    raise RuntimeError("Could not find scripts directory")
                
                if scripts_dir not in sys.path:
                    sys.path.insert(0, scripts_dir)
                
                from courier_robot_webapi import CourierRobotWebAPI
                
                # Create an instance and get status
                courier_api = CourierRobotWebAPI()
                status_data = courier_api.get_status()
                
                return jsonify(status_data)
                
            except Exception as e:
                self.get_logger().error(f"Error fetching courier robot status: {e}")
                return jsonify({
                    'success': False,
                    'error': str(e),
                    'platform': {
                        'task_state': 'Error',
                        'movement_state': 'Error',
                        'control_mode': 'Error',
                        'force_limit_status': 'Error'
                    },
                    'pushrod': {
                        'task_state': 'Error',
                        'movement_state': 'Error'
                    },
                    'sensors': {},
                    'server_id': 'N/A'
                })

        @self.app.route('/api/device_connections')
        def get_device_connections():
            """API endpoint to get device connection status"""

            # Use ThreadPoolExecutor for parallel connection checks
            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                futures = {
                    'ur15': executor.submit(self._check_tcp_connection, self.ur15_ip, 29999, timeout=0.8),
                    'courier': executor.submit(self._check_tcp_connection, self.courier_ip, 22, timeout=0.8),
                    'amr': executor.submit(self._check_tcp_connection, self.amr_ip, 22, timeout=0.8)
                }
                
                # Get results with overall timeout
                ur15_connected = False
                courier_connected = False
                amr_connected = False
                
                try:
                    ur15_connected = futures['ur15'].result(timeout=1.0)
                    courier_connected = futures['courier'].result(timeout=1.0)
                    amr_connected = futures['amr'].result(timeout=1.0)
                except concurrent.futures.TimeoutError:
                    self.get_logger().debug("Device connection check timeout")
            
            return jsonify({
                'success': True,
                'connections': {
                    'ur15': ur15_connected,
                    'courier': courier_connected,
                    'amr': amr_connected
                },
                'ips': {
                    'ur15': self.ur15_ip,
                    'courier': self.courier_ip,
                    'amr': self.amr_ip
                }
            })
        
        @self.app.route('/api/move_to_position', methods=['POST'])
        def move_to_position():
            """API endpoint to move robot to home or task position"""
            try:
                from flask import request
                data = request.get_json()
                position_type = data.get('position_type')  # 'home' or 'task'
                
                if position_type not in ['home', 'task']:
                    return jsonify({
                        'success': False,
                        'message': 'Invalid position_type. Must be "home" or "task"'
                    }), 400
                
                # Import URMoveToTarget
                try:
                    import sys
                    from pathlib import Path
                    
                    # Add scripts directory to path
                    workspace_root = Path(__file__).resolve().parents[3]
                    scripts_dir = workspace_root / 'scripts'
                    if str(scripts_dir) not in sys.path:
                        sys.path.insert(0, str(scripts_dir))
                    
                    from ur_move_to_target import URMoveToTarget
                    
                    self.get_logger().info(f'Moving robot to {position_type} position...')
                    
                    # Create controller instance
                    controller = URMoveToTarget()
                    
                    if not controller.robot:
                        self.get_logger().error('Failed to initialize robot controller')
                        return jsonify({
                            'success': False,
                            'message': 'Failed to initialize robot controller'
                        }), 500
                    
                    # Execute the requested movement
                    if position_type == 'home':
                        success = controller.movej_to_home_position()
                    else:  # task
                        success = controller.movej_to_task_position()
                    
                    if success:
                        self.get_logger().info(f'Successfully moved to {position_type} position')
                        return jsonify({
                            'success': True,
                            'message': f'Successfully moved to {position_type} position'
                        })
                    else:
                        self.get_logger().error(f'Failed to move to {position_type} position')
                        return jsonify({
                            'success': False,
                            'message': f'Failed to move to {position_type} position'
                        })
                
                except ImportError as e:
                    self.get_logger().error(f'Failed to import URMoveToTarget: {e}')
                    return jsonify({
                        'success': False,
                        'message': f'Failed to import URMoveToTarget: {str(e)}'
                    }), 500
                except Exception as e:
                    self.get_logger().error(f'Error moving to {position_type} position: {e}')
                    return jsonify({
                        'success': False,
                        'message': f'Error: {str(e)}'
                    }), 500
                    
            except Exception as e:
                self.get_logger().error(f'Error in move_to_position API: {e}')
                return jsonify({
                    'success': False,
                    'message': f'Internal server error: {str(e)}'
                }), 500
        
        @self.app.route('/api/demo/execute_step', methods=['POST'])
        def execute_demo_step():
            """API endpoint to execute a single demo step"""
            try:
                from flask import request
                data = request.get_json()
                step_number = data.get('step')
                
                if step_number is None:
                    return jsonify({
                        'success': False,
                        'message': 'Step number is required'
                    }), 400
                
                # Get server_index from request
                server_index = data.get('server_index', None)
                if server_index is None:
                    return jsonify({
                        'success': False,
                        'message': 'Server index is required'
                    }), 400
                
                # Import TaskManager and execute the step
                try:
                    import sys
                    from pathlib import Path
                    
                    # Add scripts directory to path
                    workspace_root = Path(__file__).resolve().parents[3]
                    scripts_dir = workspace_root / 'scripts'
                    if str(scripts_dir) not in sys.path:
                        sys.path.insert(0, str(scripts_dir))
                    
                    from demo_task_manager import TaskManager
                    
                    self.get_logger().info(f'Executing demo step {step_number} with server_index={server_index}...')
                    
                    # Initialize TaskManager with server_index if not already initialized or if server_index changed
                    if not hasattr(self, 'task_manager') or self.task_manager is None or self.task_manager.server_index != server_index:
                        self.get_logger().info(f'Creating new TaskManager with server_index={server_index}')
                        self.task_manager = TaskManager(server_index=server_index)
                    
                    # Execute the specific step
                    success = self._execute_single_step(step_number)
                    
                    if success:
                        self.get_logger().info(f'Step {step_number} completed successfully')
                        return jsonify({
                            'success': True,
                            'message': f'Step {step_number} completed successfully'
                        })
                    else:
                        self.get_logger().error(f'Step {step_number} failed')
                        return jsonify({
                            'success': False,
                            'message': f'Step {step_number} failed'
                        })
                
                except ImportError as e:
                    self.get_logger().error(f'Failed to import TaskManager: {e}')
                    return jsonify({
                        'success': False,
                        'message': f'Failed to import TaskManager: {str(e)}'
                    }), 500
                except Exception as e:
                    self.get_logger().error(f'Error executing step {step_number}: {e}')
                    return jsonify({
                        'success': False,
                        'message': f'Error executing step: {str(e)}'
                    }), 500
                    
            except Exception as e:
                self.get_logger().error(f'Error processing request: {e}')
                return jsonify({
                    'success': False,
                    'message': f'Error processing request: {str(e)}'
                }), 500
        
    
    def _execute_single_step(self, step_number):
        """
        Execute a single step from the complete task sequence
        
        Args:
            step_number (int): The step number to execute (1-26)
            
        Returns:
            bool: True if step completed successfully, False otherwise
        """
        try:
            # Map step numbers to their corresponding methods
            step_mapping = {
                1: self._step_1_amr_move_arms,
                2: self._step_2_ur15_rack_positioning,
                3: self._step_3_ur15_get_tool_rotate,
                4: self._step_4_ur15_unlock_knobs,
                5: self._step_5_ur15_tool_exchange_rotate_to_pushpull,
                6: self._step_6_ur15_open_handles,
                7: self._step_7_ur15_close_left,
                8: self._step_8_ur15_close_right,
                9: self._step_9_ur15_tool_exchange_pushpull_to_extract,
                10: self._step_10_ur15_move_to_target,
                11: self._step_11_amr_courier_to_extraction,
                12: self._step_12_ur15_extract_server,
                13: self._step_13_amr_courier_back_to_dock,
                14: self._step_14_ur15_return_extract_get_frame,
                15: self._step_15_ur15_put_frame,
                16: self._step_16_ur15_get_tool_extract,
                17: self._step_17_ur15_move_to_target,
                18: self._step_18_amr_courier_to_insertion,
                19: self._step_19_ur15_insert_server,
                20: self._step_20_amr_courier_back_to_dock,
                21: self._step_21_ur15_tool_exchange_extract_to_rotate,
                22: self._step_22_ur15_unlock_knob_insert,
                23: self._step_23_ur15_tool_exchange_rotate_to_pushpull,
                24: self._step_24_ur15_close_handles,
                25: self._step_25_ur15_return_tool_pushpull,
                26: self._step_26_ur15_move_to_target,
            }
            
            # Get the step function
            step_func = step_mapping.get(step_number)
            if step_func is None:
                self.get_logger().error(f'Invalid step number: {step_number}')
                return False
            
            # Execute the step
            self.get_logger().info(f'Executing Step {step_number}...')
            result = step_func()
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'Error executing step {step_number}: {e}')
            return False
    
    # Individual step implementation methods
    def _step_1_amr_move_arms(self):
        """Step 1: AMR move arms from DOCK to SIDE"""
        try:
            result = self.task_manager.amr_controller.amr_move_arm_from_dock_to_side()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 1 error: {e}')
            return False
    
    def _step_2_ur15_rack_positioning(self):
        """Step 2: UR15 rack positioning"""
        try:
            result = self.task_manager.ur15_execute_rack_positioning_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 2 error: {e}')
            return False
    
    def _step_3_ur15_get_tool_rotate(self):
        """Step 3: UR15 get tool_rotate"""
        try:
            result = self.task_manager.ur_operate_tools.get_tool_from_task_position("tool_rotate")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 3 error: {e}')
            return False
    
    def _step_4_ur15_unlock_knobs(self):
        """Step 4: UR15 unlock knobs"""
        try:
            result = self.task_manager.ur15_execute_unlock_knob_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 4 error: {e}')
            return False
    
    def _step_5_ur15_tool_exchange_rotate_to_pushpull(self):
        """Step 5: UR15 tool exchange (rotate→pushpull)"""
        try:
            result = self.task_manager.ur_operate_tools.return_tool1_get_tool2_from_task(
                tool1_name="tool_rotate", tool2_name="tool_pushpull")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 5 error: {e}')
            return False
    
    def _step_6_ur15_open_handles(self):
        """Step 6: UR15 open handles"""
        try:
            result = self.task_manager.ur15_execute_open_handle_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 6 error: {e}')
            return False
    
    def _step_7_ur15_close_left(self):
        """Step 7: UR15 close left handle"""
        try:
            result = self.task_manager.ur15_execute_close_left_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 7 error: {e}')
            return False
    
    def _step_8_ur15_close_right(self):
        """Step 8: UR15 close right handle"""
        try:
            result = self.task_manager.ur15_execute_close_right_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 8 error: {e}')
            return False
    
    def _step_9_ur15_tool_exchange_pushpull_to_extract(self):
        """Step 9: UR15 tool exchange (pushpull→extract)"""
        try:
            result = self.task_manager.ur_operate_tools.return_tool1_get_tool2_from_task(
                tool1_name="tool_pushpull", tool2_name="tool_extract")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 9 error: {e}')
            return False
    
    def _step_10_ur15_move_to_target(self):
        """Step 10: UR15 move to target position"""
        try:
            import math
            # First move
            move_result = self.task_manager.ur_operate_tools.movel_to_target_position(
                index=self.task_manager.server_index,
                execution_order=[1, 3, 2],
                offset_in_rack=[0, -0.65, 0.45]
            )
            if move_result:
                return False
            
            # Then movej to target angles
            target_joints_degrees = [113.2, -62.4, 65.1, -92.5, -90.3, -61.8]
            target_joints_radians = [math.radians(angle) for angle in target_joints_degrees]
            movej_result = self.task_manager.ur_operate_tools.robot.movej(
                target_joints_radians, a=0.5, v=0.5)
            return movej_result == 0
        except Exception as e:
            self.get_logger().error(f'Step 10 error: {e}')
            return False
    
    def _step_11_amr_courier_to_extraction(self):
        """Step 11: AMR courier to extraction position"""
        try:
            result = self.task_manager.amr_controller.amr_move_courier_from_dock_to_extraction_position()
            return result and result.get('success', False)
        except Exception as e:
            self.get_logger().error(f'Step 11 error: {e}')
            return False
    
    def _step_12_ur15_extract_server(self):
        """Step 12: UR15 extract server"""
        try:
            result = self.task_manager.ur15_execute_extract_server_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 12 error: {e}')
            return False
    
    def _step_13_amr_courier_back_to_dock(self):
        """Step 13: AMR courier back to dock"""
        try:
            result = self.task_manager.amr_controller.amr_move_courier_from_extraction_position_to_dock()
            return result and result.get('success', False)
        except Exception as e:
            self.get_logger().error(f'Step 13 error: {e}')
            return False
    
    def _step_14_ur15_return_extract_get_frame(self):
        """Step 14: UR15 return tool_extract & get frame"""
        try:
            result = self.task_manager.ur_operate_tools.return_tool_get_frame_from_task(tool_name="tool_extract")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 14 error: {e}')
            return False
    
    def _step_15_ur15_put_frame(self):
        """Step 15: UR15 put frame"""
        try:
            result = self.task_manager.ur15_execute_put_frame_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 15 error: {e}')
            return False
    
    def _step_16_ur15_get_tool_extract(self):
        """Step 16: UR15 get tool_extract"""
        try:
            result = self.task_manager.ur_operate_tools.get_tool_from_task_position("tool_extract")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 16 error: {e}')
            return False
    
    def _step_17_ur15_move_to_target(self):
        """Step 17: UR15 move to target position"""
        try:
            import math
            # First move
            move_result = self.task_manager.ur_operate_tools.movel_to_target_position(
                index=self.task_manager.server_index,
                execution_order=[1, 3, 2],
                offset_in_rack=[0, -0.65, 0.45]
            )
            if move_result:
                return False
            
            # Then movej to target angles
            target_joints_degrees = [113.2, -62.4, 65.1, -92.5, -90.3, -61.8]
            target_joints_radians = [math.radians(angle) for angle in target_joints_degrees]
            movej_result = self.task_manager.ur_operate_tools.robot.movej(
                target_joints_radians, a=1.0, v=1.0)
            return movej_result == 0
        except Exception as e:
            self.get_logger().error(f'Step 17 error: {e}')
            return False
    
    def _step_18_amr_courier_to_insertion(self):
        """Step 18: AMR courier to insertion position"""
        try:
            result = self.task_manager.amr_controller.amr_move_courier_from_dock_to_insertion_position()
            return result and result.get('success', False)
        except Exception as e:
            self.get_logger().error(f'Step 18 error: {e}')
            return False
    
    def _step_19_ur15_insert_server(self):
        """Step 19: UR15 insert server"""
        try:
            result = self.task_manager.ur15_execute_insert_server_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 19 error: {e}')
            return False
    
    def _step_20_amr_courier_back_to_dock(self):
        """Step 20: AMR courier back to dock"""
        try:
            result = self.task_manager.amr_controller.amr_move_courier_from_insertion_position_to_dock()
            return result and result.get('success', False)
        except Exception as e:
            self.get_logger().error(f'Step 20 error: {e}')
            return False
    
    def _step_21_ur15_tool_exchange_extract_to_rotate(self):
        """Step 21: UR15 tool exchange (extract→rotate)"""
        try:
            result = self.task_manager.ur_operate_tools.return_tool1_get_tool2_from_task(
                tool1_name="tool_extract", tool2_name="tool_rotate")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 21 error: {e}')
            return False
    
    def _step_22_ur15_unlock_knob_insert(self):
        """Step 22: UR15 unlock knob insert"""
        try:
            result = self.task_manager.ur15_execute_unlock_knob_insert_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 22 error: {e}')
            return False
    
    def _step_23_ur15_tool_exchange_rotate_to_pushpull(self):
        """Step 23: UR15 tool exchange (rotate→pushpull)"""
        try:
            result = self.task_manager.ur_operate_tools.return_tool1_get_tool2_from_task(
                tool1_name="tool_rotate", tool2_name="tool_pushpull")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 23 error: {e}')
            return False
    
    def _step_24_ur15_close_handles(self):
        """Step 24: UR15 close handles"""
        try:
            result = self.task_manager.ur15_execute_close_handles_task()
            return result
        except Exception as e:
            self.get_logger().error(f'Step 24 error: {e}')
            return False
    
    def _step_25_ur15_return_tool_pushpull(self):
        """Step 25: UR15 return tool_pushpull"""
        try:
            result = self.task_manager.ur_operate_tools.return_tool_from_task_position("tool_pushpull")
            return result
        except Exception as e:
            self.get_logger().error(f'Step 25 error: {e}')
            return False
    
    def _step_26_ur15_move_to_target(self):
        """Step 26: UR15 move to target position"""
        try:
            # First, recalculate server2base to ensure consistency
            self.get_logger().info('Recalculating server2base_matrix before moving to target...')
            server2base = self.task_manager.ur_operate_tools._calculate_server2base(
                index=self.task_manager.server_index
            )
            
            if server2base is None:
                self.get_logger().error('Failed to calculate server2base_matrix')
                return False
            
            # Execute move to target position operation
            move_result = self.task_manager.ur_operate_tools.movel_to_target_position(
                index=self.task_manager.server_index,
                execution_order=[1, 3, 2],
                offset_in_rack=[0, -0.55, 0.45]
            )
            # Note: movel_to_target_position returns 0 on success, non-zero on failure
            return not move_result
        except Exception as e:
            self.get_logger().error(f'Step 26 error: {e}')
            return False
    
    def _step_27_amr_arm_side_to_dock(self):
        """Step 27: AMR arm from side to dock"""
        try:
            result = self.task_manager.amr_controller.amr_move_arm_from_side_to_dock()
            return result and result.get('success', False)
        except Exception as e:
            self.get_logger().error(f'Step 27 error: {e}')
            return False
    
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
            
            # Setup output recipe - include all variables needed for monitor
            variables = [
                # Time
                'timestamp',
                # Joint data
                'target_q',
                'target_qd',
                'target_qdd',
                'target_current',
                'target_moment',
                'actual_q',
                'actual_qd',
                'actual_current',
                'joint_control_output',
                'joint_temperatures',
                'joint_mode',
                'actual_joint_voltage',
                # TCP data
                'actual_TCP_pose',
                'actual_TCP_speed',
                'actual_TCP_force',
                'target_TCP_pose',
                'target_TCP_speed',
                'tcp_offset',
                'tcp_force_scalar',
                # Tool data
                'actual_tool_accelerometer',
                'tool_mode',
                'tool_analog_input0',
                'tool_analog_input1',
                'tool_output_voltage',
                'tool_output_current',
                'tool_temperature',
                # Status and mode
                'robot_mode',
                'safety_mode',
                'robot_status_bits',
                'safety_status_bits',
                'speed_scaling',
                'target_speed_fraction',
                'runtime_state',
                # I/O
                'actual_digital_input_bits',
                'actual_digital_output_bits',
                'standard_analog_input0',
                'standard_analog_input1',
                'standard_analog_output0',
                'standard_analog_output1',
                'analog_io_types',
                'io_current',
                # Power and voltage
                'actual_main_voltage',
                'actual_robot_voltage',
                'actual_robot_current',
                # Elbow
                'elbow_position',
                'elbow_velocity',
                # Payload
                'payload',
                'payload_cog',
                # Other
                'actual_momentum',
                'actual_execution_time',
                'target_execution_time',
                'script_control_line',
            ]
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
        """Background thread to read all RTDE data"""
        self.get_logger().info("RTDE reader thread started")
        
        while self.rtde_thread_running:
            if not self.rtde_connected or not self.rtde_connection:
                time.sleep(0.05)
                continue
            
            try:
                # Call receive() outside the lock to avoid blocking
                state = self.rtde_connection.receive()
                
                if state is not None:
                    # Convert state object to dictionary and cache all data
                    rtde_data = {}
                    
                    # Convert state attributes to dictionary
                    for attr_name in dir(state):
                        if not attr_name.startswith('_'):
                            try:
                                attr_value = getattr(state, attr_name)
                                # Skip methods and non-serializable objects
                                if not callable(attr_value):
                                    # Convert numpy arrays to lists if needed
                                    if hasattr(attr_value, 'tolist'):
                                        rtde_data[attr_name] = attr_value.tolist()
                                    else:
                                        rtde_data[attr_name] = attr_value
                            except Exception:
                                # Skip attributes that cause errors
                                pass
                    
                    # Only lock when updating cache
                    with self.rtde_lock:
                        self.cached_rtde_data = rtde_data
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
    
    def _check_tcp_connection(self, host, port, timeout=3):
        """Check if a TCP connection can be established to host:port"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception as e:
            self.get_logger().debug(f"Connection check failed for {host}:{port} - {e}")
            return False
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.rtde_thread_running = False
        if hasattr(self, 'rtde_connection') and self.rtde_connection:
            try:
                self.rtde_connection.disconnect()
            except Exception:
                pass
        super().destroy_node()
    
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
    
    node = SystemMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
