import os
import json
import numpy as np
import requests
import time
import socket


# ═══════════════════════════════════════════════════════════════════════
# Lift Platform Controller - Standalone HTTP API Client
# ═══════════════════════════════════════════════════════════════════════
class LiftPlatformController:
    """
    Standalone controller for lift platform and pushrod via HTTP API
    Can be used independently without robot dependencies
    """
    
    def __init__(self, base_url="http://192.168.1.3:8090"):
        """
        Initialize lift platform controller
        
        Args:
            base_url: HTTP server base URL (default: http://192.168.1.3:8090)
        """
        self.lift_web_base = base_url
    
    def lift_send_command(self, target, command, **kwargs):
        """
        Send command to lift platform/pushrod via HTTP POST
        
        Args:
            target: 'platform' or 'pushrod'
            command: command name (up, down, stop, goto_height, force_up, force_down, etc.)
            **kwargs: additional command parameters (target_height, target_force, duration, etc.)
        
        Returns:
            dict with 'success' boolean and optional 'error' message
        """
        try:
            url = f"{self.lift_web_base}/api/cmd"
            payload = {
                'command': command,
                'target': target,
                **kwargs
            }
            
            # Format kwargs for display
            kwargs_str = ", ".join([f"{k}={v}" for k, v in kwargs.items()])
            if kwargs_str:
                print(f"📤 [{target}] {command} ({kwargs_str})")
            else:
                print(f"📤 [{target}] {command}")
            
            response = requests.post(url, json=payload, timeout=5)
            
            if response.status_code == 200:
                return {"success": True, "response": response.json()}
            else:
                error_msg = f"HTTP {response.status_code}: {response.text}"
                print(f"❌ Command failed: {error_msg}")
                return {"success": False, "error": error_msg}
                
        except requests.exceptions.Timeout:
            print("❌ Command request timeout")
            return {"success": False, "error": "Request timeout"}
        except Exception as e:
            print(f"❌ Command request error: {e}")
            return {"success": False, "error": str(e)}
    
    def lift_send_stop(self, target):
        """
        Send stop command to platform or pushrod (convenience wrapper)
        
        Args:
            target: 'platform' or 'pushrod'
        
        Returns:
            dict with 'success' boolean
        """
        return self.lift_send_command(target, 'stop')
    
    def lift_reset_all(self):
        """
        EMERGENCY RESET: Disable all controls and clear all relays
        This is a critical safety function that:
        1. Disables all control modes (height auto, force control)
        2. Resets all relays to 0 (platform and pushrod)
        
        Use when system is in an unknown state or needs emergency stop.
        
        Returns:
            dict with 'success' boolean
        """
        print("🔴 EMERGENCY RESET: Clearing all relays and disabling controls...")
        result = self.lift_send_command('platform', 'reset')
        if result['success']:
            print("✅ System reset complete - all relays cleared")
        else:
            print(f"❌ Reset failed: {result.get('error')}")
        return result
    
    def lift_get_status(self):
        """
        Query current status of platform and pushrod via HTTP GET
        
        Returns:
            dict with 'platform' and 'pushrod' status, or None on error
        """
        try:
            url = f"{self.lift_web_base}/api/status"
            response = requests.get(url, timeout=2)
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"❌ Status query failed: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ Status query error: {e}")
            return None
    
    def lift_wait_task_complete(self, target, timeout=90, poll_interval=0.2, expected_task_type=None):
        """
        Wait for platform or pushrod task to complete by polling status
        
        Args:
            target: 'platform' or 'pushrod'
            timeout: maximum wait time in seconds
            poll_interval: status polling interval in seconds
        
        Returns:
            dict with 'success', 'task_state', 'completion_reason', 'duration', 'task_info'
        """
        start_time = time.time()
        last_state = None
        task_started = False  # Track if we've seen our task start
        
        print(f"⏳ Waiting for [{target}] task to complete (timeout: {timeout}s)...")
        
        while time.time() - start_time < timeout:
            status = self.lift_get_status()
            
            if status and target in status:
                task_info = status[target]
                task_state = task_info.get('task_state', 'unknown')
                task_type = task_info.get('task_type', 'unknown')
                
                # Print state changes
                if task_state != last_state:
                    elapsed = time.time() - start_time
                    print(f"  [{elapsed:.1f}s] State: {task_state} | Type: {task_type}")
                    last_state = task_state
                
                # Track if we've seen our task start
                if task_state == 'running' and (expected_task_type is None or task_type == expected_task_type):
                    task_started = True
                
                # Check completion conditions
                if task_state == 'completed':
                    # Verify this is OUR task completing, not leftover from previous task
                    if expected_task_type is not None and task_type != expected_task_type:
                        # Wrong task - keep waiting
                        time.sleep(poll_interval)
                        continue
                    
                    reason = task_info.get('completion_reason', 'unknown')
                    duration = task_info.get('task_duration', 0)
                    print(f"✅ Task completed: {reason} (duration: {duration:.2f}s)")
                    return {
                        "success": True,
                        "task_state": task_state,
                        "completion_reason": reason,
                        "duration": duration,
                        "task_info": task_info
                    }
                
                elif task_state == 'emergency_stop':
                    reason = task_info.get('completion_reason', 'unknown')
                    print(f"🚨 EMERGENCY STOP: {reason}")
                    return {
                        "success": False,
                        "task_state": task_state,
                        "completion_reason": reason,
                        "error": "Emergency stop triggered",
                        "task_info": task_info
                    }
                
                elif task_state == 'idle' and time.time() - start_time > 1.0:
                    # Only accept idle if we saw our task start and complete
                    if task_started:
                        print(f"⚠️  Returned to idle state")
                        return {
                            "success": True,
                            "task_state": task_state,
                            "completion_reason": "idle",
                            "task_info": task_info
                        }
                    # Otherwise keep waiting - this might be leftover idle
            
            time.sleep(poll_interval)
        
        # Timeout
        print(f"⏱️  Timeout after {timeout}s (last state: {last_state})")
        return {
            "success": False,
            "task_state": last_state,
            "error": f"Timeout after {timeout}s"
        }
    
    def lift_platform_goto_height(self, target_height, timeout=60):
        """
        Platform goto specific height and wait for completion
        
        Args:
            target_height: target height in mm
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"🎯 [Platform] Moving to height: {target_height:.2f}mm")
        
        result = self.lift_send_command('platform', 'goto_height', target_height=target_height)
        if not result['success']:
            return result
        
        return self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='goto_height')
    
    def lift_platform_force_up(self, target_force, timeout=90):
        """
        Platform force-controlled up movement and wait for completion
        
        Args:
            target_force: target force in Newtons
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"⬆️  [Platform] Force-up to: {target_force:.1f}N")
        
        result = self.lift_send_command('platform', 'force_up', target_force=target_force)
        if not result['success']:
            return result
        
        wait_result = self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='force_up')
        
        # Get final sensor data if successful
        if wait_result['success']:
            sensor_data = self.lift_get_sensor_data()
            if sensor_data:
                height = sensor_data.get('height')
                force = sensor_data.get('combined_force_sensor')
                if height is not None and force is not None:
                    print(f"   📊 Final: height={height:.2f}mm, force={force:.1f}N")
                    wait_result['final_height'] = height
                    wait_result['final_force'] = force
        
        return wait_result
    
    def lift_platform_force_down(self, target_force, timeout=90):
        """
        Platform force-controlled down movement and wait for completion
        
        Args:
            target_force: target force in Newtons
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"⬇️  [Platform] Force-down to: {target_force:.1f}N")
        
        result = self.lift_send_command('platform', 'force_down', target_force=target_force)
        if not result['success']:
            return result
        
        wait_result = self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='force_down')
        
        # Get final sensor data if successful
        if wait_result['success']:
            sensor_data = self.lift_get_sensor_data()
            if sensor_data:
                height = sensor_data.get('height')
                force = sensor_data.get('combined_force_sensor')
                if height is not None and force is not None:
                    print(f"   📊 Final: height={height:.2f}mm, force={force:.1f}N")
                    wait_result['final_height'] = height
                    wait_result['final_force'] = force
        
        return wait_result
    
    def lift_pushrod_goto_height(self, target_height, timeout=30):
        """
        Pushrod goto specific height and wait for completion
        
        Args:
            target_height: target height in mm
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"🎯 [Pushrod] Moving to height: {target_height:.2f}mm")
        
        result = self.lift_send_command('pushrod', 'goto_height', target_height=target_height)
        if not result['success']:
            return result
        
        return self.lift_wait_task_complete('pushrod', timeout=timeout, expected_task_type='goto_height')
    
    def lift_platform_down_timed(self, duration):
        """
        Platform down for specified duration then stop and wait for completion
        
        Args:
            duration: down duration in seconds (REQUIRED)
        
        Returns:
            dict with success status
        """
        print(f"🔽 [Platform] Timed down for {duration:.1f}s")
        
        result = self.lift_send_command('platform', 'timed_down', duration=duration)
        if not result['success']:
            return result
        
        # Wait for timed operation to complete (duration + small buffer)
        wait_timeout = duration + 5.0
        wait_result = self.lift_wait_task_complete('platform', timeout=wait_timeout, expected_task_type='timed_up')
        
        if wait_result['success']:
            print(f"✅ Platform timed down completed")
        
        return wait_result
    
    def lift_pushrod_down_timed(self, duration):
        """
        Pushrod down for specified duration then stop and wait for completion
        
        Args:
            duration: down duration in seconds (REQUIRED)
        
        Returns:
            dict with success status
        """
        print(f"🔽 [Pushrod] Timed down for {duration:.1f}s")
        
        result = self.lift_send_command('pushrod', 'timed_down', duration=duration)
        if not result['success']:
            return result
        
        # Wait for timed operation to complete (duration + small buffer)
        wait_timeout = duration + 5.0
        wait_result = self.lift_wait_task_complete('pushrod', timeout=wait_timeout, expected_task_type='timed_down')
        
        if wait_result['success']:
            print(f"✅ Pushrod timed down completed")
        
        return wait_result
    
    def lift_get_sensor_data(self):
        """
        Get latest sensor data (height and force)
        
        Returns:
            dict with sensor data or None on error
        """
        try:
            url = f"{self.lift_web_base}/api/latest"
            response = requests.get(url, timeout=2)
            
            if response.status_code == 200:
                return response.json()
            else:
                return None
                
        except Exception as e:
            return None
    
    def lift_platform_down_to_base(self, timeout=30):
        """
        Platform down until auto-stop at base (832mm)
        Task will auto-complete when height < 832mm
        
        Args:
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and final height
        """
        print(f"🔽 [Platform] Descending to base (auto-stop at 832mm)")
        
        result = self.lift_send_command('platform', 'down')
        if not result['success']:
            return result
        
        # Wait for task to auto-complete
        wait_result = self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='manual_down')
        
        if wait_result['success']:
            # Get final height
            sensor_data = self.lift_get_sensor_data()
            if sensor_data and 'height' in sensor_data:
                final_height = sensor_data['height']
                print(f"   📍 Base height: {final_height:.2f}mm")
                wait_result['height'] = final_height
        
        return wait_result


# ═══════════════════════════════════════════════════════════════════════
# URExecuteBase - Robot + Lift Platform Integration (requires ur15_robot_arm)
# ═══════════════════════════════════════════════════════════════════════
try:
    from ur15_robot_arm.ur15 import UR15Robot
    UR15_AVAILABLE = True
except ImportError:
    UR15_AVAILABLE = False
    print("⚠️  Warning: ur15_robot_arm module not available - URExecuteBase will not work")


class URExecuteBase(LiftPlatformController):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, rs485_port=54321, base_url="http://192.168.1.3:8090"):
        # Initialize parent class (LiftPlatformController)
        super().__init__(base_url=base_url)
        
        # Check if robot module is available
        if not UR15_AVAILABLE:
            raise ImportError(
                "ur15_robot_arm module not available. "
                "Use LiftPlatformController instead for lift-only operations."
            )
        
        # Robot connection parameters
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot = None
        
        # RS485 connection parameters
        self.rs485_port = rs485_port
        self.rs485_socket = None
        
        # Define the path to camera parameters
        self.camera_params_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur15_cam_calibration_result",
            "ur15_camera_parameters"
        )
        
        # Data directory path (for storing collected data)
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_test_data"
        )
        
        # Result directory path (for storing results)
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_test_result"
        )
        
        # Initialize parameter storage
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.cam2end_matrix = None
        
        # Local coordinate system storage
        self.local_transformation_matrix = None
        self.local_origin = None
        
        # Reference pose storage
        self.ref_joint_angles = None
        
        # Estimated keypoints storage
        self.estimated_keypoints = None
        
        # Task position information storage
        self.task_position_offset = None

        # Initialize the robot connection
        self._initialize_robot()
        
        # Initialize RS485 socket connection
        self._init_rs485_socket()
        
        # Automatically load all parameters
        self._load_camera_intrinsic()
        self._load_camera_extrinsic()
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()
    
    def _load_camera_intrinsic(self):
        """
        Load camera intrinsic parameters from ur15_cam_calibration_result.json
        Returns: True if successful, False otherwise
        """
        intrinsic_file = os.path.join(
            self.camera_params_dir, 
            "ur15_cam_calibration_result.json"
        )
        
        try:
            with open(intrinsic_file, 'r') as f:
                data = json.load(f)
            
            if not data.get('success', False):
                print(f"Failed to load intrinsic parameters: calibration was not successful")
                return False
            
            self.camera_matrix = np.array(data['camera_matrix'])
            self.distortion_coefficients = np.array(data['distortion_coefficients'])
            
            print(f"Camera intrinsic parameters loaded successfully")
            print(f"RMS Error: {data.get('rms_error', None)}")
            return True
            
        except FileNotFoundError:
            print(f"Intrinsic parameter file not found: {intrinsic_file}")
            return False
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return False
        except Exception as e:
            print(f"Error loading intrinsic parameters: {e}")
            return False
    
    def _load_camera_extrinsic(self):
        """
        Load camera extrinsic parameters from ur15_cam_eye_in_hand_result.json
        Returns: True if successful, False otherwise
        """
        extrinsic_file = os.path.join(
            self.camera_params_dir, 
            "ur15_cam_eye_in_hand_result.json"
        )
        
        try:
            with open(extrinsic_file, 'r') as f:
                data = json.load(f)
            
            if not data.get('success', False):
                print(f"Failed to load extrinsic parameters: calibration was not successful")
                return False
            
            self.cam2end_matrix = np.array(data['cam2end_matrix'])
            
            print(f"Camera extrinsic parameters loaded successfully")
            print(f"RMS Error: {data.get('rms_error', None)}")
            return True
            
        except FileNotFoundError:
            print(f"Extrinsic parameter file not found: {extrinsic_file}")
            return False
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return False
        except Exception as e:
            print(f"Error loading extrinsic parameters: {e}")
            return False
    
    def _initialize_robot(self):
        """Initialize UR15 robot instance and establish connection"""
        try:
            print(f'Initializing UR15 robot at {self.robot_ip}:{self.robot_port}...')
            self.robot = UR15Robot(ip=self.robot_ip, port=self.robot_port)
            
            # Attempt to connect
            res = self.robot.open()
            if res == 0:
                print('✓ UR15 robot connected successfully')
            else:
                print(f'✗ Failed to connect to UR15 robot (error code: {res})')
                self.robot = None
        except Exception as e:
            print(f'Failed to initialize robot: {e}')
            self.robot = None
    
    def _init_rs485_socket(self):
        """Initialize RS485 socket connection"""
        try:
            print(f'Initializing RS485 socket connection at {self.robot_ip}:{self.rs485_port}...')
            # Open RS485 socket connection
            self.rs485_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            rs485_start_res = self.rs485_socket.connect((self.robot_ip, self.rs485_port))
            print(f"✓ RS485 socket connection result: {rs485_start_res}")
        except Exception as e:
            print(f'✗ Failed to initialize RS485 socket: {e}')
            self.rs485_socket = None
    
    def _load_estimated_kp_coordinates(self):
        """
        Load estimated keypoints coordinates from log_estimation_result.json
        Returns: List of keypoint dictionaries if successful, None otherwise
                 Each keypoint dict contains: keypoint_index, x, y, z, num_views, residual_norm
        """
        estimation_file = os.path.join(
            self.result_dir,
            "log_estimation_result.json"
        )
        
        try:
            with open(estimation_file, 'r') as f:
                data = json.load(f)
            
            if 'estimated_keypoints' not in data:
                print(f"No 'estimated_keypoints' field found in estimation result file")
                return None
            
            self.estimated_keypoints = data['estimated_keypoints']
            num_keypoints = len(self.estimated_keypoints)
            
            print(f"Estimated keypoints loaded successfully")
            print(f"Number of keypoints: {num_keypoints}")
            
            return self.estimated_keypoints
            
        except FileNotFoundError:
            print(f"Estimation result file not found: {estimation_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading estimated keypoints: {e}")
            return None
    
    def _load_local_coordinate_system(self):
        """
        Load local coordinate system transformation matrix and origin from log_local_coordinate_system_result.json
        Returns: Dictionary with 'transformation_matrix' and 'origin' if successful, None otherwise
        """
        coordinate_system_file = os.path.join(
            self.result_dir,
            "log_local_coordinate_system_result.json"
        )
        
        try:
            with open(coordinate_system_file, 'r') as f:
                data = json.load(f)
            
            if 'transformation_matrix' not in data:
                print(f"No 'transformation_matrix' field found in coordinate system result file")
                return None
            
            if 'origin' not in data:
                print(f"No 'origin' field found in coordinate system result file")
                return None
            
            self.local_transformation_matrix = np.array(data['transformation_matrix'])
            self.local_origin = data['origin']
            
            print(f"Local coordinate system loaded successfully")
            print(f"Origin: ({self.local_origin['x']:.6f}, {self.local_origin['y']:.6f}, {self.local_origin['z']:.6f})")
            
            return {
                'transformation_matrix': self.local_transformation_matrix,
                'origin': self.local_origin
            }
            
        except FileNotFoundError:
            print(f"Coordinate system result file not found: {coordinate_system_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading local coordinate system: {e}")
            return None
    
    def _load_ref_joint_angles(self):
        """
        Load reference joint angles from ref_pose.json
        Returns: List of joint angles if successful, None otherwise
        """
        ref_pose_file = os.path.join(
            self.data_dir,
            "ref_pose.json"
        )
        
        try:
            with open(ref_pose_file, 'r') as f:
                data = json.load(f)
            
            if 'joint_angles' not in data:
                print(f"No 'joint_angles' field found in reference pose file")
                return None
            
            self.ref_joint_angles = data['joint_angles']
            
            print(f"Reference joint angles loaded successfully")
            print(f"Joint angles: {self.ref_joint_angles}")
            
            return self.ref_joint_angles
            
        except FileNotFoundError:
            print(f"Reference pose file not found: {ref_pose_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading reference joint angles: {e}")
            return None
    
    def _load_task_position_information(self):
        """
        Load task position offset information from task_position_information.json
        The offsets are represented in the local coordinate system
        Returns: Dictionary with offset information if successful, None otherwise
        """
        task_position_file = os.path.join(
            self.data_dir,
            "task_position_information.json"
        )
        
        try:
            with open(task_position_file, 'r') as f:
                data = json.load(f)
            
            required_fields = ['x_offset_in_local', 'y_offset_in_local', 'z_offset_in_local']
            missing_fields = [field for field in required_fields if field not in data]
            
            if missing_fields:
                print(f"Missing fields in task position information file: {missing_fields}")
                return None
            
            self.task_position_offset = {
                'x': data['x_offset_in_local'],
                'y': data['y_offset_in_local'],
                'z': data['z_offset_in_local']
            }
            
            print(f"Task position information loaded successfully")
            print(f"Offset (in local coordinate system): x={self.task_position_offset['x']:.6f}, "
                  f"y={self.task_position_offset['y']:.6f}, "
                  f"z={self.task_position_offset['z']:.6f}")
            
            return self.task_position_offset
            
        except FileNotFoundError:
            print(f"Task position information file not found: {task_position_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading task position information: {e}")
            return None
    
    def movej_to_zero_state(self):
        """
        Move robot to zero state position
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        zero_position = [0, -1.57, 0, -1.57, 0, 0]
        print("Moving robot to zero state position...")
        
        res = self.robot.movej(zero_position, a=0.5, v=0.3)
        
        if res == 0:
            print("Robot moved to zero state successfully")
        else:
            print(f"Failed to move robot to zero state (error code: {res})")
        
        return res
    
    def movej_to_reference_joint_positions(self):
        """
        Move robot to reference joint positions
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.ref_joint_angles is None:
            print("Reference joint angles not loaded")
            return -1
        
        print("Moving robot to reference joint positions...")
        print(f"Target joint angles: {self.ref_joint_angles}")
        
        res = self.robot.movej(self.ref_joint_angles, a=0.5, v=0.3)
        
        if res == 0:
            print("Robot moved to reference position successfully")
        else:
            print(f"Failed to move robot to reference position (error code: {res})")
        
        return res
    
    def get_target_position(self):
        """
        Calculate target position in base coordinate system from offset in local coordinate system
        Uses the local-to-base transformation matrix and the task position offset
        Returns: numpy array [x, y, z] in base coordinate system if successful, None otherwise
        """
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return None
        
        if self.task_position_offset is None:
            print("Task position offset not loaded")
            return None
        
        # Create homogeneous coordinates for the offset point in local coordinate system
        offset_local = np.array([
            self.task_position_offset['x'],
            self.task_position_offset['y'],
            self.task_position_offset['z'],
            1.0  # homogeneous coordinate
        ])
        
        # Transform to base coordinate system
        # local_transformation_matrix is the local-to-base transformation (4x4)
        position_base_homogeneous = np.dot(self.local_transformation_matrix, offset_local)
        
        # Extract x, y, z from homogeneous coordinates
        position_base = position_base_homogeneous[:3]
        
        print(f"Target position calculation:")
        print(f"  Offset in local coordinate system: ({self.task_position_offset['x']:.6f}, "
              f"{self.task_position_offset['y']:.6f}, {self.task_position_offset['z']:.6f})")
        print(f"  Position in base coordinate system: ({position_base[0]:.6f}, "
              f"{position_base[1]:.6f}, {position_base[2]:.6f})")
        
        return position_base
    
    def ur_lock_quick_changer(self):
        """
        Lock the quick changer  via RS485 command
        Returns: True if successful, False otherwise
        """
        if self.rs485_socket is None:
            print("RS485 socket is not initialized")
            return False
        
        try:
            lock_command = [0x53, 0x26, 0x01, 0x01, 0x01, 0x3A, 0xD4]
            print("Sending lock command to quick changer...")
            
            self.rs485_socket.sendall(bytes(lock_command))
            time.sleep(0.5)
            
            rs485_data = self.rs485_socket.recv(1024)
            print(f"✓ RS485 Received Data: {' '.join(f'0x{b:02x}' for b in rs485_data)}")
            time.sleep(0.5)
            
            print("Quick changer locked successfully")
            return True
            
        except Exception as e:
            print(f"✗ Failed to lock quick changer: {e}")
            return False
    
    def ur_unlock_quick_changer(self):
        """
        Unlock the quick changer via RS485 command
        Returns: True if successful, False otherwise
        """
        if self.rs485_socket is None:
            print("RS485 socket is not initialized")
            return False
        
        try:
            unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
            print("Sending unlock command to quick changer...")
            
            self.rs485_socket.sendall(bytes(unlock_command))
            time.sleep(0.5)
            
            rs485_data = self.rs485_socket.recv(1024)
            print(f"✓ RS485 Received Data: {' '.join(f'0x{b:02x}' for b in rs485_data)}")
            time.sleep(0.5)
            
            print("Quick changer unlocked successfully")
            return True
            
        except Exception as e:
            print(f"✗ Failed to unlock quick changer: {e}")
            return False
    
    # ═══════════════════════════════════════════════════════════════
    # Lift Robot HTTP Control Methods (with status monitoring)
    # ═══════════════════════════════════════════════════════════════
    
    def lift_send_command(self, target, command, **kwargs):
        """
        Send command to lift platform/pushrod via HTTP POST
        
        Args:
            target: 'platform' or 'pushrod'
            command: command name (up, down, stop, goto_height, force_up, force_down, etc.)
            **kwargs: additional command parameters (target_height, target_force, duration, etc.)
        
        Returns:
            dict with 'success' boolean and optional 'error' message
        """
        try:
            url = f"{self.lift_web_base}/api/cmd"
            payload = {
                'command': command,
                'target': target,
                **kwargs
            }
            
            # Format kwargs for display
            kwargs_str = ", ".join([f"{k}={v}" for k, v in kwargs.items()])
            if kwargs_str:
                print(f"📤 [{target}] {command} ({kwargs_str})")
            else:
                print(f"📤 [{target}] {command}")
            
            response = requests.post(url, json=payload, timeout=5)
            
            if response.status_code == 200:
                return {"success": True, "response": response.json()}
            else:
                error_msg = f"HTTP {response.status_code}: {response.text}"
                print(f"❌ Command failed: {error_msg}")
                return {"success": False, "error": error_msg}
                
        except requests.exceptions.Timeout:
            print("❌ Command request timeout")
            return {"success": False, "error": "Request timeout"}
        except Exception as e:
            print(f"❌ Command request error: {e}")
            return {"success": False, "error": str(e)}
    
    def lift_send_stop(self, target):
        """
        Send stop command to platform or pushrod (convenience wrapper)
        
        Args:
            target: 'platform' or 'pushrod'
        
        Returns:
            dict with 'success' boolean
        """
        return self.lift_send_command(target, 'stop')
    
    def lift_reset_all(self):
        """
        EMERGENCY RESET: Disable all controls and clear all relays
        This is a critical safety function that:
        1. Disables all control modes (height auto, force control)
        2. Resets all relays to 0 (platform and pushrod)
        
        Use when system is in an unknown state or needs emergency stop.
        
        Returns:
            dict with 'success' boolean
        """
        print("🔴 EMERGENCY RESET: Clearing all relays and disabling controls...")
        result = self.lift_send_command('platform', 'reset')
        if result['success']:
            print("✅ System reset complete - all relays cleared")
        else:
            print(f"❌ Reset failed: {result.get('error')}")
        return result
    
    def lift_get_status(self):
        """
        Query current status of platform and pushrod via HTTP GET
        
        Returns:
            dict with 'platform' and 'pushrod' status, or None on error
        """
        try:
            url = f"{self.lift_web_base}/api/status"
            response = requests.get(url, timeout=2)
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"❌ Status query failed: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ Status query error: {e}")
            return None
    
    def lift_wait_task_complete(self, target, timeout=90, poll_interval=0.2, expected_task_type=None):
        """
        Wait for platform or pushrod task to complete by polling status
        
        Args:
            target: 'platform' or 'pushrod'
            timeout: maximum wait time in seconds
            poll_interval: status polling interval in seconds
        
        Returns:
            dict with 'success', 'task_state', 'completion_reason', 'duration', 'task_info'
        """
        start_time = time.time()
        last_state = None
        task_started = False  # Track if we've seen our task start
        
        print(f"⏳ Waiting for [{target}] task to complete (timeout: {timeout}s)...")
        
        while time.time() - start_time < timeout:
            status = self.lift_get_status()
            
            if status and target in status:
                task_info = status[target]
                task_state = task_info.get('task_state', 'unknown')
                task_type = task_info.get('task_type', 'unknown')
                
                # Print state changes
                if task_state != last_state:
                    elapsed = time.time() - start_time
                    print(f"  [{elapsed:.1f}s] State: {task_state} | Type: {task_type}")
                    last_state = task_state
                
                # Track if we've seen our task start
                if task_state == 'running' and (expected_task_type is None or task_type == expected_task_type):
                    task_started = True
                
                # Check completion conditions
                if task_state == 'completed':
                    # Verify this is OUR task completing, not leftover from previous task
                    if expected_task_type is not None and task_type != expected_task_type:
                        # Wrong task - keep waiting
                        time.sleep(poll_interval)
                        continue
                    
                    reason = task_info.get('completion_reason', 'unknown')
                    duration = task_info.get('task_duration', 0)
                    print(f"✅ Task completed: {reason} (duration: {duration:.2f}s)")
                    return {
                        "success": True,
                        "task_state": task_state,
                        "completion_reason": reason,
                        "duration": duration,
                        "task_info": task_info
                    }
                
                elif task_state == 'emergency_stop':
                    reason = task_info.get('completion_reason', 'unknown')
                    print(f"🚨 EMERGENCY STOP: {reason}")
                    return {
                        "success": False,
                        "task_state": task_state,
                        "completion_reason": reason,
                        "error": "Emergency stop triggered",
                        "task_info": task_info
                    }
                
                elif task_state == 'idle' and time.time() - start_time > 1.0:
                    # Only accept idle if we saw our task start and complete
                    if task_started:
                        print(f"⚠️  Returned to idle state")
                        return {
                            "success": True,
                            "task_state": task_state,
                            "completion_reason": "idle",
                            "task_info": task_info
                        }
                    # Otherwise keep waiting - this might be leftover idle
            
            time.sleep(poll_interval)
        
        # Timeout
        print(f"⏱️  Timeout after {timeout}s (last state: {last_state})")
        return {
            "success": False,
            "task_state": last_state,
            "error": f"Timeout after {timeout}s"
        }
    
    def lift_platform_goto_height(self, target_height, timeout=60):
        """
        Platform goto specific height and wait for completion
        
        Args:
            target_height: target height in mm
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"🎯 [Platform] Moving to height: {target_height:.2f}mm")
        
        result = self.lift_send_command('platform', 'goto_height', target_height=target_height)
        if not result['success']:
            return result
        
        return self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='goto_height')
    
    def lift_platform_force_up(self, target_force, timeout=90):
        """
        Platform force-controlled up movement and wait for completion
        
        Args:
            target_force: target force in Newtons
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"⬆️  [Platform] Force-up to: {target_force:.1f}N")
        
        result = self.lift_send_command('platform', 'force_up', target_force=target_force)
        if not result['success']:
            return result
        
        wait_result = self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='force_up')
        
        # Get final sensor data if successful
        if wait_result['success']:
            sensor_data = self.lift_get_sensor_data()
            if sensor_data:
                height = sensor_data.get('height')
                force = sensor_data.get('combined_force_sensor')
                if height is not None and force is not None:
                    print(f"   📊 Final: height={height:.2f}mm, force={force:.1f}N")
                    wait_result['final_height'] = height
                    wait_result['final_force'] = force
        
        return wait_result
    
    def lift_platform_force_down(self, target_force, timeout=90):
        """
        Platform force-controlled down movement and wait for completion
        
        Args:
            target_force: target force in Newtons
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"⬇️  [Platform] Force-down to: {target_force:.1f}N")
        
        result = self.lift_send_command('platform', 'force_down', target_force=target_force)
        if not result['success']:
            return result
        
        wait_result = self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='force_down')
        
        # Get final sensor data if successful
        if wait_result['success']:
            sensor_data = self.lift_get_sensor_data()
            if sensor_data:
                height = sensor_data.get('height')
                force = sensor_data.get('combined_force_sensor')
                if height is not None and force is not None:
                    print(f"   📊 Final: height={height:.2f}mm, force={force:.1f}N")
                    wait_result['final_height'] = height
                    wait_result['final_force'] = force
        
        return wait_result
    
    def lift_pushrod_goto_height(self, target_height, timeout=30):
        """
        Pushrod goto specific height and wait for completion
        
        Args:
            target_height: target height in mm
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and task info
        """
        print(f"🎯 [Pushrod] Moving to height: {target_height:.2f}mm")
        
        result = self.lift_send_command('pushrod', 'goto_height', target_height=target_height)
        if not result['success']:
            return result
        
        return self.lift_wait_task_complete('pushrod', timeout=timeout, expected_task_type='goto_height')
    
    def lift_platform_down_timed(self, duration):
        """
        Platform down for specified duration then stop and wait for completion
        
        Args:
            duration: down duration in seconds (REQUIRED)
        
        Returns:
            dict with success status
        """
        print(f"🔽 [Platform] Timed down for {duration:.1f}s")
        
        result = self.lift_send_command('platform', 'timed_down', duration=duration)
        if not result['success']:
            return result
        
        # Wait for timed operation to complete (duration + small buffer)
        wait_timeout = duration + 5.0
        wait_result = self.lift_wait_task_complete('platform', timeout=wait_timeout, expected_task_type='timed_up')
        
        if wait_result['success']:
            print(f"✅ Platform timed down completed")
        
        return wait_result
    
    def lift_pushrod_down_timed(self, duration):
        """
        Pushrod down for specified duration then stop and wait for completion
        
        Args:
            duration: down duration in seconds (REQUIRED)
        
        Returns:
            dict with success status
        """
        print(f"🔽 [Pushrod] Timed down for {duration:.1f}s")
        
        result = self.lift_send_command('pushrod', 'timed_down', duration=duration)
        if not result['success']:
            return result
        
        # Wait for timed operation to complete (duration + small buffer)
        wait_timeout = duration + 5.0
        wait_result = self.lift_wait_task_complete('pushrod', timeout=wait_timeout, expected_task_type='timed_down')
        
        if wait_result['success']:
            print(f"✅ Pushrod timed down completed")
        
        return wait_result
    
    def lift_get_sensor_data(self):
        """
        Get latest sensor data (height and force)
        
        Returns:
            dict with sensor data or None on error
        """
        try:
            url = f"{self.lift_web_base}/api/latest"
            response = requests.get(url, timeout=2)
            
            if response.status_code == 200:
                return response.json()
            else:
                return None
                
        except Exception as e:
            return None
    
    def lift_platform_down_to_base(self, timeout=30):
        """
        Platform down until auto-stop at base (832mm)
        Task will auto-complete when height < 832mm
        
        Args:
            timeout: maximum wait time in seconds
        
        Returns:
            dict with success status and final height
        """
        print(f"🔽 [Platform] Descending to base (auto-stop at 832mm)")
        
        result = self.lift_send_command('platform', 'down')
        if not result['success']:
            return result
        
        # Wait for task to auto-complete
        wait_result = self.lift_wait_task_complete('platform', timeout=timeout, expected_task_type='manual_down')
        
        if wait_result['success']:
            # Get final height
            sensor_data = self.lift_get_sensor_data()
            if sensor_data and 'height' in sensor_data:
                final_height = sensor_data['height']
                print(f"   � Base height: {final_height:.2f}mm")
                wait_result['height'] = final_height
        
        return wait_result


if __name__ == "__main__":
    # Example usage
    ur_base = URExecuteBase()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    if ur_base.camera_matrix is not None:
        print("\nCamera Matrix:")
        print(ur_base.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_base.distortion_coefficients)
    
    if ur_base.cam2end_matrix is not None:
        print("\nCamera to End-effector Matrix:")
        print(ur_base.cam2end_matrix)
    
    if ur_base.estimated_keypoints is not None:
        print("\nEstimated Keypoints:")
        for kp in ur_base.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: ({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f})")
    
    if ur_base.local_transformation_matrix is not None:
        print("\nLocal Coordinate System Transformation Matrix:")
        print(ur_base.local_transformation_matrix)
    
    if ur_base.ref_joint_angles is not None:
        print("\nReference Joint Angles (radians):")
        for i, angle in enumerate(ur_base.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f}")
    
    if ur_base.task_position_offset is not None:
        print("\nTask Position Offset (in local coordinate system):")
        print(f"x: {ur_base.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_base.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_base.task_position_offset.get('z', 0):.6f}")
    
    # Calculate target position in base coordinate system
    target_position = ur_base.get_target_position()
    if target_position is not None:
        print("\nTarget Position in Base Coordinate System:")
        print(target_position)