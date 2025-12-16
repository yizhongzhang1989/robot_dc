#!/usr/bin/env python3
"""
Courier Robot Controller
Provides HTTP API interface for lift platform and pushrod control
All functions correspond to web interface controls
"""

import requests
import time
import threading
import yaml
from pathlib import Path


class CourierRobotWebAPI:
    """
    Courier robot controller for lift platform and pushrod via HTTP API
    Provides all control functions available in the web interface
    """
    
    @staticmethod
    def _load_config_url():
        """
        Load base URL from robot_config.yaml
        
        Returns:
            str: Base URL from config, or default if not found
        """
        try:
            # Try to find config file
            script_dir = Path(__file__).parent
            config_path = script_dir.parent / 'config' / 'robot_config.yaml'
            
            if not config_path.exists():
                return None
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract lift_robot.web configuration
            web_config = config.get('lift_robot', {}).get('web', {})
            host = web_config.get('host', 'localhost')
            port = web_config.get('port', 8090)
            
            # Build URL
            return f"http://{host}:{port}"
        except Exception as e:
            # Silently fail and return None if config cannot be loaded
            return None
    
    def __init__(self, base_url=None, verbose=True):
        """
        Initialize courier robot controller
        
        Args:
            base_url: HTTP server base URL (default: read from config/robot_config.yaml, 
                     fallback to http://192.168.1.3:8090 if config not found)
                     Can override by passing explicit URL
            verbose: If True, automatically print command results (default: True)
        """
        # Determine base URL priority: explicit parameter > config file > hardcoded default
        if base_url is None:
            base_url = self._load_config_url()
            if base_url is None:
                base_url = "http://192.168.1.3:8090"  # Hardcoded fallback
                if verbose:
                    print(f"⚠️  Config file not found, using default URL: {base_url}")
            elif verbose:
                print(f"📄 Loaded URL from config: {base_url}")
        
        self.base_url = base_url
        self.verbose = verbose
        self.background_task = None  # Track background task thread
        self.background_task_lock = threading.Lock()
        self.reset_flag = False  # Flag to signal background tasks to abort
        if verbose:
            print(f"📡 CourierRobot initialized with base URL: {base_url}")
    
    def _send_command(self, target, command, **kwargs):
        """
        Internal method to send HTTP command to lift platform/pushrod
        
        Args:
            target: 'platform' or 'pushrod'
            command: command name
            **kwargs: additional parameters
        
        Returns:
            dict with 'success' boolean and optional data
        """
        try:
            url = f"{self.base_url}/api/cmd"
            payload = {'command': command, 'target': target, **kwargs}
            
            response = requests.post(url, json=payload, timeout=10)
            
            if response.status_code == 200:
                return {"success": True, "data": response.json()}
            else:
                return {
                    "success": False,
                    "error": f"HTTP {response.status_code}: {response.text}"
                }
        except requests.exceptions.Timeout:
            return {"success": False, "error": "Request timeout"}
        except requests.exceptions.ConnectionError:
            return {"success": False, "error": "Connection failed"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _get_status(self):
        """
        Internal method: Get current status of platform and pushrod with sensor data
        Always returns data without printing
        
        Returns:
            dict with complete system status including:
            - platform: essential fields (task_state, movement_state, control_mode)
            - pushrod: essential fields (task_state, movement_state)
            - sensors: height, force, frequency, etc.
            - _full_data: complete raw data for internal use
        """
        try:
            # Get task status
            status_url = f"{self.base_url}/api/status"
            status_response = requests.get(status_url, timeout=2)
            
            # Get sensor data
            sensor_url = f"{self.base_url}/api/latest"
            sensor_response = requests.get(sensor_url, timeout=2)
            
            if status_response.status_code == 200:
                status_data = status_response.json()
                
                # Extract only essential status fields for external API users
                result = {
                    "success": True,
                    "platform": {
                        'task_state': status_data.get('platform', {}).get('task_state'),
                        'movement_state': status_data.get('platform', {}).get('movement_state'),
                        'control_mode': status_data.get('platform', {}).get('control_mode'),
                        'max_force_limit': status_data.get('platform', {}).get('max_force_limit', 0.0)
                    },
                    "pushrod": {
                        'task_state': status_data.get('pushrod', {}).get('task_state'),
                        'movement_state': status_data.get('pushrod', {}).get('movement_state')
                    },
                    "server_id": status_data.get('server_id', 0),
                    "hybrid_params": status_data.get('hybrid_params', {}),
                    "hybrid_positions": status_data.get('hybrid_positions', {})
                }
                
                # Calculate force_limit_status (3 states: 'ok', 'exceeded', 'disabled')
                platform_data = status_data.get('platform', {})
                force_sensor_error = platform_data.get('force_sensor_error', False)
                force_limit_exceeded = platform_data.get('force_limit_exceeded', False)
                
                if force_sensor_error:
                    result['platform']['force_limit_status'] = 'disabled'
                    result['platform']['force_limit_message'] = platform_data.get('force_sensor_error_message', 'Sensor error')
                elif force_limit_exceeded:
                    result['platform']['force_limit_status'] = 'exceeded'
                    result['platform']['force_limit_message'] = 'Force limit exceeded'
                else:
                    result['platform']['force_limit_status'] = 'ok'
                    result['platform']['force_limit_message'] = 'Force within limits'
                
                # Keep full data available for internal use (verbose printing)
                result['_full_data'] = status_data
                
                # Merge sensor data if available
                if sensor_response.status_code == 200:
                    sensor_data = sensor_response.json()
                    result['sensors'] = {
                        'height': sensor_data.get('height'),
                        'right_force': sensor_data.get('right_force_sensor'),
                        'left_force': sensor_data.get('left_force_sensor'),
                        'combined_force': sensor_data.get('combined_force_sensor'),
                        'freq_hz': sensor_data.get('freq_hz'),
                        'right_force_freq_hz': sensor_data.get('right_force_freq_hz'),
                        'left_force_freq_hz': sensor_data.get('left_force_freq_hz'),
                        # Error information
                        'height_sensor_error': sensor_data.get('sensor_error', False),
                        'height_sensor_error_message': sensor_data.get('sensor_error_message'),
                        'right_force_error': sensor_data.get('right_force_error', False),
                        'right_force_error_message': sensor_data.get('right_force_error_message'),
                        'left_force_error': sensor_data.get('left_force_error', False),
                        'left_force_error_message': sensor_data.get('left_force_error_message')
                    }
                else:
                    result['sensors'] = {}
                
                return result
            return {"success": False, "error": f"HTTP {status_response.status_code}"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_status(self):
        """
        Public API: Get current system status (pure data retrieval, no printing)
        
        Returns:
            dict with simplified status for external API users:
            - success: bool
            - platform: {task_state, movement_state, control_mode}
            - pushrod: {task_state, movement_state}
            - sensors: {height, forces, frequencies}
        """
        result = self._get_status()
        
        # Remove _full_data from public API response
        if '_full_data' in result:
            del result['_full_data']
        
        return result
    
    def get_sensor_data(self):
        """
        Get latest sensor data (height and force)
        
        Returns:
            dict with success and sensor data
        """
        try:
            url = f"{self.base_url}/api/latest"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                result = {"success": True, "data": response.json()}
                
                # Print friendly sensor data if verbose
                if self.verbose:
                    data = result['data']
                    print("\n📡 Sensor Data:")
                    if data.get('height') is not None:
                        print(f"   Height: {data['height']:.2f} mm")
                    if data.get('right_force_sensor') is not None:
                        freq_r = data.get('right_force_freq_hz', 0)
                        print(f"   Right Force: {data['right_force_sensor']:.2f} N ({freq_r:.1f} Hz)")
                    if data.get('left_force_sensor') is not None:
                        freq_l = data.get('left_force_freq_hz', 0)
                        print(f"   Left Force: {data['left_force_sensor']:.2f} N ({freq_l:.1f} Hz)")
                    if data.get('combined_force_sensor') is not None:
                        print(f"   Combined Force: {data['combined_force_sensor']:.2f} N")
                    if data.get('freq_hz') is not None:
                        print(f"   Height Sensor Freq: {data['freq_hz']:.1f} Hz\n")
                
                return result
            return {"success": False, "error": f"HTTP {response.status_code}"}
        except Exception as e:
            if self.verbose:
                print(f"❌ Failed to get sensor data: {e}")
            return {"success": False, "error": str(e)}
    
    # ==================== Server Configuration ====================
    
    def get_server_config(self):
        """
        Get server ID and hybrid control configuration
        
        Returns:
            dict with success, server_id, hybrid_params, and calculated hybrid_positions
        """
        try:
            url = f"{self.base_url}/api/server_config"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                data = response.json()
                result = {"success": True, **data}
                
                if self.verbose:
                    print("\n⚙️  Server Configuration:")
                    print(f"   Server ID: {data['server_id']}")
                    print(f"   Hybrid Params:")
                    print(f"      High Base: {data['hybrid_params']['high_base']:.1f} mm")
                    print(f"      Low Base: {data['hybrid_params']['low_base']:.1f} mm")
                    print(f"      Step: {data['hybrid_params']['step']:.1f} mm/ID")
                    print(f"   Calculated Positions:")
                    print(f"      High Position: {data['hybrid_positions']['high_pos']:.1f} mm")
                    print(f"      Low Position: {data['hybrid_positions']['low_pos']:.1f} mm\n")
                
                return result
            return {"success": False, "error": f"HTTP {response.status_code}"}
        except Exception as e:
            if self.verbose:
                print(f"❌ Failed to get server config: {e}")
            return {"success": False, "error": str(e)}
    
    def set_server_id(self, server_id):
        """
        Set server ID (runtime only, does not persist to config)
        
        Args:
            server_id: Integer server ID
        
        Returns:
            dict with success status and updated configuration
        """
        try:
            url = f"{self.base_url}/api/server_config/set_server_id"
            payload = {'server_id': server_id}
            response = requests.post(url, json=payload, timeout=2)
            
            if response.status_code == 200:
                data = response.json()
                result = {"success": True, **data}
                
                if self.verbose:
                    print(f"\n✅ Server ID updated to: {data['server_id']}")
                    print(f"   New High Position: {data['hybrid_positions']['high_pos']:.1f} mm")
                    print(f"   New Low Position: {data['hybrid_positions']['low_pos']:.1f} mm\n")
                
                return result
            return {"success": False, "error": f"HTTP {response.status_code}"}
        except Exception as e:
            if self.verbose:
                print(f"❌ Failed to set server ID: {e}")
            return {"success": False, "error": str(e)}
    
    def set_hybrid_params(self, high_base=None, low_base=None, step=None):
        """
        Set hybrid control parameters (runtime only, does not persist to config)
        
        Args:
            high_base: Base height for high position (mm), optional
            low_base: Base height for low position (mm), optional
            step: Step distance per ID (mm), optional
        
        Returns:
            dict with success status and updated configuration
        """
        try:
            url = f"{self.base_url}/api/server_config/set_hybrid_params"
            payload = {}
            if high_base is not None:
                payload['high_base'] = high_base
            if low_base is not None:
                payload['low_base'] = low_base
            if step is not None:
                payload['step'] = step
            
            response = requests.post(url, json=payload, timeout=2)
            
            if response.status_code == 200:
                data = response.json()
                result = {"success": True, **data}
                
                if self.verbose:
                    print(f"\n✅ Hybrid parameters updated:")
                    params = data['hybrid_params']
                    print(f"   High Base: {params['high_base']:.1f} mm")
                    print(f"   Low Base: {params['low_base']:.1f} mm")
                    print(f"   Step: {params['step']:.1f} mm/ID")
                    print(f"   Calculated Positions:")
                    print(f"      High Position: {data['hybrid_positions']['high_pos']:.1f} mm")
                    print(f"      Low Position: {data['hybrid_positions']['low_pos']:.1f} mm\n")
                
                return result
            return {"success": False, "error": f"HTTP {response.status_code}"}
        except Exception as e:
            if self.verbose:
                print(f"❌ Failed to set hybrid params: {e}")
            return {"success": False, "error": str(e)}
    
    # ==================== Platform Manual Control ====================
    
    def platform_up(self):
        """
        Platform manual up movement
        
        Returns:
            dict with success status and complete state
        """
        # Check if platform is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            platform_state = status.get('platform', {}).get('task_state', 'unknown')
            if platform_state not in ['idle', 'completed']:
                if self.verbose:
                    print(f"❌ Platform is busy (state: {platform_state}), command rejected")
                return {
                    'success': False,
                    'error': f'Platform is busy (state: {platform_state})',
                    'status': status
                }
        
        if self.verbose:
            print(f"⬆️  [Platform] Manual UP")
        result = self._send_command('platform', 'up')
        
        # Get final status after command
        final_status = self.get_status()
        result['final_status'] = final_status
        
        if self.verbose:
            if result['success']:
                print(f"✅ Platform UP command sent successfully")
            else:
                print(f"❌ Failed: {result.get('error')}")
        return result
    
    def platform_down(self):
        """
        Platform manual down movement
        
        Returns:
            dict with success status and complete state
        """
        # Check if platform is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            platform_state = status.get('platform', {}).get('task_state', 'unknown')
            if platform_state not in ['idle', 'completed']:
                if self.verbose:
                    print(f"❌ Platform is busy (state: {platform_state}), command rejected")
                return {
                    'success': False,
                    'error': f'Platform is busy (state: {platform_state})',
                    'status': status
                }
        
        if self.verbose:
            print(f"⬇️  [Platform] Manual DOWN")
        result = self._send_command('platform', 'down')
        
        # Get final status after command
        final_status = self.get_status()
        result['final_status'] = final_status
        
        if self.verbose:
            if result['success']:
                print(f"✅ Platform DOWN command sent successfully")
            else:
                print(f"❌ Failed: {result.get('error')}")
        return result
    
    def platform_stop(self):
        """
        Platform stop - ALWAYS allowed, can interrupt any command including blocking ones
        
        Returns:
            dict with success status and complete state
        """
        if self.verbose:
            print(f"🛑 [Platform] STOP")
        result = self._send_command('platform', 'stop')
        
        # Get final status after command
        final_status = self.get_status()
        result['final_status'] = final_status
        
        if self.verbose:
            print(f"✅ Platform STOP command sent")
        return result
    
    # ==================== Platform Height Control ====================
    
    def platform_goto_height(self, target_height, wait=True, timeout=300):
        """
        Platform goto specific height (BLOCKING by default)
        
        Args:
            target_height: Target height in mm
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 300)
            
            Returns:
            dict with success status and complete state
        """
        # Check if platform is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            platform_state = status.get('platform', {}).get('task_state', 'unknown')
            if platform_state not in ['idle', 'completed']:
                if wait:
                    # Blocking mode: reject if busy
                    if self.verbose:
                        print(f"❌ Platform is busy (state: {platform_state}), command rejected")
                    return {
                        'success': False,
                        'error': f'Platform is busy (state: {platform_state})',
                        'status': status
                    }
                else:
                    # Non-blocking mode: auto-stop previous task
                    if self.verbose:
                        print(f"⚠️  Platform is busy (state: {platform_state}), auto-stopping...")
                    self.platform_stop()
        
        if self.verbose:
            print(f"🎯 [Platform] Goto height: {target_height}mm")
        result = self._send_command('platform', 'goto_height', target_height=target_height)
        
        if not result['success']:
            if self.verbose:
                print(f"❌ Failed: {result.get('error')}")
            return result
        
        # Wait for completion if requested
        if wait:
            if self.verbose:
                print(f"⏳ Waiting for completion (timeout: {timeout}s)...")
            completion = self._wait_for_completion(target='platform', timeout=timeout)
            
            # Only print completion message if successful (not aborted by reset)
            if completion['success'] and self.verbose:
                print(f"✅ Goto {target_height}mm completed")
            elif self.verbose and completion.get('error') != 'Aborted by reset':
                # Print error only if it's not a reset abort (reset handles its own messages)
                print(f"❌ Task failed: {completion.get('error', 'unknown')}")
            
            return completion
        else:
            # Non-blocking mode
            final_status = self.get_status()
            result['final_status'] = final_status
            if self.verbose:
                print(f"✅ Goto height command sent (non-blocking)")
            return result
    
    # ==================== Platform Force Control ====================
    
    def platform_force_up(self, target_force, wait=True, timeout=300):
        """
        Platform force-controlled up movement (BLOCKING by default)
        
        Args:
            target_force: Target force in Newtons
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 300)
            
        Returns:
            dict with success status and complete state
        """
        # Check if platform is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            platform_state = status.get('platform', {}).get('task_state', 'unknown')
            if platform_state not in ['idle', 'completed']:
                if wait:
                    # Blocking mode: reject if busy
                    if self.verbose:
                        print(f"❌ Platform is busy (state: {platform_state}), command rejected")
                    return {
                        'success': False,
                        'error': f'Platform is busy (state: {platform_state})',
                        'status': status
                    }
                else:
                    # Non-blocking mode: auto-stop previous task
                    if self.verbose:
                        print(f"⚠️  Platform is busy (state: {platform_state}), auto-stopping...")
                    self.platform_stop()
        
        if self.verbose:
            print(f"⚡⬆️  [Platform] Force UP to {target_force}N")
        result = self._send_command('platform', 'force_up', target_force=target_force)
        
        if not result['success']:
            if self.verbose:
                print(f"❌ Failed: {result.get('error')}")
            return result
        
        # Wait for completion if requested
        if wait:
            if self.verbose:
                print(f"⏳ Waiting for completion (timeout: {timeout}s)...")
            completion = self._wait_for_completion(target='platform', timeout=timeout)
            
            # Only print completion message if successful (not aborted by reset)
            if completion['success'] and self.verbose:
                print(f"✅ Force UP {target_force}N completed")
            elif self.verbose and completion.get('error') != 'Aborted by reset':
                print(f"❌ Task failed: {completion.get('error', 'unknown')}")
            
            return completion
        else:
            final_status = self.get_status()
            result['final_status'] = final_status
            if self.verbose:
                print(f"✅ Force UP command sent (non-blocking)")
            return result
    
    def platform_force_down(self, target_force, wait=True, timeout=300):
        """
        Platform force-controlled down movement (BLOCKING by default)
        
        Args:
            target_force: Target force in Newtons
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 300)
            
        Returns:
            dict with success status and complete state
        """
        # Check if platform is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            platform_state = status.get('platform', {}).get('task_state', 'unknown')
            if platform_state not in ['idle', 'completed']:
                if wait:
                    # Blocking mode: reject if busy
                    if self.verbose:
                        print(f"❌ Platform is busy (state: {platform_state}), command rejected")
                    return {
                        'success': False,
                        'error': f'Platform is busy (state: {platform_state})',
                        'status': status
                    }
                else:
                    # Non-blocking mode: auto-stop previous task
                    if self.verbose:
                        print(f"⚠️  Platform is busy (state: {platform_state}), auto-stopping...")
                    self.platform_stop()
        
        if self.verbose:
            print(f"⚡⬇️  [Platform] Force DOWN to {target_force}N")
        result = self._send_command('platform', 'force_down', target_force=target_force)
        
        if not result['success']:
            if self.verbose:
                print(f"❌ Failed: {result.get('error')}")
            return result
        
        # Wait for completion if requested
        if wait:
            if self.verbose:
                print(f"⏳ Waiting for completion (timeout: {timeout}s)...")
            completion = self._wait_for_completion(target='platform', timeout=timeout)
            
            # Only print completion message if successful (not aborted by reset)
            if completion['success'] and self.verbose:
                print(f"✅ Force DOWN {target_force}N completed")
            elif self.verbose and completion.get('error') != 'Aborted by reset':
                print(f"❌ Task failed: {completion.get('error', 'unknown')}")
            
            return completion
        else:
            final_status = self.get_status()
            result['final_status'] = final_status
            if self.verbose:
                print(f"✅ Force DOWN command sent (non-blocking)")
            return result
    
    # ==================== Platform Hybrid Control ====================
    
    def platform_hybrid_control(self, target_height, target_force, wait=True, timeout=300):
        """
        Platform hybrid control (height OR force, whichever reached first) (BLOCKING by default)
        
        Args:
            target_height: Target height in mm, or 'high_pos'/'low_pos' to use calculated hybrid positions
            target_force: Target force in Newtons
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 300)
            
        Returns:
            dict with success status and complete state
        """
        # Resolve special height values (high_pos/low_pos)
        if isinstance(target_height, str):
            if target_height.lower() in ['high_pos', 'high']:
                config = self.get_server_config()
                if config.get('success'):
                    target_height = config['hybrid_positions']['high_pos']
                    if self.verbose:
                        print(f"🔄 Resolved 'high_pos' to {target_height:.1f} mm")
                else:
                    return {'success': False, 'error': 'Failed to get server config for high_pos'}
            elif target_height.lower() in ['low_pos', 'low']:
                config = self.get_server_config()
                if config.get('success'):
                    target_height = config['hybrid_positions']['low_pos']
                    if self.verbose:
                        print(f"🔄 Resolved 'low_pos' to {target_height:.1f} mm")
                else:
                    return {'success': False, 'error': 'Failed to get server config for low_pos'}
            else:
                return {'success': False, 'error': f"Invalid height value: {target_height}. Use numeric value or 'high_pos'/'low_pos'"}
        
        # Check if platform is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            platform_state = status.get('platform', {}).get('task_state', 'unknown')
            if platform_state not in ['idle', 'completed']:
                if wait:
                    # Blocking mode: reject if busy
                    if self.verbose:
                        print(f"❌ Platform is busy (state: {platform_state}), command rejected")
                    return {
                        'success': False,
                        'error': f'Platform is busy (state: {platform_state})',
                        'status': status
                    }
                else:
                    # Non-blocking mode: auto-stop previous task
                    if self.verbose:
                        print(f"⚠️  Platform is busy (state: {platform_state}), auto-stopping...")
                    self.platform_stop()
        
        if self.verbose:
            print(f"🎯⚡ [Platform] Hybrid: {target_height}mm OR {target_force}N")
        result = self._send_command('platform', 'height_force_hybrid', 
                                target_height=target_height, target_force=target_force)
        
        if not result['success']:
            if self.verbose:
                print(f"❌ Failed: {result.get('error')}")
            return result
        
        # Wait for completion if requested
        if wait:
            if self.verbose:
                print(f"⏳ Waiting for completion (timeout: {timeout}s)...")
            completion = self._wait_for_completion(target='platform', timeout=timeout)
            
            # Only print completion message if successful (not aborted by reset)
            if completion['success'] and self.verbose:
                print(f"✅ Hybrid control completed")
            elif self.verbose and completion.get('error') != 'Aborted by reset':
                print(f"❌ Task failed: {completion.get('error', 'unknown')}")
            
            return completion
        else:
            final_status = self.get_status()
            result['final_status'] = final_status
            if self.verbose:
                print(f"✅ Hybrid control command sent (non-blocking)")
            return result
    
    # ==================== Pushrod Manual Control ====================
    
    def pushrod_up(self):
        """
        Pushrod manual up movement
        
        Returns:
            dict with success status and complete state
        """
        # Check if pushrod is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            pushrod_state = status.get('pushrod', {}).get('task_state', 'unknown')
            if pushrod_state not in ['idle', 'completed']:
                if self.verbose:
                    print(f"❌ Pushrod is busy (state: {pushrod_state}), command rejected")
                return {
                    'success': False,
                    'error': f'Pushrod is busy (state: {pushrod_state})',
                    'status': status
                }
        
        if self.verbose:
            print(f"⬆️  [Pushrod] Manual UP")
        result = self._send_command('pushrod', 'up')
        
        # Get final status after command
        final_status = self.get_status()
        result['final_status'] = final_status
        
        if self.verbose:
            if result['success']:
                print(f"✅ Pushrod UP command sent successfully")
            else:
                print(f"❌ Failed: {result.get('error')}")
        return result
    
    def pushrod_down(self):
        """
        Pushrod manual down movement
        
        Returns:
            dict with success status and complete state
        """
        # Check if pushrod is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            pushrod_state = status.get('pushrod', {}).get('task_state', 'unknown')
            if pushrod_state not in ['idle', 'completed']:
                if self.verbose:
                    print(f"❌ Pushrod is busy (state: {pushrod_state}), command rejected")
                return {
                    'success': False,
                    'error': f'Pushrod is busy (state: {pushrod_state})',
                    'status': status
                }
        
        if self.verbose:
            print(f"⬇️  [Pushrod] Manual DOWN")
        result = self._send_command('pushrod', 'down')
        
        # Get final status after command
        final_status = self.get_status()
        result['final_status'] = final_status
        
        if self.verbose:
            if result['success']:
                print(f"✅ Pushrod DOWN command sent successfully")
            else:
                print(f"❌ Failed: {result.get('error')}")
        return result
    
    def pushrod_stop(self):
        """
        Pushrod stop - ALWAYS allowed, can interrupt any command including blocking ones
        
        Returns:
            dict with success status and complete state
        """
        if self.verbose:
            print(f"🛑 [Pushrod] STOP")
        result = self._send_command('pushrod', 'stop')
        
        # Get final status after command
        final_status = self.get_status()
        result['final_status'] = final_status
        
        if self.verbose:
            print(f"✅ Pushrod STOP command sent")
        return result
    
    # ==================== Pushrod Height Control ====================
    
    def pushrod_goto_height(self, target_height, mode='absolute', wait=True, timeout=300):
        """
        Pushrod goto specific height (BLOCKING by default)
        
        Args:
            target_height: Target height in mm (absolute) or offset in mm (relative)
            mode: 'absolute' or 'relative' (default: 'absolute')
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 300)
            
        Returns:
            dict with success status and complete state
        """
        # Check if pushrod is idle or completed before sending command
        status = self._get_status()
        if status.get('success'):
            pushrod_state = status.get('pushrod', {}).get('task_state', 'unknown')
            if pushrod_state not in ['idle', 'completed']:
                if wait:
                    # Blocking mode: reject if busy
                    if self.verbose:
                        print(f"❌ Pushrod is busy (state: {pushrod_state}), command rejected")
                    return {
                        'success': False,
                        'error': f'Pushrod is busy (state: {pushrod_state})',
                        'status': status
                    }
                else:
                    # Non-blocking mode: auto-stop previous task
                    if self.verbose:
                        print(f"⚠️  Pushrod is busy (state: {pushrod_state}), auto-stopping...")
                    self.pushrod_stop()
        
        if self.verbose:
            print(f"🎯 [Pushrod] Goto height: {target_height}mm (mode: {mode})")
        result = self._send_command('pushrod', 'goto_height', 
                                 target_height=target_height, 
                                 mode=mode)
        
        if not result['success']:
            if self.verbose:
                print(f"❌ Failed: {result.get('error')}")
            return result
        
        # Wait for completion if requested
        if wait:
            if self.verbose:
                print(f"⏳ Waiting for completion (timeout: {timeout}s)...")
            completion = self._wait_for_completion(target='pushrod', timeout=timeout)
            
            # Only print completion message if successful (not aborted by reset)
            if completion['success'] and self.verbose:
                print(f"✅ Pushrod goto {target_height}mm completed")
            elif self.verbose and completion.get('error') != 'Aborted by reset':
                print(f"❌ Task failed: {completion.get('error', 'unknown')}")
            
            return completion
        else:
            final_status = self.get_status()
            result['final_status'] = final_status
            if self.verbose:
                print(f"✅ Goto height command sent (non-blocking)")
            return result
    
    # ==================== Emergency Reset ====================
    
    def emergency_reset(self):
        """
        Emergency reset - ALWAYS allowed, stops all movements and clears all states
        Can interrupt any command including blocking ones
        Cleans up background threads to return to initial state
        
        Returns:
            dict with success status
        """
        # Set reset flag to signal background tasks to abort
        self.reset_flag = True
        
        # Clean up background thread
        with self.background_task_lock:
            if self.background_task is not None and self.background_task.is_alive():
                if self.verbose:
                    print("🛑 Terminating background task...")
                # Wait briefly for background task to notice the flag
                time.sleep(0.1)
            self.background_task = None
        
        if self.verbose:
            print("🚨 EMERGENCY RESET")
        result = self._send_command('platform', 'reset')
        
        # Clear reset flag after command sent
        self.reset_flag = False
        
        # Get final status after reset
        final_status = self.get_status()
        result['final_status'] = final_status
        
        if self.verbose:
            if result['success']:
                print(f"✅ Emergency reset sent")
            else:
                print(f"❌ Emergency reset failed: {result.get('error')}")
        
        return result
    
    # ==================== Convenience Methods ====================
    
    def _execute_in_background(self, func, *args, **kwargs):
        """
        Execute a function in background thread (for interactive mode)
        
        Args:
            func: Function to execute
            *args, **kwargs: Arguments to pass to function
        
        Returns:
            bool: True if task started, False if rejected (previous task still running)
        """
        def wrapper():
            try:
                result = func(*args, **kwargs)
                # Don't print result here - function already prints if verbose
            except Exception as e:
                print(f"\n❌ Background task error: {e}")
            finally:
                with self.background_task_lock:
                    self.background_task = None
        
        with self.background_task_lock:
            # Check if previous background task is still running
            if self.background_task is not None and self.background_task.is_alive():
                if self.verbose:
                    print("⚠️  Previous command still running. Use 'stop' or 'reset' to interrupt.")
                return False
            
            # Start new background task
            self.background_task = threading.Thread(target=wrapper, daemon=True)
            self.background_task.start()
            return True
    
    
    def _wait_for_completion(self, target='platform', timeout=300, poll_interval=0.1):
        """
        Internal method: Wait for platform or pushrod task to complete
        
        Args:
            target: 'platform' or 'pushrod'
            timeout: Maximum wait time in seconds
            poll_interval: Status polling interval in seconds (default: 0.1s = 10Hz)
            
        Returns:
            dict with success status and completion info
        """
        start_time = time.time()
        if self.verbose:
            print(f"⏳ Waiting for [{target}] task to complete (timeout: {timeout}s)...")
        
        # Temporarily disable verbose to avoid spamming status output during polling
        original_verbose = self.verbose
        self.verbose = False
        
        try:
            # First, wait for task to start (state changes from idle/completed to running/executing)
            # OR check if task already completed (target already reached)
            task_started = False
            initial_wait_timeout = 5.0  # Wait up to 5 seconds for task to start
            
            # Track 'completed' state to distinguish old state vs. already at target
            completed_first_seen = None
            completed_stable_threshold = 0.2  # Must stay 'completed' for 200ms to confirm already at target
            
            while time.time() - start_time < initial_wait_timeout:
                # Check for reset flag
                if self.reset_flag:
                    self.verbose = original_verbose
                    return {"success": False, "error": "Aborted by reset"}
                
                status_result = self._get_status()
                if status_result['success'] and target in status_result:
                    task_state = status_result[target].get('task_state', 'unknown')
                    
                    # Check if task is running - new task has started
                    if task_state in ['running', 'executing']:
                        task_started = True
                        if original_verbose:
                            print(f"🔄 Task started, waiting for completion...")
                        break
                    
                    # Check if task state is 'completed'
                    elif task_state == 'completed':
                        # Track how long it stays 'completed'
                        if completed_first_seen is None:
                            completed_first_seen = time.time()
                        elif time.time() - completed_first_seen >= completed_stable_threshold:
                            # Been 'completed' for >= 1s, likely already at target (not a new task)
                            full_data = status_result.get('_full_data', {}).get(target, {})
                            reason = full_data.get('completion_reason', 'unknown')
                            
                            if original_verbose:
                                print(f"✅ Target already reached (no movement needed): {reason}")
                            
                            self.verbose = original_verbose
                            final_status = self.get_status()
                            
                            return {
                                "success": True,
                                "task_state": "completed",
                                "completion_reason": reason,
                                "duration": 0.0,
                                "final_status": final_status
                            }
                    else:
                        # State is not 'completed', reset the tracker
                        completed_first_seen = None
                
                time.sleep(poll_interval)
            
            if not task_started and original_verbose:
                print(f"⚠️  Task did not start within {initial_wait_timeout}s, may have already completed or failed...")
            
            # Now wait for completion
            while time.time() - start_time < timeout:
                # Check for reset flag
                if self.reset_flag:
                    self.verbose = original_verbose
                    return {"success": False, "error": "Aborted by reset"}
                
                status_result = self._get_status()
                
                if status_result['success'] and target in status_result:
                    task_state = status_result[target].get('task_state', 'unknown')
                    
                    if task_state == 'completed':
                        # Try to get from full data if available
                        full_data = status_result.get('_full_data', {}).get(target, {})
                        reason = full_data.get('completion_reason', 'unknown')
                        duration = full_data.get('task_duration', 0)
                        if original_verbose:
                            print(f"✅ Task completed: {reason}")
                        
                        # Restore verbose but don't print final status here
                        # (caller will print it after printing completion message)
                        self.verbose = original_verbose
                        final_status = self.get_status()
                        
                        return {
                            "success": True,
                            "task_state": task_state,
                            "completion_reason": reason,
                            "duration": duration,
                            "final_status": final_status
                        }
                    elif task_state == 'emergency_stop':
                        # Try to get from full data if available
                        full_data = status_result.get('_full_data', {}).get(target, {})
                        reason = full_data.get('completion_reason', 'unknown')
                        if original_verbose:
                            print(f"🚨 EMERGENCY STOP: {reason}")
                        
                        # Restore verbose but don't print final status here
                        self.verbose = original_verbose
                        final_status = self.get_status()
                        
                        return {
                            "success": False,
                            "task_state": task_state,
                            "error": f"Emergency stop: {reason}",
                            "final_status": final_status
                        }
                
                time.sleep(poll_interval)
            
            # Timeout - restore verbose and get final status
            self.verbose = original_verbose
            if original_verbose:
                print(f"⏱️  Timeout after {timeout}s")
                self.get_status()  # Print final status on timeout
            
            return {
                "success": False,
                "error": f"Timeout after {timeout}s"
            }
        finally:
            # Ensure verbose is always restored
            self.verbose = original_verbose


def interactive_mode(robot):
    """
    Interactive command-line mode for manual control
    
    Args:
        robot: CourierRobotWebAPI instance
    
    Available commands:
    
    Status & Info:
      status, st        - Show current status
      sensor, sn        - Show sensor data
      help, h, ?        - Show this help
      quit, exit, q     - Exit interactive mode
    
    Server Configuration:
      setid <id>        - Set server ID (updates high_pos/low_pos), e.g., 'setid 15'
    
    Platform Height Control:
      goto <height>     - Go to height (mm), e.g., 'goto 900' (runs in background, can be interrupted)
      g <height>        - Short form of goto
      goto! <height>    - Non-blocking goto (fire-and-forget)
      g! <height>       - Non-blocking short form
    
    Platform Force Control:
      fup <force>       - Force control up (N), e.g., 'fup 50' (background)
      fdown <force>     - Force control down (N), e.g., 'fdown 30' (background)
      fup! <force>      - Non-blocking force up
      fdown! <force>    - Non-blocking force down
      hybrid <h> <f>    - Hybrid control, e.g., 'hybrid 900 50' or 'hybrid high_pos 200' (background)
      hybrid! <h> <f>   - Non-blocking hybrid control
                          <h> can be: numeric height (mm), 'high_pos', 'high', 'low_pos', or 'low'
    
    Platform Manual Control:
      up                - Manual up (use 'stop' to stop)
      down              - Manual down (use 'stop' to stop)
      stop              - Stop platform (can interrupt any command!)
    
    Pushrod Control:
      pup               - Pushrod manual up (use 'pstop' to stop)
      pdown             - Pushrod manual down (use 'pstop' to stop)
      pgoto <height>    - Pushrod goto absolute height (mm, background)
      pgoto! <height>   - Pushrod goto absolute height (non-blocking)
      prel <offset>     - Pushrod relative move (mm, background), e.g., 'prel 10' or 'prel -5'
      prel! <offset>    - Pushrod relative move (non-blocking)
      pstop             - Stop pushrod (can interrupt any command!)
    
    Emergency:
      reset, emergency  - Emergency reset (can interrupt any command!)
    
    Note: Commands without '!' run in background - you can type 'stop' anytime to interrupt!
    """
    print("\n" + "="*60)
    print("🤖 CourierRobot Interactive Mode")
    print("="*60)
    print("Type 'help' for available commands, 'quit' to exit")
    print("="*60 + "\n")
    
    while True:
        try:
            cmd = input("robot> ").strip().lower()
            
            if not cmd:
                continue
            
            parts = cmd.split()
            command = parts[0]
            
            # Exit commands
            if command in ['quit', 'exit', 'q']:
                print("👋 Exiting interactive mode")
                break
            
            # Help
            elif command in ['help', 'h', '?']:
                print(interactive_mode.__doc__)
            
            # Status commands
            elif command in ['status', 'st']:
                import json
                status = robot.get_status()
                print(json.dumps(status, indent=2))
            
            elif command in ['sensor', 'sn']:
                robot.get_sensor_data()
            
            # Server ID configuration
            elif command == 'setid':
                if len(parts) < 2:
                    print("❌ Usage: setid <id>")
                else:
                    try:
                        server_id = int(parts[1])
                        robot.set_server_id(server_id)
                    except ValueError:
                        print("❌ Invalid server ID value")
            
            # Platform height control
            elif command in ['goto', 'g', 'goto!', 'g!']:
                if len(parts) < 2:
                    print("❌ Usage: goto <height> or goto! <height> (non-blocking)")
                else:
                    try:
                        height = float(parts[1])
                        wait = not command.endswith('!')  # Non-blocking if ends with !
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_goto_height, height, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_goto_height(height, wait=False)
                    except ValueError:
                        print("❌ Invalid height value")
            
            # Platform force control
            elif command in ['fup', 'fup!']:
                if len(parts) < 2:
                    print("❌ Usage: fup <force> or fup! <force> (non-blocking)")
                else:
                    try:
                        force = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_force_up, force, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_force_up(force, wait=False)
                    except ValueError:
                        print("❌ Invalid force value")
            
            elif command in ['fdown', 'fdown!']:
                if len(parts) < 2:
                    print("❌ Usage: fdown <force> or fdown! <force> (non-blocking)")
                else:
                    try:
                        force = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_force_down, force, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_force_down(force, wait=False)
                    except ValueError:
                        print("❌ Invalid force value")
            
            elif command in ['hybrid', 'hybrid!']:
                if len(parts) < 3:
                    print("❌ Usage: hybrid <height|high_pos|low_pos> <force> or hybrid! <height|high_pos|low_pos> <force> (non-blocking)")
                else:
                    try:
                        # height can be numeric or string ('high_pos', 'low_pos')
                        height_str = parts[1]
                        if height_str.lower() in ['high_pos', 'high', 'low_pos', 'low']:
                            height = height_str  # Pass string directly
                        else:
                            height = float(height_str)  # Parse as numeric
                        
                        force = float(parts[2])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_hybrid_control, height, force, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_hybrid_control(height, force, wait=False)
                    except ValueError:
                        print("❌ Invalid force value")
            
            # Platform manual control
            elif command == 'up':
                robot.platform_up()
            
            elif command == 'down':
                robot.platform_down()
            
            elif command == 'stop':
                robot.platform_stop()
            
            # Pushrod control
            elif command == 'pup':
                robot.pushrod_up()
            
            elif command == 'pdown':
                robot.pushrod_down()
            
            elif command in ['pgoto', 'pgoto!']:
                if len(parts) < 2:
                    print("❌ Usage: pgoto <height> or pgoto! <height> (non-blocking)")
                else:
                    try:
                        height = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.pushrod_goto_height, height, mode='absolute', wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.pushrod_goto_height(height, mode='absolute', wait=False)
                    except ValueError:
                        print("❌ Invalid height value")
            
            elif command in ['prel', 'prel!']:
                if len(parts) < 2:
                    print("❌ Usage: prel <offset> or prel! <offset> (non-blocking)")
                else:
                    try:
                        offset = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.pushrod_goto_height, offset, mode='relative', wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.pushrod_goto_height(offset, mode='relative', wait=False)
                    except ValueError:
                        print("❌ Invalid offset value")
            
            elif command == 'pstop':
                robot.pushrod_stop()
            
            # Emergency
            elif command in ['reset', 'emergency']:
                robot.emergency_reset()
            
            else:
                print(f"❌ Unknown command: '{command}'. Type 'help' for available commands.")
        
        except KeyboardInterrupt:
            print("\n👋 Exiting interactive mode (Ctrl+C)")
            break
        except Exception as e:
            print(f"❌ Error: {e}")


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Courier Robot Web API Controller',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--url',
        type=str,
        default='http://192.168.1.3:8090',
        help='Base URL of the courier robot HTTP server'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        default=True,
        help='Enable verbose output'
    )
    parser.add_argument(
        '--no-verbose',
        dest='verbose',
        action='store_false',
        help='Disable verbose output'
    )
    
    args = parser.parse_args()
    
    # Example usage
    print("="*60)
    print("CourierRobot Control Example")
    print("="*60)
    
    # Initialize robot with parsed URL
    robot = CourierRobotWebAPI(base_url=args.url, verbose=args.verbose)
    
    # Run interactive mode
    interactive_mode(robot)
    # ==================== Test Area ====================
