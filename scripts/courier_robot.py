#!/usr/bin/env python3
"""
Courier Robot Controller
Provides HTTP API interface for lift platform and pushrod control
All functions correspond to web interface controls
"""

import requests
import time
import threading


class CourierRobotWebAPI:
    """
    Courier robot controller for lift platform and pushrod via HTTP API
    Provides all control functions available in the web interface
    """
    
    def __init__(self, base_url="http://192.168.1.3:8090", verbose=True):
        """
        Initialize courier robot controller
        
        Args:
            base_url: HTTP server base URL (default: http://192.168.1.3:8090)
            verbose: If True, automatically print command results (default: True)
        """
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
                        'control_mode': status_data.get('platform', {}).get('control_mode')
                    },
                    "pushrod": {
                        'task_state': status_data.get('pushrod', {}).get('task_state'),
                        'movement_state': status_data.get('pushrod', {}).get('movement_state')
                    }
                }
                
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
                        'left_force_freq_hz': sensor_data.get('left_force_freq_hz')
                    }
                else:
                    result['sensors'] = {}
                
                return result
            return {"success": False, "error": f"HTTP {status_response.status_code}"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_status(self):
        """
        Public API: Get current system status with optional verbose printing
        
        Returns:
            dict with simplified status for external API users:
            - success: bool
            - platform: {task_state, movement_state, control_mode}
            - pushrod: {task_state, movement_state}
            - sensors: {height, forces, frequencies}
        """
        result = self._get_status()
        
        # Print friendly status if verbose
        if self.verbose and result.get('success'):
            # Use full data for printing if available
            data_to_print = result.get('_full_data', {})
            
            print("\n" + "="*60)
            print("📊 SYSTEM STATUS")
            print("="*60)
            
            # Platform status
            if 'platform' in data_to_print:
                pf = data_to_print['platform']
                print(f"\n🏗️  Platform:")
                print(f"   Task State: {pf.get('task_state', 'N/A')}")
                print(f"   Movement: {pf.get('movement_state', 'N/A')}")
                print(f"   Control Mode: {pf.get('control_mode', 'N/A')}")
                if pf.get('current_height') is not None:
                    print(f"   Current Height: {pf['current_height']:.2f} mm")
                if pf.get('target_height') is not None:
                    print(f"   Target Height: {pf['target_height']:.2f} mm")
            
            # Pushrod status
            if 'pushrod' in data_to_print:
                pr = data_to_print['pushrod']
                print(f"\n🔧 Pushrod:")
                print(f"   Task State: {pr.get('task_state', 'N/A')}")
                print(f"   Movement: {pr.get('movement_state', 'N/A')}")
                if pr.get('current_height') is not None:
                    print(f"   Current Height: {pr['current_height']:.2f} mm")
            
            # Sensor data
            if result.get('sensors'):
                s = result['sensors']
                print(f"\n📡 Sensors:")
                if s.get('height') is not None:
                    print(f"   Height: {s['height']:.2f} mm")
                if s.get('right_force') is not None:
                    freq_r = s.get('right_force_freq_hz', 0)
                    print(f"   Right Force: {s['right_force']:.2f} N ({freq_r:.1f} Hz)")
                if s.get('left_force') is not None:
                    freq_l = s.get('left_force_freq_hz', 0)
                    print(f"   Left Force: {s['left_force']:.2f} N ({freq_l:.1f} Hz)")
                if s.get('combined_force') is not None:
                    print(f"   Combined Force: {s['combined_force']:.2f} N")
                if s.get('freq_hz') is not None:
                    print(f"   Height Sensor Freq: {s['freq_hz']:.1f} Hz")
            
            print("="*60 + "\n")
        
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
    
    def platform_goto_height(self, target_height, wait=True, timeout=60):
        """
        Platform goto specific height (BLOCKING by default)
        
        Args:
            target_height: Target height in mm
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 60)
            
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
    
    def platform_force_up(self, target_force, wait=True, timeout=60):
        """
        Platform force-controlled up movement (BLOCKING by default)
        
        Args:
            target_force: Target force in Newtons
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 60)
            
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
    
    def platform_force_down(self, target_force, wait=True, timeout=60):
        """
        Platform force-controlled down movement (BLOCKING by default)
        
        Args:
            target_force: Target force in Newtons
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 60)
            
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
    
    def platform_hybrid_control(self, target_height, target_force, wait=True, timeout=60):
        """
        Platform hybrid control (height OR force, whichever reached first) (BLOCKING by default)
        
        Args:
            target_height: Target height in mm
            target_force: Target force in Newtons
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 60)
            
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
    
    def pushrod_goto_height(self, target_height, mode='absolute', wait=True, timeout=60):
        """
        Pushrod goto specific height (BLOCKING by default)
        
        Args:
            target_height: Target height in mm (absolute) or offset in mm (relative)
            mode: 'absolute' or 'relative' (default: 'absolute')
            wait: If True, wait for completion before returning (default: True)
                  If False (non-blocking), will auto-stop previous task if running
            timeout: Maximum wait time in seconds (default: 60)
            
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
    
    
    def _wait_for_completion(self, target='platform', timeout=60, poll_interval=0.1):
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
                            duration_str = f"{duration:.2f}s" if duration is not None else "N/A"
                            print(f"✅ Task completed: {reason} (duration: {duration_str})")
                        
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


if __name__ == "__main__":
    # Example usage
    print("="*60)
    print("CourierRobot Control Example")
    print("="*60)
    
    # Initialize robot
    robot = CourierRobotWebAPI()
    
    # Import and run interactive mode
    from courier_robot_terminal_test import interactive_mode
    interactive_mode(robot)
    # ==================== Test Area ====================
# Uncomment the commands you want to test

# 1. Query status
# robot.get_status()

# 2. Query sensors
# robot.get_sensor_data()

# ========== Blocking Mode Tests (default wait=True) ==========

# 3. goto 900mm (blocking, auto-wait for completion)
# robot.platform_goto_height(900)

# 4. goto 850mm (blocking, auto-wait for completion)
# robot.platform_goto_height(850)

# 5. Force control up to 50N (blocking)
# robot.platform_force_up(50.0)

# 6. Force control down to 30N (blocking)
# robot.platform_force_down(30.0)

# 7. Hybrid control (height 900mm, force 50N) (blocking)
# robot.platform_hybrid_control(900, 50.0)

# 8. Pushrod goto 100mm (blocking)
# robot.pushrod_goto_height(100)

# ========== Non-blocking Mode Tests (wait=False, auto-override) ==========

# 9. Non-blocking goto 900, then immediately change to 850 (auto-stop old task)
# robot.platform_goto_height(900, wait=False)
# time.sleep(0.5)  # Let it start moving
# robot.platform_goto_height(850, wait=False)  # ⚠️ Will auto-stop, then goto 850

# 10. Non-blocking force control, then immediately change target force (auto-override)
# robot.platform_force_up(50.0, wait=False)
# time.sleep(0.5)
# robot.platform_force_up(30.0, wait=False)  # ⚠️ Auto-stop, change to 30N

# ========== Manual Control (always non-blocking) ==========

# 11. Manual up (non-blocking, requires manual stop)
# robot.platform_up()
# time.sleep(2)
# robot.platform_stop()

# 12. Manual down (non-blocking, requires manual stop)
# robot.platform_down()
# time.sleep(2)
# robot.platform_stop()

# 13. Pushrod manual up (non-blocking)
# robot.pushrod_up()
# time.sleep(2)
# robot.pushrod_stop()

# ========== Emergency Stop ==========

# 14. Emergency stop
# robot.emergency_reset()
