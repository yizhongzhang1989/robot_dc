#!/usr/bin/env python3
"""
Courier Robot Controller
Provides HTTP API interface for lift platform and pushrod control
All functions correspond to web interface controls
"""

import requests
import time


class CourierRobot:
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
    
    def get_status(self, print_status=True):
        """
        Get current status of platform and pushrod with sensor data
        
        Args:
            print_status: If True and verbose=True, print formatted status (default: True)
        
        Returns:
            dict with complete system status including:
            - data: platform and pushrod task_state, movement_state, etc.
            - sensors: height, force, frequency, etc.
        """
        try:
            # Get task status
            status_url = f"{self.base_url}/api/status"
            status_response = requests.get(status_url, timeout=2)
            
            # Get sensor data
            sensor_url = f"{self.base_url}/api/latest"
            sensor_response = requests.get(sensor_url, timeout=2)
            
            if status_response.status_code == 200:
                result = {"success": True, "data": status_response.json()}
                
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
                
                # Print friendly status if verbose and print_status=True
                if self.verbose and print_status:
                    print("\n" + "="*60)
                    print("📊 SYSTEM STATUS")
                    print("="*60)
                    
                    # Platform status
                    if 'platform' in result['data']:
                        pf = result['data']['platform']
                        print(f"\n🏗️  Platform:")
                        print(f"   Task State: {pf.get('task_state', 'N/A')}")
                        print(f"   Movement: {pf.get('movement_state', 'N/A')}")
                        print(f"   Control Mode: {pf.get('control_mode', 'N/A')}")
                        if pf.get('current_height') is not None:
                            print(f"   Current Height: {pf['current_height']:.2f} mm")
                        if pf.get('target_height') is not None:
                            print(f"   Target Height: {pf['target_height']:.2f} mm")
                    
                    # Pushrod status
                    if 'pushrod' in result['data']:
                        pr = result['data']['pushrod']
                        print(f"\n🔧 Pushrod:")
                        print(f"   Task State: {pr.get('task_state', 'N/A')}")
                        print(f"   Movement: {pr.get('movement_state', 'N/A')}")
                        if pr.get('current_height') is not None:
                            print(f"   Current Height: {pr['current_height']:.2f} mm")
                    
                    # Sensor data
                    if result['sensors']:
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
                
                return result
            return {"success": False, "error": f"HTTP {status_response.status_code}"}
        except Exception as e:
            if self.verbose:
                print(f"❌ Failed to get status: {e}")
            return {"success": False, "error": str(e)}
    
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            platform_state = status.get('data', {}).get('platform', {}).get('task_state', 'unknown')
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
        result['status'] = final_status
        
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            platform_state = status.get('data', {}).get('platform', {}).get('task_state', 'unknown')
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
        result['status'] = final_status
        
        if self.verbose:
            if result['success']:
                print(f"✅ Platform DOWN command sent successfully")
            else:
                print(f"❌ Failed: {result.get('error')}")
        return result
    
    def platform_stop(self):
        """
        Platform stop
        
        Returns:
            dict with success status and complete state
        """
        # Stop command should always be allowed (no state check)
        if self.verbose:
            print(f"🛑 [Platform] STOP")
        result = self._send_command('platform', 'stop')
        
        # Get final status after command
        final_status = self.get_status()
        result['status'] = final_status
        
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            platform_state = status.get('data', {}).get('platform', {}).get('task_state', 'unknown')
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
                    time.sleep(0.1)  # Brief pause for stop to take effect
        
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
            completion = self.wait_for_completion(target='platform', timeout=timeout)
            if completion['success'] and self.verbose:
                duration = completion.get('duration', 0) or 0
                print(f"✅ Goto {target_height}mm completed ({duration:.2f}s)")
            return completion
        else:
            # Non-blocking mode
            final_status = self.get_status()
            result['status'] = final_status
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            platform_state = status.get('data', {}).get('platform', {}).get('task_state', 'unknown')
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
                    time.sleep(0.1)  # Brief pause for stop to take effect
        
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
            completion = self.wait_for_completion(target='platform', timeout=timeout)
            if completion['success'] and self.verbose:
                duration = completion.get('duration', 0) or 0
                print(f"✅ Force UP {target_force}N completed ({duration:.2f}s)")
            return completion
        else:
            final_status = self.get_status()
            result['status'] = final_status
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            platform_state = status.get('data', {}).get('platform', {}).get('task_state', 'unknown')
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
                    time.sleep(0.1)  # Brief pause for stop to take effect
        
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
            completion = self.wait_for_completion(target='platform', timeout=timeout)
            if completion['success'] and self.verbose:
                duration = completion.get('duration', 0) or 0
                print(f"✅ Force DOWN {target_force}N completed ({duration:.2f}s)")
            return completion
        else:
            final_status = self.get_status()
            result['status'] = final_status
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            platform_state = status.get('data', {}).get('platform', {}).get('task_state', 'unknown')
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
                    time.sleep(0.1)  # Brief pause for stop to take effect
        
        if self.verbose:
            print(f"🎯⚡ [Platform] Hybrid: {target_height}mm OR {target_force}N")
        result = self._send_command('platform', 'hybrid_control', 
                                target_height=target_height, target_force=target_force)
        
        if not result['success']:
            if self.verbose:
                print(f"❌ Failed: {result.get('error')}")
            return result
        
        # Wait for completion if requested
        if wait:
            if self.verbose:
                print(f"⏳ Waiting for completion (timeout: {timeout}s)...")
            completion = self.wait_for_completion(target='platform', timeout=timeout)
            if completion['success'] and self.verbose:
                duration = completion.get('duration', 0) or 0
                print(f"✅ Hybrid control completed ({duration:.2f}s)")
            return completion
        else:
            final_status = self.get_status()
            result['status'] = final_status
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            pushrod_state = status.get('data', {}).get('pushrod', {}).get('task_state', 'unknown')
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
        result['status'] = final_status
        
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            pushrod_state = status.get('data', {}).get('pushrod', {}).get('task_state', 'unknown')
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
        result['status'] = final_status
        
        if self.verbose:
            if result['success']:
                print(f"✅ Pushrod DOWN command sent successfully")
            else:
                print(f"❌ Failed: {result.get('error')}")
        return result
    
    def pushrod_stop(self):
        """
        Pushrod stop
        
        Returns:
            dict with success status and complete state
        """
        # Stop command should always be allowed (no state check)
        if self.verbose:
            print(f"🛑 [Pushrod] STOP")
        result = self._send_command('pushrod', 'stop')
        
        # Get final status after command
        final_status = self.get_status()
        result['status'] = final_status
        
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
        status = self.get_status(print_status=False)
        if status.get('success'):
            pushrod_state = status.get('data', {}).get('pushrod', {}).get('task_state', 'unknown')
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
                    time.sleep(0.1)  # Brief pause for stop to take effect
        
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
            completion = self.wait_for_completion(target='pushrod', timeout=timeout)
            if completion['success'] and self.verbose:
                duration = completion.get('duration', 0) or 0
                print(f"✅ Pushrod goto {target_height}mm completed ({duration:.2f}s)")
            return completion
        else:
            final_status = self.get_status()
            result['status'] = final_status
            if self.verbose:
                print(f"✅ Goto height command sent (non-blocking)")
            return result
    
    # ==================== Emergency Reset ====================
    
    def emergency_reset(self):
        """
        Emergency reset - stops all movements and clears all states
        
        Returns:
            dict with success status and complete state
        """
        # Emergency reset should always be allowed (no state check)
        if self.verbose:
            print("🚨 EMERGENCY RESET")
        result = self._send_command('platform', 'reset')
        
        # Get final status after command
        import time
        time.sleep(0.5)  # Wait a bit for reset to complete
        final_status = self.get_status()
        result['status'] = final_status
        
        if self.verbose:
            print(f"✅ Emergency reset completed")
        return result
    
    # ==================== Convenience Methods ====================
    
    def wait_for_completion(self, target='platform', timeout=60, poll_interval=0.5):
        """
        Wait for platform or pushrod task to complete
        
        Args:
            target: 'platform' or 'pushrod'
            timeout: Maximum wait time in seconds
            poll_interval: Status polling interval in seconds
            
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
            task_started = False
            initial_wait_timeout = 5.0  # Wait up to 5 seconds for task to start
            
            while time.time() - start_time < initial_wait_timeout:
                status_result = self.get_status()
                if status_result['success'] and target in status_result['data']:
                    task_state = status_result['data'][target].get('task_state', 'unknown')
                    if task_state in ['running', 'executing']:
                        task_started = True
                        if original_verbose:
                            print(f"🔄 Task started, waiting for completion...")
                        break
                time.sleep(poll_interval)
            
            if not task_started and original_verbose:
                print(f"⚠️  Task did not start within {initial_wait_timeout}s, checking if already completed...")
            
            # Now wait for completion
            while time.time() - start_time < timeout:
                status_result = self.get_status()
                
                if status_result['success'] and target in status_result['data']:
                    task_state = status_result['data'][target].get('task_state', 'unknown')
                    
                    if task_state == 'completed':
                        reason = status_result['data'][target].get('completion_reason', 'unknown')
                        duration = status_result['data'][target].get('task_duration', 0)
                        if original_verbose:
                            duration_str = f"{duration:.2f}s" if duration is not None else "N/A"
                            print(f"✅ Task completed: {reason} (duration: {duration_str})")
                        
                        # Restore verbose and get final status with full output
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
                        reason = status_result['data'][target].get('completion_reason', 'unknown')
                        if original_verbose:
                            print(f"🚨 EMERGENCY STOP: {reason}")
                        
                        # Restore verbose and get final status with full output
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
    robot = CourierRobot()
    
    # Get current status
    print("\n1. Getting current status...")
    status = robot.get_status()
    if status['success']:
        print(f"   Platform state: {status['data']['platform']['task_state']}")
        print(f"   Pushrod state: {status['data']['pushrod']['task_state']}")
    
    # Get sensor data
    print("\n2. Getting sensor data...")
    sensors = robot.get_sensor_data()
    if sensors['success']:
        print(f"   Height: {sensors['data'].get('height', 'N/A')} mm")
        print(f"   Force: {sensors['data'].get('combined_force_sensor', 'N/A')} N")
    
    print("\n" + "="*60)
    print("Available methods:")
    print("="*60)
    print("\n📊 Status & Sensors:")
    print("  - robot.get_status()")
    print("  - robot.get_sensor_data()")
    print("\n🏗️ Platform Controls:")
    print("  Manual: platform_up(), platform_down(), platform_stop()")
    print("  Height: platform_goto_height(target_height)")
    print("  Force:  platform_force_up(target_force), platform_force_down(target_force)")
    print("  Hybrid: platform_hybrid_control(target_height, target_force)")
    print("\n🔧 Pushrod Controls:")
    print("  Manual: pushrod_up(), pushrod_down(), pushrod_stop()")
    print("  Height: pushrod_goto_height(target_height, mode='absolute')")
    print("          mode can be 'absolute' or 'relative'")
    print("\n⚖️  Force Sensor Calibration:")
    print("  - robot.tare_force_sensor(channel='right')  # channel: 'right' or 'left'")
    print("  - robot.tare_both_force_sensors()           # Tare both sensors")
    print("\n🚨 Emergency:")
    print("  - robot.emergency_reset()")
    print("\n⏳ Wait for completion:")
    print("  - robot.wait_for_completion(target='platform', timeout=60)")
    print("="*60)
