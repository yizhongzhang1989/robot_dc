#!/usr/bin/env python3
"""
Force Sensor Configuration Dashboard
Web interface for reading and writing force sensor parameters via ROS2 Modbus service
"""

import threading
import os
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
import logging
from typing import Optional
from array import array
import time


class ForceSensorDashboard:
    """
    Web dashboard for force sensor configuration
    
    Uses ROS2 Modbus service for ALL communication (avoids serial port conflicts)
    
    Supports reading and writing:
    - Mask value
    - Sampling frequency
    - RC filter coefficient
    - Zero tracking range
    - Zero tracking time
    - Clear zero
    - Tare operation
    - Input polarity
    """
    
    def __init__(self, device_id=52, ros_node=None, host='0.0.0.0', web_port=5001):
        """
        Initialize force sensor dashboard
        
        Args:
            device_id: Modbus device ID (1-247)
            ros_node: ROS2 node instance (for Modbus service access)
            host: Web server host
            web_port: Web server port
        """
        self.device_id = device_id
        self.ros_node = ros_node
        self.host = host
        self.web_port = web_port
        
        # Current configuration cache
        self.current_config = {}
        self.config_lock = threading.Lock()
        
        # Flask App Setup
        template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config['SECRET_KEY'] = 'force_sensor_dashboard_secret'
        
        # Socket.IO Setup
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Disable Flask logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        log = logging.getLogger('socketio')
        log.setLevel(logging.ERROR)
        log = logging.getLogger('engineio')
        log.setLevel(logging.ERROR)
        
        self._setup_routes()
        
        self.flask_thread = threading.Thread(target=self._run, daemon=True)
        
        print(f"Force Sensor Dashboard initialized:")
        print(f"  Device ID: {self.device_id}")
        print(f"  Using ROS2 Modbus Service: /modbus_request")
        print(f"  Web Interface: http://0.0.0.0:{self.web_port}")
    
    def _read_registers(self, start_addr: int, count: int) -> Optional[list]:
        """Read multiple registers via ROS2 Modbus service (Function Code 0x03)"""
        if not self.ros_node:
            print("ERROR: ROS2 node not available")
            return None
        
        try:
            from modbus_driver_interfaces.srv import ModbusRequest
            
            # Create service client if not exists
            if not hasattr(self, '_modbus_client'):
                self._modbus_client = self.ros_node.create_client(ModbusRequest, '/modbus_request')
            
            if not self._modbus_client.wait_for_service(timeout_sec=1.0):
                print("ERROR: Modbus service not available")
                return None
            
            # Create request
            req = ModbusRequest.Request()
            req.slave_id = self.device_id
            req.function_code = 3  # Read Holding Registers
            req.address = start_addr
            req.count = count
            req.values = array('H')  # Empty for read
            
            # Call service synchronously
            future = self._modbus_client.call_async(req)
            
            # Wait for response with timeout
            timeout_time = time.time() + 2.0
            while not future.done() and time.time() < timeout_time:
                time.sleep(0.01)
            
            if not future.done():
                print(f"ERROR: Timeout reading registers {start_addr}-{start_addr+count-1}")
                return None
            
            response = future.result()
            
            if response.success and len(response.response) >= count:
                return list(response.response[:count])
            else:
                print(f"ERROR: Failed to read registers {start_addr}-{start_addr+count-1}")
                return None
                
        except Exception as e:
            print(f"Error reading registers {start_addr}-{start_addr+count-1}: {e}")
            return None
    
    def _read_register(self, reg_addr: int) -> Optional[int]:
        """Read single register via ROS2 Modbus service (Function Code 0x03)"""
        if not self.ros_node:
            print("ERROR: ROS2 node not available")
            return None
        
        try:
            from modbus_driver_interfaces.srv import ModbusRequest
            
            # Create service client if not exists
            if not hasattr(self, '_modbus_client'):
                self._modbus_client = self.ros_node.create_client(ModbusRequest, '/modbus_request')
            
            if not self._modbus_client.wait_for_service(timeout_sec=1.0):
                print("ERROR: Modbus service not available")
                return None
            
            # Create request
            req = ModbusRequest.Request()
            req.slave_id = self.device_id
            req.function_code = 3  # Read Holding Registers
            req.address = reg_addr
            req.count = 1
            req.values = array('H')  # Empty for read
            
            # Call service synchronously
            future = self._modbus_client.call_async(req)
            
            # Wait for response with timeout
            timeout_time = time.time() + 2.0
            while not future.done() and time.time() < timeout_time:
                time.sleep(0.01)
            
            if not future.done():
                print(f"ERROR: Timeout reading register {reg_addr}")
                return None
            
            response = future.result()
            
            if response.success and len(response.response) > 0:
                return response.response[0]
            else:
                print(f"ERROR: Failed to read register {reg_addr}")
                return None
                
        except Exception as e:
            print(f"Error reading register {reg_addr}: {e}")
            return None
    
    def _write_register(self, reg_addr: int, value: int) -> bool:
        """Write single register via ROS2 Modbus service (Function Code 0x06)"""
        if not self.ros_node:
            print("ERROR: ROS2 node not available")
            return False
        
        try:
            from modbus_driver_interfaces.srv import ModbusRequest
            
            # Create service client if not exists
            if not hasattr(self, '_modbus_client'):
                self._modbus_client = self.ros_node.create_client(ModbusRequest, '/modbus_request')
            
            if not self._modbus_client.wait_for_service(timeout_sec=1.0):
                print("ERROR: Modbus service not available")
                return False
            
            # Create request
            req = ModbusRequest.Request()
            req.slave_id = self.device_id
            req.function_code = 6  # Write Single Register
            req.address = reg_addr
            req.count = 1
            req.values = array('H', [value & 0xFFFF])
            
            # Call service synchronously
            future = self._modbus_client.call_async(req)
            
            # Wait for response with timeout
            timeout_time = time.time() + 2.0
            while not future.done() and time.time() < timeout_time:
                time.sleep(0.01)
            
            if not future.done():
                print(f"ERROR: Timeout writing register {reg_addr}")
                return False
            
            response = future.result()
            
            if response.success:
                return True
            else:
                print(f"ERROR: Failed to write register {reg_addr}")
                return False
                
        except Exception as e:
            print(f"Error writing register {reg_addr}: {e}")
            return False
    
    def read_config(self):
        """Read all configuration parameters (optimized with batch read)"""
        config = {}
        
        # Batch read registers 40004-40006 (addresses 0x0003-0x0005)
        # Mask value, Sampling freq, RC filter
        batch1 = self._read_registers(0x0003, 3)
        if batch1:
            config['mask_value'] = batch1[0]
            config['sampling_freq'] = batch1[1]
            config['rc_filter'] = batch1[2]
        else:
            config['mask_value'] = 0
            config['sampling_freq'] = 5
            config['rc_filter'] = 1
        
        # Batch read registers 40021-40022 (addresses 0x0014-0x0015)
        # Zero tracking range, Zero tracking time
        batch2 = self._read_registers(0x0014, 2)
        if batch2:
            config['zero_track_range'] = batch2[0]
            config['zero_track_time'] = batch2[1]
        else:
            config['zero_track_range'] = 0
            config['zero_track_time'] = 0
        
        # Read polarity (40024, address 0x0017)
        polarity = self._read_register(0x0017)
        config['polarity'] = polarity if polarity is not None else 2
        
        # Add descriptions
        config['sampling_freq_desc'] = {
            1: '600 pcs/s', 2: '300 pcs/s', 3: '150 pcs/s',
            4: '75 pcs/s', 5: '37.5 pcs/s (default)', 
            6: '18.75 pcs/s', 7: '10 pcs/s'
        }.get(config['sampling_freq'], 'Unknown')
        
        config['rc_filter_desc'] = {
            1: 'None', 2: '0.8', 3: '0.6', 4: '0.4', 5: '0.2', 6: '0.1'
        }.get(config['rc_filter'], 'Unknown')
        
        config['zero_track_time_desc'] = {
            0: 'Disabled', 1: '5s', 2: '10s', 3: '15s', 4: '20s'
        }.get(config['zero_track_time'], 'Unknown')
        
        config['polarity_desc'] = {
            1: 'Unipolar (0~65535)', 2: 'Bipolar (-32768~32767)'
        }.get(config['polarity'], 'Unknown')
        
        # Cache the configuration
        with self.config_lock:
            self.current_config = config
        
        return config
    
    def _setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('force_sensor_config.html')
        
        @self.app.route('/api/read_config', methods=['GET'])
        def api_read_config():
            """Read all configuration from sensor"""
            try:
                config = self.read_config()
                return jsonify({'success': True, 'config': config})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.app.route('/api/write_config', methods=['POST'])
        def api_write_config():
            """Write configuration parameter to sensor"""
            try:
                data = request.get_json()
                param = data.get('parameter')
                value = data.get('value')
                
                if param is None or value is None:
                    return jsonify({'success': False, 'error': 'Missing parameter or value'}), 400
                
                # Parameter to register address mapping
                param_map = {
                    'mask_value': 0x0003,       # 40004
                    'sampling_freq': 0x0004,    # 40005
                    'rc_filter': 0x0005,        # 40006
                    'zero_track_range': 0x0014, # 40021
                    'zero_track_time': 0x0015,  # 40022
                    'clear_zero': 0x0016,       # 40023
                    'tare': 0x0011,             # 40018
                    'polarity': 0x0017          # 40024
                }
                
                if param not in param_map:
                    return jsonify({'success': False, 'error': f'Unknown parameter: {param}'}), 400
                
                reg_addr = param_map[param]
                
                # Special handling for clear_zero (must be 0x0011)
                if param == 'clear_zero' and value == 1:
                    value = 0x0011
                
                # Write to sensor
                success = self._write_register(reg_addr, int(value))
                
                if success:
                    # Wait for device to update
                    time.sleep(0.3)
                    
                    # Re-read config to verify
                    config = self.read_config()
                    
                    # Verify the write (skip for special commands)
                    if param not in ['tare', 'clear_zero']:
                        actual_value = config.get(param)
                        if actual_value == value:
                            return jsonify({
                                'success': True,
                                'message': f'{param} updated to {value}',
                                'config': config
                            })
                        else:
                            return jsonify({
                                'success': False,
                                'error': f'Write sent but value not updated (expected {value}, got {actual_value})',
                                'config': config
                            }), 500
                    else:
                        # Special commands don't need verification
                        return jsonify({
                            'success': True,
                            'message': f'{param} operation completed',
                            'config': config
                        })
                else:
                    return jsonify({'success': False, 'error': 'Modbus write operation failed'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.app.route('/api/device_info', methods=['GET'])
        def api_device_info():
            """Get device connection information"""
            return jsonify({
                'device_id': self.device_id,
                'port': 'ROS2 Modbus Service',
                'baudrate': 115200,
                'connected': self.ros_node is not None
            })
        
        @self.app.route('/api/raw_value', methods=['GET'])
        def api_raw_value():
            """Get raw sensor value (register 40001)"""
            try:
                raw_value = self._read_register(0x0000)  # 40001 - 1 = 0x0000
                if raw_value is not None:
                    # Convert to signed 16-bit for bipolar mode
                    if raw_value > 32767:
                        raw_value = raw_value - 65536
                    return jsonify({'success': True, 'value': raw_value})
                else:
                    return jsonify({'success': False, 'error': 'Failed to read sensor'}), 500
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
    
    def _run(self):
        """Run Flask server"""
        self.socketio.run(self.app, host=self.host, port=self.web_port, 
                         debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
    
    def start(self):
        """Start the web dashboard"""
        self.flask_thread.start()
        print(f"✅ Force Sensor Dashboard started at http://0.0.0.0:{self.web_port}")
    
    def stop(self):
        """Stop the web dashboard"""
        print("Stopping Force Sensor Dashboard...")
