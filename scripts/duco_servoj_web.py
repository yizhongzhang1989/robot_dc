#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
import threading
from flask import Flask, request, jsonify

# Add duco_robot_arm directory to search path
duco_robot_arm_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src', 'duco_robot_arm', 'duco_robot_arm'))
sys.path.append(duco_robot_arm_path)
sys.path.append(os.path.join(duco_robot_arm_path, 'gen_py'))
sys.path.append(os.path.join(duco_robot_arm_path, 'lib'))

from DucoCobot import DucoCobot

def ConvertPose2Rad(pose):
    """Convert angles to radians"""
    result = []
    for val in pose:
        result.append(math.radians(val))
    return result

def ConvertRad2Pose(pose_rad):
    """Convert radians to angles"""
    result = []
    for val in pose_rad:
        result.append(math.degrees(val))
    return result

class RobotServoController:
    def __init__(self, ip_robot='192.168.1.10', port_robot=7003):
        self.duco_cobot = DucoCobot(ip_robot, port_robot)
        self.current_pose = [0, 0, 0, 0, 0, 0]  # Current joint angles (degrees)
        self.target_pose = [0, 0, 0, 0, 0, 0]   # Target joint angles (degrees)
        self.is_connected = False
        self.is_running = False
        self.control_thread = None
        self.control_interval = 0.1  # 100ms default interval
        self.v = 2.0  # Velocity
        self.a = 2.0  # Acceleration
        self.kp = 250  # Proportional gain
        self.kd = 25   # Derivative gain
        self.lock = threading.Lock()
        
        # Joint angle limits (degrees)
        self.joint_limits = [
            (-180, 180),  # Joint 1
            (-180, 180),  # Joint 2
            (-180, 180),  # Joint 3
            (-180, 180),  # Joint 4
            (-180, 180),  # Joint 5
            (-180, 180),  # Joint 6
        ]
        
    def connect_robot(self):
        """Connect to robot"""
        try:
            rlt = self.duco_cobot.open()
            if rlt == 0:
                self.is_connected = True
                self.get_current_position()
                self.target_pose = self.current_pose.copy()
                print("Robot connection successful")
                return True
            else:
                self.is_connected = False
                print("Robot connection failed")
                return False
        except Exception as e:
            self.is_connected = False
            print(f"Failed to connect to robot: {e}")
            return False
            
    def disconnect_robot(self):
        """Disconnect from robot"""
        try:
            if self.is_running:
                self.stop_control()
            if self.is_connected:
                self.duco_cobot.close()  # Don't need to check return value for disconnect
                self.is_connected = False
                print("Robot disconnected")
        except Exception as e:
            print(f"Error disconnecting robot: {e}")
            
    def get_current_position(self):
        """Get current robot joint positions"""
        try:
            if not self.is_connected:
                return False
            joints_rad = self.duco_cobot.get_actual_joints_position()
            with self.lock:
                self.current_pose = ConvertRad2Pose(joints_rad)
            return True
        except Exception as e:
            print(f"Failed to get current position: {e}")
            return False
            
    def clamp_angle(self, angle, joint_index):
        """Limit joint angles within safe range"""
        min_angle, max_angle = self.joint_limits[joint_index]
        return max(min_angle, min(max_angle, angle))
        
    def set_target_pose(self, pose):
        """Set target joint angles"""
        with self.lock:
            for i, angle in enumerate(pose):
                self.target_pose[i] = self.clamp_angle(angle, i)
                
    def set_control_interval(self, interval):
        """Set control loop interval in seconds"""
        if 0.01 <= interval <= 2.0:  # 10ms to 2s
            self.control_interval = interval
            return True
        return False
        
    def set_control_gains(self, kp, kd):
        """Set PID control gains"""
        if 1 <= kp <= 1000 and 1 <= kd <= 500:  # Reasonable ranges for robot control
            self.kp = kp
            self.kd = kd
            print(f"Control gains updated: kp={kp}, kd={kd}")
            return True
        return False
        
    def control_loop(self):
        """Main control loop running in separate thread"""
        next_time = time.perf_counter()
        
        while self.is_running:
            try:
                if self.is_connected:
                    with self.lock:
                        pose_rad = ConvertPose2Rad(self.target_pose)
                    
                    # Send servo command
                    self.duco_cobot.servoj(pose_rad, self.v, self.a, 
                                          False, kp=self.kp, kd=self.kd)
                    
                    # Update current position
                    self.get_current_position()
                
                # Calculate next execution time
                next_time += self.control_interval
                
                # Accurate timing control
                sleep_time = next_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # If we're behind schedule, reset timing to prevent drift
                    next_time = time.perf_counter()
                    
            except Exception as e:
                print(f"Control loop error: {e}")
                break
                
    def start_control(self):
        """Start servo control"""
        if not self.is_connected:
            return False
            
        if not self.is_running:
            self.is_running = True
            self.control_thread = threading.Thread(target=self.control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            print("Servo control started")
            return True
        return False
        
    def stop_control(self):
        """Stop servo control"""
        if self.is_running:
            self.is_running = False
            if self.control_thread:
                self.control_thread.join(timeout=1.0)
            print("Servo control stopped")
            return True
        return False
        
    def get_status(self):
        """Get robot status"""
        with self.lock:
            return {
                'connected': self.is_connected,
                'running': self.is_running,
                'current_pose': self.current_pose.copy(),
                'target_pose': self.target_pose.copy(),
                'control_interval': self.control_interval,
                'joint_limits': self.joint_limits,
                'kp': self.kp,
                'kd': self.kd
            }

# Global robot controller
robot_controller = RobotServoController()

# Flask web application
app = Flask(__name__)

@app.route('/')
def index():
    """Main page"""
    return get_html_template()

@app.route('/api/connect', methods=['POST'])
def connect():
    """Connect to robot"""
    success = robot_controller.connect_robot()
    return jsonify({'success': success, 'status': robot_controller.get_status()})

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from robot"""
    robot_controller.disconnect_robot()
    return jsonify({'success': True, 'status': robot_controller.get_status()})

@app.route('/api/start_control', methods=['POST'])
def start_control():
    """Start servo control"""
    success = robot_controller.start_control()
    return jsonify({'success': success, 'status': robot_controller.get_status()})

@app.route('/api/stop_control', methods=['POST'])
def stop_control():
    """Stop servo control"""
    success = robot_controller.stop_control()
    return jsonify({'success': success, 'status': robot_controller.get_status()})

@app.route('/api/set_target', methods=['POST'])
def set_target():
    """Set target joint angles"""
    data = request.get_json()
    if 'pose' in data and len(data['pose']) == 6:
        robot_controller.set_target_pose(data['pose'])
        return jsonify({'success': True, 'status': robot_controller.get_status()})
    return jsonify({'success': False, 'error': 'Invalid pose data'})

@app.route('/api/set_interval', methods=['POST'])
def set_interval():
    """Set control interval"""
    data = request.get_json()
    if 'interval' in data:
        success = robot_controller.set_control_interval(data['interval'])
        return jsonify({'success': success, 'status': robot_controller.get_status()})
    return jsonify({'success': False, 'error': 'Invalid interval'})

@app.route('/api/set_gains', methods=['POST'])
def set_gains():
    """Set control gains (kp, kd)"""
    data = request.get_json()
    if 'kp' in data and 'kd' in data:
        success = robot_controller.set_control_gains(data['kp'], data['kd'])
        return jsonify({'success': success, 'status': robot_controller.get_status()})
    return jsonify({'success': False, 'error': 'Invalid gain values'})

@app.route('/api/sync_current', methods=['POST'])
def sync_current():
    """Sync target pose to current pose"""
    if robot_controller.get_current_position():
        with robot_controller.lock:
            robot_controller.target_pose = robot_controller.current_pose.copy()
        return jsonify({'success': True, 'status': robot_controller.get_status()})
    return jsonify({'success': False, 'error': 'Failed to get current position'})

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get current status"""
    return jsonify(robot_controller.get_status())

def get_html_template():
    """Return HTML template as string"""
    return """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>DuCo Robot ServOJ Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        h1 {
            text-align: center;
            color: #333;
            margin-bottom: 30px;
        }
        .status-panel {
            background-color: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            border-left: 4px solid #007bff;
        }
        .control-panel {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 30px;
            margin-bottom: 30px;
        }
        .joint-controls {
            background-color: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
        }
        .joint-slider {
            margin-bottom: 15px;
        }
        .joint-slider label {
            display: block;
            font-weight: bold;
            margin-bottom: 5px;
            color: #333;
        }
        .slider-container {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .slider {
            flex: 1;
            height: 8px;
            border-radius: 4px;
            background: #ddd;
            outline: none;
            -webkit-appearance: none;
        }
        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #007bff;
            cursor: pointer;
        }
        .angle-display {
            min-width: 80px;
            text-align: center;
            font-weight: bold;
            background-color: white;
            padding: 5px 10px;
            border-radius: 4px;
            border: 1px solid #ddd;
        }
        .button-panel {
            text-align: center;
            margin-bottom: 20px;
        }
        .btn {
            padding: 12px 24px;
            margin: 0 10px 10px 0;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-size: 14px;
            font-weight: bold;
            transition: background-color 0.3s;
        }
        .btn-primary {
            background-color: #007bff;
            color: white;
        }
        .btn-primary:hover {
            background-color: #0056b3;
        }
        .btn-success {
            background-color: #28a745;
            color: white;
        }
        .btn-success:hover {
            background-color: #1e7e34;
        }
        .btn-danger {
            background-color: #dc3545;
            color: white;
        }
        .btn-danger:hover {
            background-color: #c82333;
        }
        .btn-warning {
            background-color: #ffc107;
            color: #212529;
        }
        .btn-warning:hover {
            background-color: #e0a800;
        }
        .btn:disabled {
            background-color: #6c757d;
            cursor: not-allowed;
        }
        .interval-control {
            background-color: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
        }
        .interval-input {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-top: 10px;
        }
        .interval-input input {
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 4px;
            width: 100px;
        }
        .pose-display {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-top: 20px;
        }
        .pose-info {
            background-color: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
        }
        .pose-info h4 {
            margin-top: 0;
            color: #333;
        }
        .pose-values {
            font-family: monospace;
            background-color: white;
            padding: 10px;
            border-radius: 4px;
            border: 1px solid #ddd;
        }
        .status-connected {
            color: #28a745;
            font-weight: bold;
        }
        .status-disconnected {
            color: #dc3545;
            font-weight: bold;
        }
        .status-running {
            color: #007bff;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 DuCo Robot ServOJ Control Interface</h1>
        
        <div class="status-panel">
            <strong>Status:</strong> 
            <span id="connection-status" class="status-disconnected">Disconnected</span> | 
            <span id="control-status">Stopped</span> | 
            <span>Interval: <span id="current-interval">100</span> ms</span> | 
            <span>Kp: <span id="current-kp">250</span></span> | 
            <span>Kd: <span id="current-kd">25</span></span>
        </div>
        
        <div class="button-panel">
            <button id="connect-btn" class="btn btn-primary">Connect Robot</button>
            <button id="disconnect-btn" class="btn btn-danger" disabled>Disconnect Robot</button>
            <button id="start-btn" class="btn btn-success" disabled>Start Control</button>
            <button id="stop-btn" class="btn btn-warning" disabled>Stop Control</button>
            <button id="sync-btn" class="btn btn-primary" disabled>Sync to Current Pose</button>
        </div>
        
        <div class="control-panel">
            <div class="joint-controls">
                <h3>🎛️ Joint Controls</h3>
                <div id="joint-sliders">
                    <!-- Joint sliders will be generated here -->
                </div>
            </div>
            
            <div class="interval-control">
                <h3>⏱️ Control Settings</h3>
                <label for="interval-slider">Control Interval:</label>
                <div class="interval-input">
                    <input type="range" id="interval-slider" min="10" max="1000" value="100" step="10">
                    <input type="number" id="interval-value" min="10" max="1000" value="100" step="10">
                    <span>ms</span>
                    <button id="update-interval-btn" class="btn btn-primary">Update</button>
                </div>
                
                <h4 style="margin-top: 20px;">🎛️ PID Control Gains</h4>
                <div class="interval-input">
                    <label for="kp-input">Kp (Proportional):</label>
                    <input type="range" id="kp-slider" min="1" max="1000" value="250" step="1">
                    <input type="number" id="kp-value" min="1" max="1000" value="250" step="1">
                </div>
                <div class="interval-input" style="margin-top: 10px;">
                    <label for="kd-input">Kd (Derivative):</label>
                    <input type="range" id="kd-slider" min="1" max="500" value="25" step="1">
                    <input type="number" id="kd-value" min="1" max="500" value="25" step="1">
                    <button id="update-gains-btn" class="btn btn-primary">Update Gains</button>
                </div>
                
                <h4 style="margin-top: 20px;">📊 Joint Limits</h4>
                <div class="pose-values" id="joint-limits">
                    Joint 1: -180° to 180°<br>
                    Joint 2: -180° to 180°<br>
                    Joint 3: -180° to 180°<br>
                    Joint 4: -180° to 180°<br>
                    Joint 5: -180° to 180°<br>
                    Joint 6: -180° to 180°
                </div>
            </div>
        </div>
        
        <div class="pose-display">
            <div class="pose-info">
                <h4>📍 Current Pose (degrees)</h4>
                <div class="pose-values" id="current-pose">
                    J1: 0.0°, J2: 0.0°, J3: 0.0°<br>
                    J4: 0.0°, J5: 0.0°, J6: 0.0°
                </div>
            </div>
            <div class="pose-info">
                <h4>🎯 Target Pose (degrees)</h4>
                <div class="pose-values" id="target-pose">
                    J1: 0.0°, J2: 0.0°, J3: 0.0°<br>
                    J4: 0.0°, J5: 0.0°, J6: 0.0°
                </div>
            </div>
        </div>
    </div>

    <script>
        let currentStatus = {
            connected: false,
            running: false,
            current_pose: [0, 0, 0, 0, 0, 0],
            target_pose: [0, 0, 0, 0, 0, 0],
            control_interval: 0.1,
            joint_limits: [[-180, 180], [-180, 180], [-180, 180], [-180, 180], [-180, 180], [-180, 180]],
            kp: 250,
            kd: 25
        };

        // Initialize joint sliders
        function initializeSliders() {
            const slidersContainer = document.getElementById('joint-sliders');
            slidersContainer.innerHTML = '';
            
            for (let i = 0; i < 6; i++) {
                const limits = currentStatus.joint_limits[i];
                const sliderDiv = document.createElement('div');
                sliderDiv.className = 'joint-slider';
                sliderDiv.innerHTML = `
                    <label for="joint-${i}">Joint ${i + 1}:</label>
                    <div class="slider-container">
                        <input type="range" id="joint-${i}" class="slider" 
                               min="${limits[0]}" max="${limits[1]}" value="0" step="0.1">
                        <span class="angle-display" id="angle-${i}">0.0°</span>
                    </div>
                `;
                slidersContainer.appendChild(sliderDiv);
                
                // Add event listener
                const slider = document.getElementById(`joint-${i}`);
                const display = document.getElementById(`angle-${i}`);
                
                slider.addEventListener('input', function() {
                    const value = parseFloat(this.value);
                    display.textContent = value.toFixed(1) + '°';
                    currentStatus.target_pose[i] = value;
                    updateTargetPose();
                });
            }
        }

        // Update target pose on server
        function updateTargetPose() {
            if (!currentStatus.connected) return;
            
            fetch('/api/set_target', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    pose: currentStatus.target_pose
                })
            });
        }

        // Update UI with current status
        function updateUI() {
            // Update status display
            const connStatus = document.getElementById('connection-status');
            const ctrlStatus = document.getElementById('control-status');
            
            connStatus.textContent = currentStatus.connected ? 'Connected' : 'Disconnected';
            connStatus.className = currentStatus.connected ? 'status-connected' : 'status-disconnected';
            
            ctrlStatus.textContent = currentStatus.running ? 'Running' : 'Stopped';
            ctrlStatus.className = currentStatus.running ? 'status-running' : '';
            
            document.getElementById('current-interval').textContent = Math.round(currentStatus.control_interval * 1000);
            document.getElementById('current-kp').textContent = currentStatus.kp;
            document.getElementById('current-kd').textContent = currentStatus.kd;
            
            // Update button states
            document.getElementById('connect-btn').disabled = currentStatus.connected;
            document.getElementById('disconnect-btn').disabled = !currentStatus.connected;
            document.getElementById('start-btn').disabled = !currentStatus.connected || currentStatus.running;
            document.getElementById('stop-btn').disabled = !currentStatus.running;
            document.getElementById('sync-btn').disabled = !currentStatus.connected;
            
            // Update sliders
            for (let i = 0; i < 6; i++) {
                const slider = document.getElementById(`joint-${i}`);
                const display = document.getElementById(`angle-${i}`);
                if (slider && display) {
                    slider.value = currentStatus.target_pose[i];
                    display.textContent = currentStatus.target_pose[i].toFixed(1) + '°';
                }
            }
            
            // Update pose displays
            const currentPoseDiv = document.getElementById('current-pose');
            const targetPoseDiv = document.getElementById('target-pose');
            
            currentPoseDiv.innerHTML = formatPose(currentStatus.current_pose);
            targetPoseDiv.innerHTML = formatPose(currentStatus.target_pose);
            
            // Update control parameter inputs
            const intervalSlider = document.getElementById('interval-slider');
            const intervalValue = document.getElementById('interval-value');
            const intervalMs = Math.round(currentStatus.control_interval * 1000);
            intervalSlider.value = intervalMs;
            intervalValue.value = intervalMs;
            
            // Update gain controls
            const kpSlider = document.getElementById('kp-slider');
            const kpValue = document.getElementById('kp-value');
            const kdSlider = document.getElementById('kd-slider');
            const kdValue = document.getElementById('kd-value');
            
            kpSlider.value = currentStatus.kp;
            kpValue.value = currentStatus.kp;
            kdSlider.value = currentStatus.kd;
            kdValue.value = currentStatus.kd;
        }

        function formatPose(pose) {
            return `J1: ${pose[0].toFixed(1)}°, J2: ${pose[1].toFixed(1)}°, J3: ${pose[2].toFixed(1)}°<br>` +
                   `J4: ${pose[3].toFixed(1)}°, J5: ${pose[4].toFixed(1)}°, J6: ${pose[5].toFixed(1)}°`;
        }

        // API functions
        async function apiCall(endpoint, method = 'GET', data = null) {
            try {
                const options = {
                    method: method,
                    headers: {
                        'Content-Type': 'application/json'
                    }
                };
                
                if (data) {
                    options.body = JSON.stringify(data);
                }
                
                const response = await fetch(endpoint, options);
                const result = await response.json();
                
                if (result.status) {
                    currentStatus = result.status;
                    updateUI();
                }
                
                return result;
            } catch (error) {
                console.error('API call failed:', error);
                return { success: false, error: error.message };
            }
        }

        // Event listeners
        document.getElementById('connect-btn').addEventListener('click', () => {
            apiCall('/api/connect', 'POST');
        });

        document.getElementById('disconnect-btn').addEventListener('click', () => {
            apiCall('/api/disconnect', 'POST');
        });

        document.getElementById('start-btn').addEventListener('click', () => {
            apiCall('/api/start_control', 'POST');
        });

        document.getElementById('stop-btn').addEventListener('click', () => {
            apiCall('/api/stop_control', 'POST');
        });

        document.getElementById('sync-btn').addEventListener('click', () => {
            apiCall('/api/sync_current', 'POST');
        });

        document.getElementById('update-interval-btn').addEventListener('click', () => {
            const intervalMs = parseInt(document.getElementById('interval-value').value);
            const intervalSec = intervalMs / 1000.0;
            apiCall('/api/set_interval', 'POST', { interval: intervalSec });
        });

        document.getElementById('update-gains-btn').addEventListener('click', () => {
            const kp = parseInt(document.getElementById('kp-value').value);
            const kd = parseInt(document.getElementById('kd-value').value);
            apiCall('/api/set_gains', 'POST', { kp: kp, kd: kd });
        });

        // Sync interval slider and input
        document.getElementById('interval-slider').addEventListener('input', function() {
            document.getElementById('interval-value').value = this.value;
        });

        document.getElementById('interval-value').addEventListener('input', function() {
            document.getElementById('interval-slider').value = this.value;
        });

        // Sync gain sliders and inputs
        document.getElementById('kp-slider').addEventListener('input', function() {
            document.getElementById('kp-value').value = this.value;
        });

        document.getElementById('kp-value').addEventListener('input', function() {
            document.getElementById('kp-slider').value = this.value;
        });

        document.getElementById('kd-slider').addEventListener('input', function() {
            document.getElementById('kd-value').value = this.value;
        });

        document.getElementById('kd-value').addEventListener('input', function() {
            document.getElementById('kd-slider').value = this.value;
        });

        // Periodic status update
        function updateStatus() {
            apiCall('/api/status');
        }

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            initializeSliders();
            updateStatus();
            // Update status every 500ms
            setInterval(updateStatus, 500);
        });
    </script>
</body>
</html>"""

def main():
    """Main function"""
    print("DuCo Robot ServOJ Web Control Interface")
    print("Starting web server...")
    print("Robot IP: 192.168.1.10:7003")
    print("Web interface will be available at: http://localhost:5000")
    print("Press Ctrl+C to stop the server")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
        robot_controller.disconnect_robot()
    except Exception as e:
        print(f"Server error: {e}")
        robot_controller.disconnect_robot()

if __name__ == '__main__':
    main()
