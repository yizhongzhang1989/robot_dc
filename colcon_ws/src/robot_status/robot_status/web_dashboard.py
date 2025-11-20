#!/usr/bin/env python3
"""
Robot Status Web Dashboard

Provides a Flask-based web interface for viewing and managing robot status.

REST API Endpoints:
    GET  /                      - Web dashboard (HTML)
    GET  /api/status            - Get all status
    GET  /api/status/<namespace> - Get status for specific namespace
    GET  /api/namespaces        - List all namespaces
    POST /api/status            - Set status (JSON: {namespace, key, value})

Usage:
    ros2 run robot_status web_dashboard
    ros2 run robot_status web_dashboard --ros-args -p port:=8005
"""

import rclpy
from rclpy.node import Node
from flask import Flask, jsonify, request, render_template_string
import json
import threading
import sys
import logging


# HTML Template for dashboard
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Robot Status Dashboard</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #f0f2f5;
            padding: 20px;
        }
        .container { max-width: 1400px; margin: 0 auto; }
        h1 { 
            color: #1a73e8;
            margin-bottom: 10px;
            font-size: 2em;
        }
        .header {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        .update-info {
            color: #666;
            font-size: 0.9em;
            margin-top: 5px;
        }
        .controls {
            background: white;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        .controls button {
            background: #1a73e8;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
            margin-right: 10px;
        }
        .controls button:hover {
            background: #1557b0;
        }
        .controls input, .controls select {
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 4px;
            margin-right: 5px;
        }
        .namespace-tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        .tab {
            padding: 12px 24px;
            background: white;
            border: 2px solid #ddd;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.3s;
            font-weight: 500;
        }
        .tab:hover {
            border-color: #1a73e8;
            transform: translateY(-2px);
        }
        .tab.active {
            background: #1a73e8;
            color: white;
            border-color: #1a73e8;
        }
        .namespace-content {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            display: none;
        }
        .namespace-content.active {
            display: block;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .status-item {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 6px;
            border-left: 4px solid #1a73e8;
            position: relative;
        }
        .status-key {
            font-weight: 600;
            color: #333;
            margin-bottom: 8px;
            font-size: 1.1em;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .status-value {
            background: #fff;
            padding: 10px;
            border-radius: 4px;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            max-height: 200px;
            overflow-y: auto;
            white-space: pre-wrap;
            word-break: break-word;
        }
        .delete-btn {
            background: #dc3545;
            color: white;
            border: none;
            padding: 4px 12px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 0.85em;
            transition: background 0.2s;
        }
        .delete-btn:hover {
            background: #c82333;
        }
        .namespace-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 15px;
        }
        .delete-namespace-btn {
            background: #dc3545;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 0.9em;
            transition: background 0.2s;
        }
        .delete-namespace-btn:hover {
            background: #c82333;
        }
        .empty-state {
            text-align: center;
            padding: 40px;
            color: #999;
        }
        .form-group {
            margin-bottom: 10px;
        }
        .form-group label {
            display: inline-block;
            width: 100px;
            font-weight: 500;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 Robot Status Dashboard</h1>
            <div class="update-info">
                Last updated: <span id="last-update">Loading...</span>
                <span style="margin-left: 20px;">Auto-refresh: <strong>ON</strong> (every 2s)</span>
            </div>
        </div>

        <div class="controls">
            <button onclick="refreshStatus()">🔄 Refresh Now</button>
            <button onclick="toggleSetStatus()">➕ Set Status</button>
            <div id="set-status-form" style="display:none; margin-top:15px; padding-top:15px; border-top:1px solid #ddd;">
                <div class="form-group">
                    <label>Namespace:</label>
                    <input type="text" id="new-namespace" placeholder="robot1, shared, etc.">
                </div>
                <div class="form-group">
                    <label>Key:</label>
                    <input type="text" id="new-key" placeholder="pose, battery, etc.">
                </div>
                <div class="form-group">
                    <label>Value (JSON):</label>
                    <input type="text" id="new-value" placeholder='{"x": 1, "y": 2}' style="width:300px;">
                </div>
                <button onclick="submitStatus()">Submit</button>
                <button onclick="toggleSetStatus()">Cancel</button>
            </div>
        </div>

        <div class="namespace-tabs" id="tabs"></div>
        <div id="content"></div>
    </div>

    <script>
        let currentNamespace = null;
        let statusData = {};

        function toggleSetStatus() {
            const form = document.getElementById('set-status-form');
            form.style.display = form.style.display === 'none' ? 'block' : 'none';
        }

        async function submitStatus() {
            const namespace = document.getElementById('new-namespace').value;
            const key = document.getElementById('new-key').value;
            const value = document.getElementById('new-value').value;

            if (!namespace || !key || !value) {
                alert('Please fill all fields');
                return;
            }

            try {
                const response = await fetch('/api/status', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({namespace, key, value})
                });
                const result = await response.json();
                if (result.success) {
                    alert('Status set successfully!');
                    toggleSetStatus();
                    refreshStatus();
                } else {
                    alert('Error: ' + result.message);
                }
            } catch (error) {
                alert('Error: ' + error);
            }
        }

        async function deleteStatus(namespace, key) {
            if (!confirm(`Delete ${namespace}.${key}?`)) {
                return;
            }

            try {
                const response = await fetch(`/api/status/${namespace}/${key}`, {
                    method: 'DELETE'
                });
                const result = await response.json();
                if (result.success) {
                    refreshStatus();
                } else {
                    alert('Error: ' + result.message);
                }
            } catch (error) {
                alert('Error: ' + error);
            }
        }

        async function deleteNamespace(namespace) {
            if (!confirm(`Delete entire namespace '${namespace}' and all its keys?`)) {
                return;
            }

            try {
                const response = await fetch(`/api/namespace/${namespace}`, {
                    method: 'DELETE'
                });
                const result = await response.json();
                if (result.success) {
                    currentNamespace = null;
                    refreshStatus();
                } else {
                    alert('Error: ' + result.message);
                }
            } catch (error) {
                alert('Error: ' + error);
            }
        }

        async function refreshStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                
                if (data.success) {
                    statusData = data.status;
                    renderDashboard();
                    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
                }
            } catch (error) {
                console.error('Error fetching status:', error);
            }
        }

        function renderDashboard() {
            const namespaces = Object.keys(statusData).sort();
            
            if (namespaces.length === 0) {
                document.getElementById('tabs').innerHTML = '';
                document.getElementById('content').innerHTML = 
                    '<div class="empty-state"><h2>No robot status available</h2><p>Use the "Set Status" button to add status data</p></div>';
                return;
            }

            // Set first namespace as active if none selected
            if (!currentNamespace || !namespaces.includes(currentNamespace)) {
                currentNamespace = namespaces[0];
            }

            // Render tabs
            document.getElementById('tabs').innerHTML = namespaces.map(ns => 
                `<div class="tab ${ns === currentNamespace ? 'active' : ''}" onclick="switchNamespace('${ns}')">${ns}</div>`
            ).join('');

            // Render content
            const content = namespaces.map(ns => {
                const items = statusData[ns];
                const keys = Object.keys(items).sort();
                
                const itemsHtml = keys.map(key => {
                    let displayValue = items[key];
                    try {
                        const parsed = JSON.parse(items[key]);
                        displayValue = JSON.stringify(parsed, null, 2);
                    } catch (e) {}
                    
                    return `
                        <div class="status-item">
                            <div class="status-key">
                                <span>${key}</span>
                                <button class="delete-btn" onclick="deleteStatus('${ns}', '${key}')">🗑️ Delete</button>
                            </div>
                            <div class="status-value">${displayValue}</div>
                        </div>
                    `;
                }).join('');

                return `
                    <div class="namespace-content ${ns === currentNamespace ? 'active' : ''}" data-namespace="${ns}">
                        <div class="namespace-header">
                            <h2>${ns}</h2>
                            <button class="delete-namespace-btn" onclick="deleteNamespace('${ns}')">🗑️ Delete Namespace</button>
                        </div>
                        <div class="status-grid">${itemsHtml || '<div class="empty-state">No status items</div>'}</div>
                    </div>
                `;
            }).join('');

            document.getElementById('content').innerHTML = content;
        }

        function switchNamespace(namespace) {
            currentNamespace = namespace;
            renderDashboard();
        }

        // Auto-refresh every 2 seconds
        setInterval(refreshStatus, 2000);
        
        // Initial load
        refreshStatus();
    </script>
</body>
</html>
'''


class WebDashboardNode(Node):
    """ROS2 node that runs Flask web dashboard."""
    
    def __init__(self):
        super().__init__('robot_status_web_client')
        
        # Declare parameters
        self.declare_parameter('port', 8005)
        self.declare_parameter('host', '0.0.0.0')
        
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        
        # Import service types
        try:
            from robot_status.srv import SetStatus, GetStatus, ListStatus, DeleteStatus
            
            # Create service clients
            self.set_client = self.create_client(SetStatus, 'robot_status/set')
            self.get_client = self.create_client(GetStatus, 'robot_status/get')
            self.list_client = self.create_client(ListStatus, 'robot_status/list')
            self.delete_client = self.create_client(DeleteStatus, 'robot_status/delete')
            
            # Wait for services
            self.get_logger().info("Waiting for robot_status services...")
            if not self.set_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("robot_status/set service not available!")
                sys.exit(1)
            if not self.list_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("robot_status/list service not available!")
                sys.exit(1)
            if not self.delete_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("robot_status/delete service not available!")
                sys.exit(1)
                
            self.get_logger().info("Connected to robot_status services")
            
        except ImportError as e:
            self.get_logger().error(f"Failed to import service types: {e}")
            sys.exit(1)
        
        # Create Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Robot Status Web Dashboard")
        self.get_logger().info(f"Access at: http://{self.host}:{self.port}")
        self.get_logger().info("=" * 60)
    
    def setup_routes(self):
        """Setup Flask routes."""
        
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        @self.app.route('/api/status', methods=['GET'])
        def get_all_status():
            try:
                from robot_status.srv import ListStatus
                request_msg = ListStatus.Request()
                request_msg.ns = ''
                
                future = self.list_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result():
                    return jsonify({
                        'success': True,
                        'namespaces': future.result().namespaces,
                        'status': json.loads(future.result().status_dict)
                    })
                return jsonify({'success': False, 'error': 'Service call failed'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/status/<namespace>', methods=['GET'])
        def get_namespace_status(namespace):
            try:
                from robot_status.srv import ListStatus
                request_msg = ListStatus.Request()
                request_msg.ns = namespace
                
                future = self.list_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result():
                    return jsonify({
                        'success': True,
                        'status': json.loads(future.result().status_dict)
                    })
                return jsonify({'success': False, 'error': 'Service call failed'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/namespaces', methods=['GET'])
        def get_namespaces():
            try:
                from robot_status.srv import ListStatus
                request_msg = ListStatus.Request()
                request_msg.ns = ''
                
                future = self.list_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result():
                    return jsonify({
                        'success': True,
                        'namespaces': future.result().namespaces
                    })
                return jsonify({'success': False, 'error': 'Service call failed'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/status', methods=['POST'])
        def set_status():
            try:
                from robot_status.srv import SetStatus
                data = request.json
                request_msg = SetStatus.Request()
                request_msg.ns = data['namespace']
                request_msg.key = data['key']
                request_msg.value = data['value']
                
                future = self.set_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result():
                    return jsonify({
                        'success': future.result().success,
                        'message': future.result().message
                    })
                return jsonify({'success': False, 'message': 'Service call failed'})
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/api/status/<namespace>/<key>', methods=['DELETE'])
        def delete_status(namespace, key):
            try:
                from robot_status.srv import DeleteStatus
                request_msg = DeleteStatus.Request()
                request_msg.ns = namespace
                request_msg.key = key
                
                future = self.delete_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result():
                    return jsonify({
                        'success': future.result().success,
                        'message': future.result().message
                    })
                return jsonify({'success': False, 'message': 'Service call failed'})
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/api/namespace/<namespace>', methods=['DELETE'])
        def delete_namespace(namespace):
            try:
                from robot_status.srv import DeleteStatus
                request_msg = DeleteStatus.Request()
                request_msg.ns = namespace
                request_msg.key = ''  # Empty key = delete entire namespace
                
                future = self.delete_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result():
                    return jsonify({
                        'success': future.result().success,
                        'message': future.result().message
                    })
                return jsonify({'success': False, 'message': 'Service call failed'})
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)})
    
    def run_flask(self):
        """Run Flask app."""
        # Disable Flask access logs
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = WebDashboardNode()
        
        # Run Flask in separate thread
        flask_thread = threading.Thread(target=node.run_flask, daemon=True)
        flask_thread.start()
        
        # Spin ROS2 node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
