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
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .header-left h1 {
            margin-bottom: 5px;
        }
        .update-info {
            color: #666;
            font-size: 0.9em;
        }
        .refresh-btn {
            background: #1a73e8;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
            transition: background 0.2s;
        }
        .refresh-btn:hover {
            background: #1557b0;
        }
        .connection-status {
            display: inline-block;
            padding: 4px 12px;
            border-radius: 4px;
            font-size: 0.85em;
            font-weight: 600;
            margin-left: 20px;
        }
        .connection-status.connected {
            background: #e6f4ea;
            color: #137333;
        }
        .connection-status.disconnected {
            background: #fce8e6;
            color: #c5221f;
        }
        .connection-status.connecting {
            background: #fef7e0;
            color: #ea8600;
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
        .status-type {
            font-size: 0.85em;
            color: #666;
            font-weight: normal;
            background: #e8f0fe;
            padding: 2px 8px;
            border-radius: 3px;
            margin-left: 10px;
            font-family: monospace;
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
            <div class="header-left">
                <h1>🤖 Robot Status Dashboard</h1>
                <div class="update-info">
                    Last updated: <span id="last-update">Loading...</span>
                    <span style="margin-left: 20px;">Auto-refresh: <strong>ON</strong> (every 2s)</span>
                    <span class="connection-status connecting" id="connection-status">⚪ Connecting...</span>
                </div>
            </div>
            <button class="refresh-btn" onclick="refreshStatus()">🔄 Refresh Now</button>
        </div>

        <div class="namespace-tabs" id="tabs"></div>
        <div id="content"></div>
    </div>

    <script>
        let currentNamespace = null;
        let statusData = {};

        function updateConnectionStatus(isConnected) {
            const statusElem = document.getElementById('connection-status');
            if (isConnected) {
                statusElem.className = 'connection-status connected';
                statusElem.textContent = '🟢 Connected';
            } else {
                statusElem.className = 'connection-status disconnected';
                statusElem.textContent = '🔴 Disconnected';
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
                    updateConnectionStatus(true);
                } else {
                    updateConnectionStatus(false);
                }
            } catch (error) {
                console.error('Error fetching status:', error);
                updateConnectionStatus(false);
            }
        }

        function renderDashboard() {
            const namespaces = Object.keys(statusData).sort();
            
            if (namespaces.length === 0) {
                document.getElementById('tabs').innerHTML = '';
                document.getElementById('content').innerHTML = 
                    '<div class="empty-state"><h2>No robot status available</h2></div>';
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
                    const item = items[key];
                    let displayValue = '';
                    let typeInfo = '';
                    
                    // Handle new format with type and value
                    if (typeof item === 'object' && item.type && item.value !== undefined) {
                        typeInfo = `<span class="status-type">${item.type}</span>`;
                        
                        // Format the display value
                        if (typeof item.value === 'object') {
                            displayValue = JSON.stringify(item.value, null, 2);
                        } else {
                            displayValue = String(item.value);
                        }
                    } else {
                        // Fallback for old format
                        try {
                            const parsed = JSON.parse(item);
                            displayValue = JSON.stringify(parsed, null, 2);
                        } catch (e) {
                            displayValue = String(item);
                        }
                    }
                    
                    return `
                        <div class="status-item">
                            <div class="status-key">
                                <div>
                                    <span>${key}</span>
                                    ${typeInfo}
                                </div>
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
                import pickle
                import base64
                
                request_msg = ListStatus.Request()
                request_msg.ns = ''
                
                future = self.list_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result():
                    status_dict = json.loads(future.result().status_dict)
                    
                    # Process each value to extract type and displayable content
                    processed_status = {}
                    for namespace, keys in status_dict.items():
                        processed_status[namespace] = {}
                        for key, pickle_str in keys.items():
                            # Unpickle to get the actual object and its type
                            try:
                                # Decode the base64 pickle string
                                pickled = base64.b64decode(pickle_str.encode('ascii'))
                                obj = pickle.loads(pickled)
                                
                                # Get type information
                                type_name = type(obj).__name__
                                module_name = type(obj).__module__
                                if module_name not in ['builtins', '__main__']:
                                    type_str = f"{module_name}.{type_name}"
                                else:
                                    type_str = type_name
                                
                                # Get displayable value
                                display_value = None
                                try:
                                    # Try direct JSON serialization
                                    display_value = obj
                                    json.dumps(display_value)  # Test if serializable
                                except (TypeError, ValueError):
                                    # Try tolist() for numpy arrays
                                    if hasattr(obj, 'tolist'):
                                        display_value = obj.tolist()
                                    elif hasattr(obj, '__dict__'):
                                        # For custom classes, show attributes
                                        display_value = {'_type': type_str, **obj.__dict__}
                                    else:
                                        # Fall back to string representation
                                        display_value = str(obj)
                                
                                processed_status[namespace][key] = {
                                    'type': type_str,
                                    'value': display_value,
                                    'pickle': pickle_str  # Keep pickle for reference
                                }
                            except Exception as e:
                                # If unpickling fails, try to extract type from error message
                                error_msg = str(e)
                                type_str = 'unknown'
                                
                                # Try to extract class name from pickle error
                                # Error format: "Can't get attribute 'ClassName' on <module '__main__'..."
                                if "Can't get attribute" in error_msg:
                                    import re
                                    match = re.search(r"Can't get attribute '(\w+)'", error_msg)
                                    if match:
                                        class_name = match.group(1)
                                        # Try to extract module name too
                                        module_match = re.search(r"on <module '([^']+)'", error_msg)
                                        if module_match:
                                            module_name = module_match.group(1)
                                            type_str = f"{module_name}.{class_name}"
                                        else:
                                            type_str = class_name
                                
                                # Display pickle string with prefix
                                display_value = f"pickle_base64: {pickle_str}"
                                
                                processed_status[namespace][key] = {
                                    'type': type_str,
                                    'value': display_value,
                                    'pickle': pickle_str
                                }
                    
                    return jsonify({
                        'success': True,
                        'namespaces': future.result().namespaces,
                        'status': processed_status
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
