#!/usr/bin/env python3
"""
Workflow API Server - Provides CRUD operations for workflow files
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
import json
from pathlib import Path
from common.workspace_utils import get_temp_directory

app = Flask(__name__, static_folder='.', static_url_path='')
CORS(app)  # Enable CORS

# Workflow configuration directory
WORKFLOW_CONFIG_DIR = Path(__file__).parent.parent.parent / 'ur15_workflow' / 'examples'

# Workflow files directory for new workflow creation
WORKFLOW_FILES_PATH = Path(get_temp_directory()) / 'workflow_files'
# Ensure directory exists at startup
WORKFLOW_FILES_PATH.mkdir(parents=True, exist_ok=True)

# Initialize RobotStatusClient for Redis access
robot_status_client = None
try:
    from robot_status_redis.client_utils import RobotStatusClient
    robot_status_client = RobotStatusClient()
    print("✓ RobotStatusClient initialized")
except Exception as e:
    print(f"✗ Failed to initialize RobotStatusClient: {e}")
    print("  Robot status endpoints will not be available")

# Initialize UR15Robot for direct robot communication
ur15_robot = None
try:
    import sys
    import os
    # Add ur15_robot_arm package to path
    current_dir = Path(__file__).parent.parent.parent
    ur15_pkg_path = current_dir / 'ur15_robot_arm'
    if ur15_pkg_path.exists():
        sys.path.insert(0, str(ur15_pkg_path))
    
    from ur15_robot_arm.ur15 import UR15Robot
    # Default UR15 connection parameters (can be overridden via env variables)
    ur15_ip = os.environ.get('UR15_IP', '192.168.1.15')
    ur15_port = int(os.environ.get('UR15_PORT', '30002'))
    ur15_robot = UR15Robot(ur15_ip, ur15_port)
    print(f"✓ UR15Robot initialized (IP: {ur15_ip}, Port: {ur15_port})")
except Exception as e:
    print(f"✗ Failed to initialize UR15Robot: {e}")
    print("  Direct robot joint position endpoint will not be available")


@app.route('/api/workflow', methods=['POST'])
def create_workflow():
    """Create a new workflow file"""
    try:
        data = request.get_json()
        filename = data.get('fileName')
        content = data.get('content')
        
        if not filename:
            return jsonify({'success': False, 'error': 'Filename cannot be empty'}), 400
        
        # Ensure filename ends with .jsonme ends with .json
        if not filename.endswith('.json'):
            filename += '.json'
        
        # Build full path using WORKFLOW_FILES_PATH for new workflow creation
        filepath = WORKFLOW_FILES_PATH / filename
        
        # Check if file already exists
        if filepath.exists():
            return jsonify({'success': False, 'error': 'File already exists'}), 400
        
        # Ensure directory exists
        WORKFLOW_FILES_PATH.mkdir(parents=True, exist_ok=True)
        
        # Write to file
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(content, f, indent=2, ensure_ascii=False)
        
        return jsonify({
            'success': True,
            'message': f'Workflow created at: {filepath}',
            'filepath': str(filepath)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/workflow', methods=['PUT'])
def save_workflow():
    """Save (update) workflow file"""
    try:
        data = request.get_json()
        filename = data.get('fileName')
        content = data.get('content')
        
        if not filename:
            return jsonify({'success': False, 'error': 'Filename cannot be empty'}), 400
        
        # Ensure filename ends with .json
        if not filename.endswith('.json'):
            filename += '.json'
        
        # Build full path using WORKFLOW_FILES_PATH
        filepath = WORKFLOW_FILES_PATH / filename
        
        # Ensure directory exists
        WORKFLOW_FILES_PATH.mkdir(parents=True, exist_ok=True)
        
        # Write to file
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(content, f, indent=2, ensure_ascii=False)
        
        return jsonify({
            'success': True,
            'message': f'Workflow saved to: {filepath}',
            'filepath': str(filepath)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/workflow/<filename>', methods=['DELETE'])
def delete_workflow(filename):
    """Delete workflow file"""
    try:
        # Ensure filename ends with .json
        if not filename.endswith('.json'):
            filename += '.json'
        
        # Build full path
        filepath = WORKFLOW_FILES_PATH / filename
        
        # Check if file exists
        if not filepath.exists():
            return jsonify({'success': False, 'error': 'File not found'}), 404
        
        # Delete file
        filepath.unlink()
        
        return jsonify({
            'success': True,
            'message': f'Workflow file deleted: {filename}'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/workflows', methods=['GET'])
def list_workflows():
    """List all workflow files"""
    try:
        # Ensure directory exists
        WORKFLOW_FILES_PATH.mkdir(parents=True, exist_ok=True)
        
        files = [f.name for f in WORKFLOW_FILES_PATH.glob('*.json')]
        
        return jsonify({
            'success': True,
            'files': sorted(files)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/workflow/<filename>', methods=['GET'])
def get_workflow(filename):
    """Get workflow file content"""
    try:
        # Ensure filename ends with .json
        if not filename.endswith('.json'):
            filename += '.json'
        
        # Build full path
        filepath = WORKFLOW_FILES_PATH / filename
        
        # Check if file exists
        if not filepath.exists():
            return jsonify({'success': False, 'error': 'File not found'}), 404
        
        # Read file
        with open(filepath, 'r', encoding='utf-8') as f:
            content = json.load(f)
        
        return jsonify({
            'success': True,
            'content': content,
            'filename': filename
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/templates', methods=['GET'])
def list_templates():
    """List all template files from WORKFLOW_CONFIG_DIR"""
    try:
        if not WORKFLOW_CONFIG_DIR.exists():
            return jsonify({'success': True, 'files': []})
        
        files = [f.name for f in WORKFLOW_CONFIG_DIR.glob('*.json')]
        
        return jsonify({
            'success': True,
            'files': sorted(files)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/template/<filename>', methods=['GET'])
def get_template(filename):
    """Get template file content from WORKFLOW_CONFIG_DIR"""
    try:
        # Ensure filename ends with .json
        if not filename.endswith('.json'):
            filename += '.json'
        
        # Build full path
        filepath = WORKFLOW_CONFIG_DIR / filename
        
        # Check if file exists
        if not filepath.exists():
            return jsonify({'success': False, 'error': 'Template not found'}), 404
        
        # Read file
        with open(filepath, 'r', encoding='utf-8') as f:
            content = json.load(f)
        
        return jsonify({
            'success': True,
            'content': content,
            'filename': filename
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/robot_status/<namespace>/<key>', methods=['GET'])
def get_robot_status(namespace, key):
    """Get robot status from Redis"""
    try:
        if robot_status_client is None:
            return jsonify({'success': False, 'error': 'RobotStatusClient not available'}), 503
        
        value = robot_status_client.get_status(namespace, key)
        
        if value is None:
            return jsonify({'success': False, 'error': f'Status not found: {namespace}/{key}'}), 404
        
        # Convert numpy arrays to lists for JSON serialization
        import numpy as np
        if isinstance(value, np.ndarray):
            value = value.tolist()
        
        return jsonify({
            'success': True,
            'namespace': namespace,
            'key': key,
            'value': value
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/ur15/actual_joint_positions', methods=['GET'])
def get_actual_joint_positions():
    """Get actual joint positions directly from UR15 robot (in radians)"""
    try:
        if ur15_robot is None:
            return jsonify({'success': False, 'error': 'UR15Robot not available'}), 503
        
        # Open connection if not already connected
        if not ur15_robot.connected:
            result = ur15_robot.open()
            if result != 0:
                return jsonify({'success': False, 'error': 'Failed to connect to robot'}), 503
        
        # Get actual joint positions (already in radians)
        joint_positions = ur15_robot.get_actual_joint_positions()
        
        if joint_positions is None:
            return jsonify({'success': False, 'error': 'Failed to read joint positions from robot'}), 500
        
        # Convert numpy array to list for JSON serialization
        import numpy as np
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()
        
        return jsonify({
            'success': True,
            'value': joint_positions,
            'unit': 'radians',
            'description': 'Actual joint positions from UR15 robot'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/')
def index():
    """Serve the main HTML page"""
    response = send_from_directory('.', 'workflow_index.html')
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response


@app.route('/<path:path>')
def serve_static(path):
    """Serve static files (HTML, JS, CSS, etc.)"""
    response = send_from_directory('.', path)
    # Disable cache for JS and CSS files
    if path.endswith('.js') or path.endswith('.css') or path.endswith('.json'):
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
    return response


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Workflow Config Center API Server')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host address')
    parser.add_argument('--port', type=int, default=8008, help='Port number')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    
    args = parser.parse_args()
    
    print(f"Workflow API Server starting...")
    print(f"Workflow config directory: {WORKFLOW_CONFIG_DIR}")
    print(f"Server running on http://{args.host}:{args.port}")
    print(f"Open browser: http://localhost:{args.port}")
    
    app.run(host=args.host, port=args.port, debug=args.debug)
