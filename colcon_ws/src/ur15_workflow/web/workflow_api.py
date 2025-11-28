#!/usr/bin/env python3
"""
Workflow API Server - Provides CRUD operations for workflow files
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import os
import json
from pathlib import Path

app = Flask(__name__)
CORS(app)  # Enable CORS

# Workflow configuration directory
WORKFLOW_CONFIG_DIR = Path(__file__).parent.parent.parent / 'ur15_workflow' / 'config'

# Initialize RobotStatusClient for Redis access
robot_status_client = None
try:
    from robot_status_redis.client_utils import RobotStatusClient
    robot_status_client = RobotStatusClient()
    print("✓ RobotStatusClient initialized")
except Exception as e:
    print(f"✗ Failed to initialize RobotStatusClient: {e}")
    print("  Robot status endpoints will not be available")


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
        
        # Build full path
        filepath = WORKFLOW_CONFIG_DIR / filename
        
        # Check if file already exists
        if filepath.exists():
            return jsonify({'success': False, 'error': 'File already exists'}), 400
        
        # Ensure directory exists
        WORKFLOW_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        
        # Write to file
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(content, f, indent=2, ensure_ascii=False)
        
        return jsonify({
            'success': True,
            'message': f'Workflow file created: {filename}',
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
        
        # Build full path
        filepath = WORKFLOW_CONFIG_DIR / filename
        
        # Ensure directory exists
        WORKFLOW_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        
        # Write to file
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(content, f, indent=2, ensure_ascii=False)
        
        return jsonify({
            'success': True,
            'message': f'Workflow file saved: {filename}',
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
        filepath = WORKFLOW_CONFIG_DIR / filename
        
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
        if not WORKFLOW_CONFIG_DIR.exists():
            return jsonify({'success': True, 'files': []})
        
        files = [f.name for f in WORKFLOW_CONFIG_DIR.glob('*.json')]
        
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
        filepath = WORKFLOW_CONFIG_DIR / filename
        
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


if __name__ == '__main__':
    print(f"Workflow API Server starting...")
    print(f"Workflow config directory: {WORKFLOW_CONFIG_DIR}")
    print(f"Server running on http://localhost:8008")
    app.run(host='0.0.0.0', port=8008, debug=True)
