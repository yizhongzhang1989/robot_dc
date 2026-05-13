#!/usr/bin/env python3
"""
Workflow API Server - Provides CRUD operations for workflow files
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
import json
from pathlib import Path
from common.workspace_utils import get_temp_directory, get_workspace_root

app = Flask(__name__, static_folder='.', static_url_path='')
CORS(app)  # Enable CORS

# Workflow configuration directory
WORKFLOW_CONFIG_DIR = Path(get_workspace_root()) / 'colcon_ws' / 'src' / 'ur_workflow' / 'examples'

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

# Robot client registry — populated at startup from --robot CLI args (one
# entry per UR robot in robot_config.yaml). Each entry is a dict with
# 'ip' and 'port' string values; the actual UR15Robot socket is lazy-opened
# on first /api/<robot_name>/actual_joint_positions request and cached
# under 'robot'. This service is shared by every dashboard, so we must
# never assume only one robot is connected — historically the editor read
# joints from a hard-coded ur15 connection, which silently fed ur15 joints
# into ur10e workflows.
robot_clients = {}  # robot_name → {'ip': str, 'port': int, 'robot': UR15Robot|None}
# Legacy alias for the previous single-robot endpoint, set to whichever
# entry was configured with --robot ur15 (or the first one if no ur15).
_legacy_default_robot_name = None


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


@app.route('/api/robots', methods=['GET'])
def list_robots():
    """List configured robots that joint-position queries can be sent to."""
    return jsonify({
        'success': True,
        'robots': [
            {'name': name, 'ip': entry['ip'], 'port': entry['port']}
            for name, entry in robot_clients.items()
        ],
        'default': _legacy_default_robot_name,
    })


# Workflow-context defaults that the editor falls back on if the robot's
# namespace is unknown or robot_status_redis is unreachable. Historically
# the editor was ur15-only so these mirror the legacy hardcoded values.
# Note: positioning_service_url is intentionally NOT included — the
# workflow runner now resolves it from robot_config.yaml at launch time
# (see runner._resolve_positioning_service_url), so embedding it in every
# new workflow JSON only creates drift between the JSON and the config.
_FALLBACK_CONTEXT_DEFAULTS = {
    'status_namespace': 'ur15',
    'robot_ip': '192.168.1.15',
    'robot_port': 30002,
    'robot_type': 'ur15',
    'camera_topic': '/ur15_camera/image_raw',
}


@app.route('/api/<robot_name>/context_defaults', methods=['GET'])
def get_context_defaults(robot_name):
    """Return per-robot defaults for a workflow's top-level ``context`` block.

    The editor calls this at load time (with the active robot's namespace
    derived from ``?robot=<ns>``) so that "New Workflow", template loads
    and drop-onto-empty paths can pre-fill the JSON with the right values
    for the robot the dashboard is editing for.

    Resolution order per key:
      1. value stored in robot_status_redis under ``robot_name`` namespace
         (populated at launch time by each web node's
         ``_publish_static_robot_info``)
      2. ``_FALLBACK_CONTEXT_DEFAULTS`` (ur15-shaped) — used when no client
         exists, the value is missing, or the lookup raises.

    The response always returns a populated ``defaults`` dict and a
    per-key ``sources`` map ('redis' or 'fallback') so the UI can surface
    which values are real and which are guesses.
    """
    defaults = dict(_FALLBACK_CONTEXT_DEFAULTS)
    sources = {k: 'fallback' for k in defaults}
    # status_namespace is the only key the editor knows for certain: it's
    # whatever the dashboard passed us via ?robot=...
    defaults['status_namespace'] = robot_name
    sources['status_namespace'] = 'request'

    # positioning_service_url is intentionally omitted: it's resolved by
    # the workflow runner from robot_config.yaml at launch time, not
    # baked per-workflow.

    redis_backed_keys = ('robot_ip', 'robot_port', 'robot_type', 'camera_topic')

    if robot_status_client is None:
        return jsonify({
            'success': True,
            'robot': robot_name,
            'defaults': defaults,
            'sources': sources,
            'warning': 'robot_status_client unavailable; using fallback values',
        })

    for key in redis_backed_keys:
        try:
            value = robot_status_client.get_status(robot_name, key)
        except Exception as exc:  # noqa: BLE001
            print(f"⚠ context_defaults: get_status({robot_name}, {key}) raised: {exc}")
            continue
        if value is None or value == '':
            continue
        # robot_port is stored as int; everything else is a plain string. We
        # don't attempt smart coercion beyond that — the editor consumes the
        # values verbatim.
        defaults[key] = value
        sources[key] = 'redis'

    return jsonify({
        'success': True,
        'robot': robot_name,
        'defaults': defaults,
        'sources': sources,
    })


def _get_or_open_robot(robot_name: str):
    """Return a connected UR15Robot for ``robot_name`` or (None, error_msg)."""
    entry = robot_clients.get(robot_name)
    if entry is None:
        return None, f"Robot '{robot_name}' is not configured on this service"

    robot = entry.get('robot')
    if robot is None:
        # Lazy import — keeps the service importable on hosts without
        # ur_robot_arm installed (e.g. during unit testing).
        try:
            from ur_robot_arm.ur15 import UR15Robot
        except Exception as exc:  # noqa: BLE001
            return None, f"UR15Robot module unavailable: {exc}"
        robot = UR15Robot(entry['ip'], entry['port'])
        entry['robot'] = robot

    if not getattr(robot, 'connected', False):
        rc = robot.open()
        if rc != 0:
            return None, f"Failed to connect to robot '{robot_name}' at {entry['ip']}:{entry['port']}"
    return robot, None


@app.route('/api/<robot_name>/actual_joint_positions', methods=['GET'])
def get_actual_joint_positions(robot_name):
    """Get actual joint positions for ``robot_name`` (in radians).

    ``robot_name`` is the same key used in robot_config.yaml (e.g. 'ur15',
    'ur10e'). The legacy ``/api/ur15/actual_joint_positions`` route remains
    available — it maps to whichever robot the service was launched with
    as ur15.
    """
    try:
        robot, err = _get_or_open_robot(robot_name)
        if robot is None:
            return jsonify({'success': False, 'error': err}), 503

        joint_positions = robot.get_actual_joint_positions()
        if joint_positions is None:
            return jsonify({'success': False, 'error': 'Failed to read joint positions from robot'}), 500

        import numpy as np
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()

        return jsonify({
            'success': True,
            'robot': robot_name,
            'value': joint_positions,
            'unit': 'radians',
            'description': f'Actual joint positions from {robot_name} robot',
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
    parser.add_argument(
        '--robot',
        action='append',
        default=[],
        metavar='NAME:IP:PORT',
        help='Register a robot for joint-position queries. May be repeated. '
             'Format: name:ip[:port] (port defaults to 30002). The "name" '
             'must match the robot key in robot_config.yaml and the URL '
             'path /api/<name>/actual_joint_positions used by the editor. '
             'Example: --robot ur15:192.168.1.15 --robot ur10e:192.168.1.16'
    )
    # Legacy single-robot CLI (still respected; kept so older launch files
    # don't break). Effectively equivalent to --robot ur15:<ip>:<port>.
    parser.add_argument('--ur15-ip', type=str, default=None,
                        help='[Deprecated, use --robot] UR15 robot IP address')
    parser.add_argument('--ur15-port', type=int, default=30002,
                        help='[Deprecated, use --robot] UR15 robot control port')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    
    args = parser.parse_args()

    # Build the robot registry from --robot occurrences plus the legacy
    # --ur15-ip flag.
    def _add_robot(name: str, ip: str, port: int):
        if not name or not ip:
            return
        if name in robot_clients:
            print(f"⚠ Duplicate --robot {name}, keeping first entry")
            return
        robot_clients[name] = {'ip': ip, 'port': int(port), 'robot': None}
        print(f"✓ Registered robot '{name}' at {ip}:{port}")

    for spec in args.robot:
        parts = spec.split(':')
        if len(parts) == 2:
            name, ip = parts
            port = 30002
        elif len(parts) == 3:
            name, ip, port_s = parts
            try:
                port = int(port_s)
            except ValueError:
                print(f"✗ Bad --robot spec '{spec}': port must be an int. Skipping.")
                continue
        else:
            print(f"✗ Bad --robot spec '{spec}'. Expected name:ip[:port]. Skipping.")
            continue
        _add_robot(name, ip, port)

    if args.ur15_ip:
        _add_robot('ur15', args.ur15_ip, args.ur15_port)

    if robot_clients:
        # Prefer 'ur15' if registered; otherwise fall back to the first robot.
        _legacy_default_robot_name = 'ur15' if 'ur15' in robot_clients else next(iter(robot_clients))
    else:
        print("⚠ No robots registered. /api/<name>/actual_joint_positions will return 503.")
        print("  Use --robot NAME:IP[:PORT] (or legacy --ur15-ip) to enable joint readback.")

    print(f"Workflow API Server starting...")
    print(f"Workflow config directory: {WORKFLOW_CONFIG_DIR}")
    print(f"Registered robots: {list(robot_clients.keys()) or '(none)'}")
    print(f"Server running on http://{args.host}:{args.port}")
    print(f"Open browser: http://localhost:{args.port}")
    
    app.run(host=args.host, port=args.port, debug=args.debug)
