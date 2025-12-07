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
    python3 web_dashboard.py --host 0.0.0.0 --port 8005
"""

from flask import Flask, jsonify, request, render_template
import json
import sys
import logging
import pickle
import base64
import os
import argparse
import math

try:
    from robot_status_redis.redis_backend import get_redis_backend, is_redis_available
    REDIS_AVAILABLE = True
except ImportError:
    try:
        from redis_backend import get_redis_backend, is_redis_available
        REDIS_AVAILABLE = True
    except ImportError:
        REDIS_AVAILABLE = False


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(name)s]: %(message)s'
)
logger = logging.getLogger('robot_status_web')


class WebDashboard:
    """Flask web dashboard with Redis backend."""
    
    def __init__(self, host='0.0.0.0', port=8005):
        self.host = host
        self.port = port
        
        # Initialize Redis backend
        if not REDIS_AVAILABLE:
            error_msg = (
                "Redis module not installed. Install with:\n"
                "  pip3 install redis"
            )
            logger.error(error_msg)
            sys.exit(1)
        
        if not is_redis_available():
            error_msg = (
                "Redis server not available. Start Redis with:\n"
                "  sudo apt-get install redis-server\n"
                "  sudo systemctl start redis-server\n"
                "Verify with: redis-cli ping"
            )
            logger.error(error_msg)
            sys.exit(1)
        
        self._redis_backend = get_redis_backend()
        if self._redis_backend is None:
            logger.error("Failed to connect to Redis")
            sys.exit(1)
        
        logger.info("✓ Connected to Redis backend")
        
        # Create Flask app with template and static folders
        package_dir = os.path.dirname(__file__)
        template_dir = os.path.join(package_dir, 'templates')
        static_dir = os.path.join(package_dir, 'static')
        
        self.app = Flask(__name__, 
                        template_folder=template_dir,
                        static_folder=static_dir)
        self.setup_routes()
        
        logger.info("=" * 60)
        logger.info("Robot Status Web Dashboard")
        logger.info("Backend: Redis")
        logger.info(f"Access at: http://{self.host}:{self.port}")
        logger.info("=" * 60)
    
    def _sanitize_value(self, value):
        """Sanitize value to be JSON-serializable, converting NaN/Inf to None."""
        if isinstance(value, float):
            if math.isnan(value) or math.isinf(value):
                return None
        elif isinstance(value, (list, tuple)):
            return [self._sanitize_value(v) for v in value]
        elif isinstance(value, dict):
            return {k: self._sanitize_value(v) for k, v in value.items()}
        return value
    
    def _process_status_dict(self, status_dict):
        """Process status dict to extract type and displayable content."""
        processed_status = {}
        
        for namespace, keys in status_dict.items():
            processed_status[namespace] = {}
            for key, data in keys.items():
                # Extract pickle_str and timestamp from data dict
                if isinstance(data, dict):
                    pickle_str = data.get('pickle')
                    timestamp = data.get('timestamp')
                else:
                    # Fallback for old format (should not happen with new implementation)
                    pickle_str = data
                    timestamp = None
                
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
                    
                    # Sanitize the value to remove NaN/Inf
                    display_value = self._sanitize_value(display_value)
                    
                    processed_status[namespace][key] = {
                        'type': type_str,
                        'value': display_value,
                        'pickle': pickle_str,  # Keep pickle for reference
                        'timestamp': timestamp  # Include timestamp for age display
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
                        'pickle': pickle_str,
                        'timestamp': timestamp  # Include timestamp for age display
                    }
        
        return processed_status
    
    def _list_status(self, namespace=None):
        """List status from Redis backend."""
        return self._redis_backend.list_status(namespace)
    
    def _delete_status(self, namespace, key=''):
        """Delete status from Redis backend."""
        if key:
            return self._redis_backend.delete_status(namespace, key)
        else:
            # Delete entire namespace
            storage_tree = self._redis_backend.list_status(namespace)
            if namespace in storage_tree:
                success = True
                for k in storage_tree[namespace].keys():
                    if not self._redis_backend.delete_status(namespace, k):
                        success = False
                return success
            return False
    
    def setup_routes(self):
        """Setup Flask routes."""
        
        @self.app.route('/')
        def index():
            return render_template('dashboard.html')
        
        @self.app.route('/api/status', methods=['GET'])
        def get_all_status():
            try:
                storage_tree = self._list_status()
                namespaces = list(storage_tree.keys())
                processed_status = self._process_status_dict(storage_tree)
                
                return jsonify({
                    'success': True,
                    'namespaces': namespaces,
                    'status': processed_status
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/status/<namespace>', methods=['GET'])
        def get_namespace_status(namespace):
            try:
                storage_tree = self._list_status(namespace)
                return jsonify({
                    'success': True,
                    'status': storage_tree
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/namespaces', methods=['GET'])
        def get_namespaces():
            try:
                storage_tree = self._list_status()
                namespaces = list(storage_tree.keys())
                return jsonify({
                    'success': True,
                    'namespaces': namespaces
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        

        
        @self.app.route('/api/status/<namespace>/<key>', methods=['DELETE'])
        def delete_status(namespace, key):
            try:
                success = self._delete_status(namespace, key)
                message = f"Deleted {namespace}.{key}" if success else "Delete failed"
                return jsonify({
                    'success': success,
                    'message': message
                })
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/api/namespace/<namespace>', methods=['DELETE'])
        def delete_namespace(namespace):
            try:
                success = self._delete_status(namespace, '')
                message = f"Deleted namespace {namespace}" if success else "Delete failed"
                return jsonify({
                    'success': success,
                    'message': message
                })
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)})
    
    def run(self):
        """Run Flask app."""
        # Disable Flask access logs
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Robot Status Web Dashboard')
    parser.add_argument('--host', default='0.0.0.0', help='Host address (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8005, help='Port number (default: 8005)')
    args = parser.parse_args()
    
    try:
        dashboard = WebDashboard(host=args.host, port=args.port)
        dashboard.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.error(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
