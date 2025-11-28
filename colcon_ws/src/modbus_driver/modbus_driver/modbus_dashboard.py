import threading
import os
from collections import deque
from flask import Flask, render_template, jsonify, request
import logging

class ModbusDashboard:
    def __init__(self, host='0.0.0.0', port=5000):
        self.host = host
        self.port = port
        self.request_logs = deque(maxlen=100)
        self.total_requests_count = 0
        self.log_counter = 0
        
        # Flask App Setup
        template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
        self.app = Flask(__name__, template_folder=template_dir)
        
        # Disable Flask logging to keep console clean
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        self._setup_routes()
        
        self.flask_thread = threading.Thread(target=self._run)
        self.flask_thread.daemon = True

    def _setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('dashboard.html')

        @self.app.route('/api/logs')
        def get_logs():
            after_id = request.args.get('after_id', -1, type=int)
            new_logs = [log for log in self.request_logs if log['id'] > after_id]
            return jsonify({
                'total_requests': self.total_requests_count,
                'logs': new_logs
            })

    def _run(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)

    def start(self):
        self.flask_thread.start()

    def add_log(self, log_entry):
        self.total_requests_count += 1
        log_entry['id'] = self.log_counter
        self.log_counter += 1
        self.request_logs.append(log_entry)
