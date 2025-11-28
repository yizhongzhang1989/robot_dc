import threading
import os
from collections import deque
from queue import Queue
from flask import Flask, render_template, jsonify, request, send_file
from flask_socketio import SocketIO
import logging
import json
from datetime import datetime

try:
    from common.workspace_utils import get_temp_directory
    DEFAULT_LOG_DIR = os.path.join(get_temp_directory(), 'modbus_logs')
except Exception:
    DEFAULT_LOG_DIR = 'modbus_logs'

class ModbusDashboard:
    def __init__(self, host='0.0.0.0', port=5000, log_dir=None, batch_interval=0.1, buffer_size=100):
        self.host = host
        self.port = port
        self.request_logs = deque(maxlen=100)  # Keep last 100 in memory for display
        self.total_requests_count = 0
        self.log_counter = 0
        
        # Batching configuration
        self.batch_interval = batch_interval  # seconds between batch emissions
        self.buffer_size = buffer_size  # max logs before forced flush
        
        # Queue for non-blocking add_log
        self.log_queue = Queue(maxsize=10000)  # Large queue to prevent blocking
        
        # Batching buffers
        self.socket_batch = []
        self.socket_batch_lock = threading.Lock()
        self.file_buffer = []
        self.file_buffer_lock = threading.Lock()
        
        # Setup log directory and file
        if log_dir is None:
            log_dir = DEFAULT_LOG_DIR
        self.log_dir = os.path.abspath(log_dir)
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create log file with timestamp
        log_filename = f"modbus_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        self.log_file_path = os.path.join(self.log_dir, log_filename)
        self.log_file_handle = open(self.log_file_path, 'a', buffering=8192)
        self.log_file_lock = threading.Lock()
        
        # Start batch processing threads
        self.running = True
        self.queue_processor_thread = threading.Thread(target=self._queue_processor)
        self.queue_processor_thread.daemon = True
        self.socket_batch_thread = threading.Thread(target=self._socket_batch_worker)
        self.socket_batch_thread.daemon = True
        self.file_batch_thread = threading.Thread(target=self._file_batch_worker)
        self.file_batch_thread.daemon = True
        
        # Flask App Setup
        template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config['SECRET_KEY'] = 'modbus_dashboard_secret'
        
        # Socket.IO Setup
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Disable Flask logging to keep console clean
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        log = logging.getLogger('socketio')
        log.setLevel(logging.ERROR)
        log = logging.getLogger('engineio')
        log.setLevel(logging.ERROR)

        self._setup_routes()
        
        self.flask_thread = threading.Thread(target=self._run)
        self.flask_thread.daemon = True
        
        # Start batch workers
        self.queue_processor_thread.start()
        self.socket_batch_thread.start()
        self.file_batch_thread.start()

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
        
        @self.app.route('/api/download_log')
        def download_log():
            if os.path.exists(self.log_file_path):
                return send_file(self.log_file_path, as_attachment=True, download_name=os.path.basename(self.log_file_path))
            else:
                return jsonify({'error': 'Log file not found'}), 404
        
        @self.app.route('/api/log_info')
        def log_info():
            if os.path.exists(self.log_file_path):
                file_size = os.path.getsize(self.log_file_path)
                return jsonify({
                    'filename': os.path.basename(self.log_file_path),
                    'size': file_size,
                    'size_mb': round(file_size / 1024 / 1024, 2)
                })
            else:
                return jsonify({'error': 'Log file not found'}), 404

    def _queue_processor(self):
        """Worker thread that processes log queue (non-blocking for Modbus)"""
        while self.running:
            try:
                log_entry = self.log_queue.get(timeout=0.1)
                
                # Add to file buffer
                with self.file_buffer_lock:
                    self.file_buffer.append(log_entry)
                    # Force flush if buffer is full
                    if len(self.file_buffer) >= self.buffer_size:
                        try:
                            with self.log_file_lock:
                                for entry in self.file_buffer:
                                    self.log_file_handle.write(json.dumps(entry) + '\n')
                                self.log_file_handle.flush()
                            self.file_buffer.clear()
                        except Exception as e:
                            print(f"Error flushing file buffer: {e}")
                
                # Keep last 100 in memory for real-time display
                self.request_logs.append(log_entry)
                
                # Add to Socket.IO batch
                with self.socket_batch_lock:
                    self.socket_batch.append(log_entry)
                    # Force emit if batch is full
                    if len(self.socket_batch) >= self.buffer_size:
                        self.socketio.emit('new_logs_batch', {
                            'logs': self.socket_batch[:],
                            'total_requests': self.total_requests_count
                        })
                        self.socket_batch.clear()
                
            except Exception:
                # Queue timeout, continue
                pass
    
    def _socket_batch_worker(self):
        """Worker thread that batches Socket.IO emissions"""
        import time
        while self.running:
            time.sleep(self.batch_interval)
            with self.socket_batch_lock:
                if self.socket_batch:
                    # Emit batch of logs
                    self.socketio.emit('new_logs_batch', {
                        'logs': self.socket_batch[:],
                        'total_requests': self.total_requests_count
                    })
                    self.socket_batch.clear()
    
    def _file_batch_worker(self):
        """Worker thread that batches file writes"""
        import time
        while self.running:
            time.sleep(self.batch_interval)
            with self.file_buffer_lock:
                if self.file_buffer:
                    try:
                        with self.log_file_lock:
                            for log_entry in self.file_buffer:
                                self.log_file_handle.write(json.dumps(log_entry) + '\n')
                            self.log_file_handle.flush()
                        self.file_buffer.clear()
                    except Exception as e:
                        print(f"Error writing batch to log file: {e}")
    
    def _run(self):
        self.socketio.run(self.app, host=self.host, port=self.port, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)

    def start(self):
        self.flask_thread.start()
    
    def stop(self):
        """Clean shutdown of batch workers and file handle"""
        self.running = False
        if hasattr(self, 'log_file_handle'):
            with self.log_file_lock:
                if self.file_buffer:
                    for log_entry in self.file_buffer:
                        self.log_file_handle.write(json.dumps(log_entry) + '\n')
                self.log_file_handle.flush()
                self.log_file_handle.close()

    def add_log(self, log_entry):
        """Non-blocking add_log - just puts to queue (O(1) operation)"""
        self.total_requests_count += 1
        log_entry['id'] = self.log_counter
        self.log_counter += 1
        
        # Non-blocking queue put
        try:
            self.log_queue.put_nowait(log_entry)
        except Exception:
            # Queue full, drop log (prevents blocking Modbus)
            pass
