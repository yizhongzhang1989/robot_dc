#!/usr/bin/env python3

import os
import json
import xml.etree.ElementTree as ET
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
import rclpy
from rclpy.node import Node
import base64

class URDFWebServer(Node):
    def __init__(self):
        super().__init__('urdf_web_server')
        
        # Get parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('urdf_file', '/home/a/Documents/robot_dc/colcon_ws/src/duco_gcr5_910_urdf/urdf/duco_gcr5_910_urdf.urdf')
        
        self.port = self.get_parameter('port').value
        self.urdf_file = self.get_parameter('urdf_file').value
        
        # Try to load URDF file directly
        try:
            self.get_logger().info(f"Loading URDF from: {self.urdf_file}")
            with open(self.urdf_file, 'r') as f:
                self.urdf_content = f.read()
            self.get_logger().info("Successfully loaded URDF file")
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF file: {e}")
            self.urdf_content = ""
        
        # Parse joints from URDF
        self.joints = self.parse_joints()
        
        # Joint state tracking
        self.joint_states = {joint: 0.0 for joint in self.joints.keys()}
        
        # Load mesh files
        self.meshes = {}
        self.load_meshes()
        
        # Initialize ROS components
        self.init_ros_components()
        
        # Start web server
        self.start_web_server()
        
    def init_ros_components(self):
        """Initialize ROS-related components"""
        self.get_logger().info("ROS components initialized")
        
    def load_meshes(self):
        """Load mesh files from the URDF package"""
        try:
            # Get the directory containing the URDF file
            urdf_dir = os.path.dirname(self.urdf_file)
            package_dir = os.path.dirname(urdf_dir)  # Go up one level from urdf/ to package root
            mesh_dir = os.path.join(package_dir, 'meshes')
            
            if not os.path.exists(mesh_dir):
                self.get_logger().warn(f"Mesh directory not found: {mesh_dir}")
                return
                
            # Load all STL files
            for root, dirs, files in os.walk(mesh_dir):
                for file in files:
                    if file.endswith('.stl') or file.endswith('.STL'):
                        mesh_path = os.path.join(root, file)
                        try:
                            with open(mesh_path, 'rb') as f:
                                mesh_data = f.read()
                                # Encode as base64 for web transmission
                                self.meshes[file] = base64.b64encode(mesh_data).decode('utf-8')
                            self.get_logger().info(f"Loaded mesh: {file}")
                        except Exception as e:
                            self.get_logger().error(f"Failed to load mesh {file}: {e}")
                            
        except Exception as e:
            self.get_logger().error(f"Error loading meshes: {e}")
        
    def parse_joints(self):
        """Parse joint information from URDF"""
        joints = {}
        try:
            root = ET.fromstring(self.urdf_content)
            
            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                
                if joint_type in ['revolute', 'prismatic', 'continuous']:
                    limit_elem = joint.find('limit')
                    if limit_elem is not None:
                        lower = float(limit_elem.get('lower', '-3.14'))
                        upper = float(limit_elem.get('upper', '3.14'))
                    else:
                        lower, upper = -3.14, 3.14
                    
                    joints[joint_name] = {
                        'type': joint_type,
                        'lower': lower,
                        'upper': upper,
                        'value': 0.0
                    }
            
            self.get_logger().info(f'Found {len(joints)} controllable joints: {list(joints.keys())}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse joints: {str(e)}')
            
        return joints
    
    def update_joint_value(self, joint_name, value):
        """Update joint value"""
        if joint_name in self.joint_states:
            # Clamp value to joint limits
            joint_info = self.joints[joint_name]
            clamped_value = max(joint_info['lower'], min(joint_info['upper'], value))
            self.joint_states[joint_name] = clamped_value
            self.get_logger().info(f'Updated joint {joint_name} to {clamped_value}')
    
    def start_web_server(self):
        """Start HTTP server"""
        server = HTTPServer(('', self.port), self.create_request_handler())
        self.get_logger().info(f"Server started on http://localhost:{self.port}")
        server.serve_forever()
    
    def create_request_handler(self):
        """Create request handler with access to node instance"""
        node = self
        
        class RequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                # Suppress default logging
                pass
            
            def do_GET(self):
                parsed_url = urlparse(self.path)
                path = parsed_url.path
                
                if path == '/':
                    self.serve_html()
                elif path == '/api/urdf':
                    self.serve_urdf()
                elif path == '/api/joints':
                    self.serve_joints()
                elif path == '/api/meshes':
                    self.serve_meshes()
                elif path == '/api/joint_states':
                    self.serve_joint_states()
                elif path.startswith('/third_party/'):
                    self.serve_static_file(path)
                else:
                    self.send_error(404)
            
            def do_HEAD(self):
                # Handle HEAD requests the same as GET but without body
                self.do_GET()
            
            def do_POST(self):
                parsed_url = urlparse(self.path)
                path = parsed_url.path
                
                if path == '/api/update_joint':
                    self.update_joint()
                else:
                    self.send_error(404)
            
            def serve_html(self):
                html_content = self.get_html_content()
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(html_content.encode())
            
            def serve_urdf(self):
                self.send_response(200)
                self.send_header('Content-type', 'application/xml')
                self.end_headers()
                self.wfile.write(node.urdf_content.encode())
            
            def serve_joints(self):
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(node.joints).encode())
            
            def serve_meshes(self):
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(node.meshes).encode())
            
            def serve_joint_states(self):
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(node.joint_states).encode())
            
            def serve_static_file(self, path):
                """Serve static files from third_party directory"""
                try:
                    # Get the package directory (go up from urdf_web_viewer/ to src/urdf_web_viewer/)
                    package_dir = os.path.dirname(os.path.dirname(__file__))
                    file_path = os.path.join(package_dir, path.lstrip('/'))
                    
                    if os.path.exists(file_path) and os.path.isfile(file_path):
                        # Determine content type
                        content_type = 'text/plain'
                        if path.endswith('.js'):
                            content_type = 'application/javascript'
                        elif path.endswith('.css'):
                            content_type = 'text/css'
                        elif path.endswith('.html'):
                            content_type = 'text/html'
                        elif path.endswith('.json'):
                            content_type = 'application/json'
                        
                        # Read and serve the file
                        with open(file_path, 'r', encoding='utf-8') as f:
                            content = f.read()
                        
                        self.send_response(200)
                        self.send_header('Content-type', content_type)
                        self.end_headers()
                        self.wfile.write(content.encode())
                    else:
                        self.send_error(404)
                        
                except Exception as e:
                    node.get_logger().error(f"Error serving static file {path}: {e}")
                    self.send_error(500)
            
            def update_joint(self):
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                
                try:
                    data = json.loads(post_data.decode())
                    joint_name = data['joint_name']
                    value = float(data['value'])
                    
                    node.update_joint_value(joint_name, value)
                    
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({'status': 'success'}).encode())
                    
                except Exception as e:
                    self.send_error(400, str(e))
            
            def get_html_content(self):
                return '''
<!DOCTYPE html>
<html>
<head>
    <title>URDF Web Viewer</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 0; background-color: #f0f0f0; overflow: hidden; }
        .container { display: flex; height: 100vh; }
        .viewer-section { flex: 3; background: #000; position: relative; }
        .controls-section { flex: 1; background: #fff; padding: 20px; overflow-y: auto; border-left: 2px solid #ddd; }
        h1 { color: #333; text-align: center; margin-bottom: 20px; font-size: 1.5em; }
        .joint-control { margin-bottom: 20px; }
        .joint-control label { display: block; margin-bottom: 5px; font-weight: bold; font-size: 0.9em; }
        .joint-control input[type="range"] { width: 100%; }
        .joint-control .value { color: #666; font-size: 0.8em; margin-top: 5px; }
        .status { margin-top: 20px; padding: 10px; background: #e7f3ff; border-radius: 5px; font-size: 0.9em; }
        .loading { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: white; text-align: center; }
        .controls { position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.7); color: white; padding: 10px; border-radius: 5px; font-size: 0.8em; }
        #viewer { width: 100%; height: 100%; }
        .info { margin-bottom: 20px; padding: 10px; background: #f8f9fa; border-radius: 5px; font-size: 0.8em; }
    </style>
</head>
<body>
    <div class="container">
        <div class="viewer-section">
            <div id="viewer"></div>
            <div class="controls">
                <div>🖱️ Mouse Controls:</div>
                <div>• Left click + drag: Rotate</div>
                <div>• Right click + drag: Pan</div>
                <div>• Scroll: Zoom</div>
            </div>
            <div class="loading" id="loading">Loading 3D model...</div>
        </div>
        <div class="controls-section">
            <h1>🤖 URDF Viewer</h1>
            <div class="info">
                <strong>Robot Model:</strong> <span id="robot-name">Loading...</span><br>
                <strong>Total Joints:</strong> <span id="joint-count">0</span><br>
                <strong>Status:</strong> <span id="status">Initializing...</span>
            </div>
            <h3>Joint Controls</h3>
            <div id="joint-controls">
                <div class="loading">Loading joint controls...</div>
            </div>
        </div>
    </div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
    <script src="/third_party/urdf-loaders/javascript/umd/URDFLoader.js"></script>

    <script>
        let scene, camera, renderer, controls;
        let joints = {};
        let meshes = {};
        let robotGroup;
        let jointStates = {};
        let robotName = '';
        
        // Initialize Three.js scene
        function initScene() {
            const container = document.getElementById('viewer');
            
            // Scene
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x222222);
            
            // Camera
            camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
            camera.position.set(2, 2, 2);
            
            // Renderer
            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(container.clientWidth, container.clientHeight);
            renderer.shadowMap.enabled = true;
            renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            container.appendChild(renderer.domElement);
            
            // Controls
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.05;
            
            // Lights
            const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
            scene.add(ambientLight);
            
            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
            directionalLight.position.set(5, 5, 5);
            directionalLight.castShadow = true;
            scene.add(directionalLight);
            
            // Grid
            const gridHelper = new THREE.GridHelper(5, 50, 0x444444, 0x444444);
            scene.add(gridHelper);
            
            // Robot group
            robotGroup = new THREE.Group();
            scene.add(robotGroup);
            
            // Handle window resize
            window.addEventListener('resize', onWindowResize);
            
            // Start animation loop
            animate();
        }
        
        function onWindowResize() {
            const container = document.getElementById('viewer');
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        }
        
        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }
        
        // Load initial data
        async function initialize() {
            try {
                document.getElementById('status').textContent = 'Loading URDF...';
                
                // Get URDF content
                const urdfResponse = await fetch('/api/urdf');
                const urdfText = await urdfResponse.text();
                
                // Parse robot name
                const parser = new DOMParser();
                const xmlDoc = parser.parseFromString(urdfText, 'text/xml');
                const robotElement = xmlDoc.getElementsByTagName('robot')[0];
                robotName = robotElement ? robotElement.getAttribute('name') : 'Unknown Robot';
                document.getElementById('robot-name').textContent = robotName;
                
                // Get joints
                const jointsResponse = await fetch('/api/joints');
                joints = await jointsResponse.json();
                document.getElementById('joint-count').textContent = Object.keys(joints).length;
                
                // Get meshes
                const meshesResponse = await fetch('/api/meshes');
                meshes = await meshesResponse.json();
                
                // Get initial joint states
                const statesResponse = await fetch('/api/joint_states');
                jointStates = await statesResponse.json();
                
                // Create joint controls
                createJointControls();
                
                // Load 3D model
                await load3DModel(urdfText);
                
                document.getElementById('status').textContent = 'Ready';
                document.getElementById('loading').style.display = 'none';
                
            } catch (error) {
                console.error('Initialization error:', error);
                document.getElementById('status').textContent = 'Error: ' + error.message;
                document.getElementById('loading').textContent = 'Error loading model: ' + error.message;
            }
        }
        
        function createJointControls() {
            const container = document.getElementById('joint-controls');
            container.innerHTML = '';
            
            if (Object.keys(joints).length === 0) {
                container.innerHTML = '<div style="color: #666; font-style: italic;">No controllable joints found</div>';
                return;
            }
            
            for (const [jointName, jointInfo] of Object.entries(joints)) {
                const div = document.createElement('div');
                div.className = 'joint-control';
                
                const label = document.createElement('label');
                label.textContent = jointName;
                
                const slider = document.createElement('input');
                slider.type = 'range';
                slider.min = jointInfo.lower;
                slider.max = jointInfo.upper;
                slider.step = 0.01;
                slider.value = jointStates[jointName] || 0;
                
                const valueSpan = document.createElement('div');
                valueSpan.className = 'value';
                updateValueDisplay(valueSpan, jointInfo, parseFloat(slider.value));
                
                slider.addEventListener('input', (e) => {
                    const value = parseFloat(e.target.value);
                    updateValueDisplay(valueSpan, jointInfo, value);
                    updateJoint(jointName, value);
                });
                
                div.appendChild(label);
                div.appendChild(slider);
                div.appendChild(valueSpan);
                container.appendChild(div);
            }
        }
        
        function updateValueDisplay(element, jointInfo, value) {
            element.textContent = `${value.toFixed(2)} rad (${jointInfo.lower.toFixed(2)} to ${jointInfo.upper.toFixed(2)})`;
        }
        
        async function updateJoint(jointName, value) {
            try {
                const response = await fetch('/api/update_joint', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        joint_name: jointName,
                        value: value
                    })
                });
                
                if (response.ok) {
                    jointStates[jointName] = value;
                    updateRobotPose();
                    document.getElementById('status').textContent = `Updated ${jointName} to ${value.toFixed(2)}`;
                }
                
            } catch (error) {
                console.error('Update error:', error);
                document.getElementById('status').textContent = 'Error updating joint';
            }
        }
        
        async function load3DModel(urdfText) {
            try {
                // Create URDF loader
                const loader = new URDFLoader();
                
                // Configure loader options
                loader.parseCollision = false;
                loader.loadMeshCb = (path, manager, done) => {
                    // Check if this is the correct signature
                    if (typeof manager === 'function') {
                        // If manager is actually the done callback, adjust
                        done = manager;
                        manager = null;
                    }
                    
                    // Extract mesh filename from path
                    const meshName = path.split('/').pop();
                    
                    console.log('Loading mesh:', meshName, 'from path:', path);
                    console.log('Manager:', manager);
                    console.log('Done callback type:', typeof done);
                    
                    if (meshes[meshName]) {
                        try {
                            // Decode base64 mesh data
                            const binaryData = atob(meshes[meshName]);
                            const bytes = new Uint8Array(binaryData.length);
                            for (let k = 0; k < binaryData.length; k++) {
                                bytes[k] = binaryData.charCodeAt(k);
                            }
                            
                            // Parse STL
                            const stlLoader = new THREE.STLLoader();
                            const stlGeometry = stlLoader.parse(bytes.buffer);
                            stlGeometry.computeVertexNormals();
                            
                            // Create mesh with material
                            const material = new THREE.MeshPhongMaterial({ 
                                color: 0x888888,
                                shininess: 100,
                                transparent: true,
                                opacity: 0.9
                            });
                            
                            const mesh = new THREE.Mesh(stlGeometry, material);
                            mesh.castShadow = true;
                            mesh.receiveShadow = true;
                            
                            if (typeof done === 'function') {
                                done(mesh);
                            } else {
                                console.error('done is not a function:', done);
                            }
                        } catch (error) {
                            console.warn(`Failed to load mesh ${meshName}:`, error);
                            if (typeof done === 'function') {
                                done(null);
                            }
                        }
                    } else {
                        console.warn(`Mesh not found: ${meshName}`);
                        if (typeof done === 'function') {
                            done(null);
                        }
                    }
                };
                
                // Load the URDF
                const robot = loader.parse(urdfText);
                
                // Store reference to robot for joint control
                window.robotModel = robot;
                
                // Add robot to scene
                robotGroup.add(robot);
                
                // Center the robot
                const box = new THREE.Box3().setFromObject(robotGroup);
                const center = box.getCenter(new THREE.Vector3());
                robotGroup.position.set(-center.x, -center.y, -center.z);
                
                console.log('Robot loaded successfully:', robot);
                console.log('Available joints:', Object.keys(robot.joints));
                
            } catch (error) {
                console.error('Error loading URDF model:', error);
                throw error;
            }
        }
        
        function updateRobotPose() {
            if (!window.robotModel) return;
            
            // Apply joint values using the URDF loader's built-in joint control
            for (const [jointName, jointValue] of Object.entries(jointStates)) {
                const joint = window.robotModel.joints[jointName];
                if (joint) {
                    joint.setJointValue(jointValue);
                }
            }
        }
        
        // Initialize when page loads
        window.addEventListener('load', () => {
            initScene();
            initialize();
        });
    </script>
</body>
</html>
                '''
        
        return RequestHandler

def main():
    rclpy.init()
    
    try:
        node = URDFWebServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
