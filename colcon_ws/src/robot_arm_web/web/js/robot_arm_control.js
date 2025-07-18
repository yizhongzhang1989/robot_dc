// Robot Arm Web Control JavaScript

let commandCounter = 0;
let stateUpdateInterval;
let renderLoopActive = false;
let lastStateUpdate = null;
let updateCount = 0;
let updateRate = 0;
let lastUIUpdate = 0;
let robotStateData = null;

// User configurable settings
const settings = {
    dataRefreshRate: 100,     // How often to fetch data from API (ms)
    uiRefreshRate: 500,      // How often to update the UI elements (ms)
    render3DRate: 30,        // Target FPS for 3D rendering
    animationEnabled: true,  // Whether to enable continuous rendering
    debug: false            // Enable debug mode
};

// 3D Visualization variables
let scene, camera, renderer, controls;
let baseFrame, tcpFrame;
let worldGroup; // Global rotation group - like glRotate in OpenGL
let isViewer3DInitialized = false;
let needsRender = true;      // Flag to indicate if rendering is needed
let directionalLight;        // Global directional light variable

// URDF variables
let urdfJoints = {};
let urdfMeshes = {};
let urdfJointStates = {};
let robotModel = null;

// Initialize the page
window.addEventListener('load', function() {
    console.log('Page load event fired');
    
    // Check if Three.js is loaded
    if (typeof THREE === 'undefined') {
        console.error('Three.js is not loaded');
        return;
    }
    
    console.log('Three.js loaded successfully');
    
    fetchRobotArmInfo();
    updateConnectionStatus(true);
    startStateMonitoring();
    
    // Initialize 3D viewer with a small delay to ensure DOM is ready
    setTimeout(() => {
        console.log('Initializing 3D viewer...');
        init3DViewer();
    }, 100);
    
    // Initialize URDF viewer
    setTimeout(() => {
        console.log('Starting URDF initialization...');
        
        // Test URDFLoader availability first
        if (typeof URDFLoader === 'undefined') {
            console.error('ERROR: URDFLoader is not defined!');
            return;
        }
        console.log('URDFLoader is available');
        
        // Create a simple test to see if the parser works
        try {
            const testLoader = new URDFLoader();
            console.log('URDFLoader instance created successfully');
            
            // Test parsing with simple URDF
            const simpleURDF = `<?xml version="1.0"?>
<robot name="test">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>`;
            
            const testRobot = testLoader.parse(simpleURDF);
            console.log('Simple URDF parsed successfully');
            console.log('Test robot type:', testRobot ? testRobot.constructor.name : 'null');
            console.log('Test robot children:', testRobot && testRobot.children ? testRobot.children.length : 'undefined');
            
            // Now try the actual URDF initialization
            console.log('About to call initURDFViewer()...');
            try {
                console.log('Calling initURDFViewer() now...');
                initURDFViewer();
                console.log('initURDFViewer() call returned');
            } catch (error) {
                console.error('Error in initURDFViewer() call:', error);
            }
            
        } catch (error) {
            console.error('ERROR in URDF test:', error.message);
            console.error('Error stack:', error.stack);
        }
    }, 300);
    
    logCommand('System', 'Web interface initialized');
});

// WebSocket for real-time updates
let stateSocket = null;

// Start robot state monitoring
function startStateMonitoring() {
    // Try WebSocket first for real-time updates
    if ("WebSocket" in window) {
        connectWebSocket();
    } else {
        // Fallback to polling if WebSockets are not supported
        logCommand('System', 'WebSockets not supported, using polling instead');
        stateUpdateInterval = setInterval(fetchRobotState, settings.dataRefreshRate);
    }
}

// Connect to WebSocket for real-time robot state updates
function connectWebSocket() {
    // Close existing socket if any
    if (stateSocket) {
        stateSocket.close();
    }
    
    // Create WebSocket connection
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/robot_state`;
    
    stateSocket = new WebSocket(wsUrl);
    
    stateSocket.onopen = function(e) {
        logCommand('System', 'WebSocket connection established');
        updateConnectionStatus(true);
        
        // Cancel polling interval if it exists
        if (stateUpdateInterval) {
            clearInterval(stateUpdateInterval);
            stateUpdateInterval = null;
        }
    };
    
    stateSocket.onmessage = function(event) {
        // Handle ping messages
        if (event.data === "ping") return;
        
        try {
            const stateData = JSON.parse(event.data);
            
            // Store the data for UI updates
            robotStateData = stateData;
            
            // Always update TCP frame in 3D viewer immediately for smooth motion
            updateTCPFramePosition();
            
            // Request a 3D render
            needsRender = true;
            
            // Calculate update rate - fixed to use 1000 for correct Hz calculation
            const now = Date.now();
            if (lastStateUpdate) {
                const timeBetween = now - lastStateUpdate;
                updateCount++;
                
                // Update debug info if enabled
                if (settings.debug) {
                    document.getElementById('debugLastUpdate').textContent = new Date().toLocaleTimeString() + '.' + String(now % 1000).padStart(3, '0');
                    document.getElementById('debugTimeBetween').textContent = timeBetween;
                    document.getElementById('debugMsgCount').textContent = updateCount;
                }
                
                // Calculate rolling average of last 10 updates for more stable rate display
                if (updateCount % 10 === 0) {
                    // Store last 10 intervals to calculate average rate
                    if (!window.timeIntervals) window.timeIntervals = [];
                    window.timeIntervals.push(timeBetween);
                    // Keep only last 10 intervals
                    if (window.timeIntervals.length > 10) window.timeIntervals.shift();
                    
                    // Calculate average interval
                    const avgInterval = window.timeIntervals.reduce((sum, val) => sum + val, 0) / window.timeIntervals.length;
                    
                    // Calculate Hz correctly: 1000ms / avg_time_between_messages_ms
                    updateRate = Math.round(1000 / avgInterval * 10) / 10;
                    
                    if (settings.debug) {
                        document.getElementById('debugAvgInterval').textContent = avgInterval.toFixed(2) + "ms";
                    }
                }
            }
            lastStateUpdate = now;
            
            // Update UI if enough time has passed since last update
            if (now - lastUIUpdate >= settings.uiRefreshRate) {
                updateRobotStateDisplay(robotStateData);
                updateStateStatus(true);
                document.getElementById('updateRate').textContent = updateRate;
                lastUIUpdate = now;
            }
        } catch (error) {
            console.error('Error processing WebSocket message:', error);
        }
    };
    
    stateSocket.onclose = function(event) {
        if (event.wasClean) {
            logCommand('System', `WebSocket connection closed cleanly, code=${event.code} reason=${event.reason}`);
        } else {
            logCommand('System', 'WebSocket connection died', 'error');
            // Try to reconnect in 5 seconds
            setTimeout(connectWebSocket, 5000);
        }
        
        // Fall back to polling if WebSocket fails
        if (!stateUpdateInterval) {
            stateUpdateInterval = setInterval(fetchRobotState, settings.dataRefreshRate);
        }
    };
    
    stateSocket.onerror = function(error) {
        console.error('WebSocket error:', error);
        logCommand('System', `WebSocket error: ${error.type || 'Connection failed'}`, 'error');
        updateConnectionStatus(false);
        
        // Fall back to polling if WebSocket fails
        if (!stateUpdateInterval) {
            logCommand('System', 'Falling back to polling method');
            stateUpdateInterval = setInterval(fetchRobotState, settings.dataRefreshRate);
        }
    };
}

// Fetch robot state (fallback polling method)
async function fetchRobotState() {
    // Skip if we have an active WebSocket connection
    if (stateSocket && stateSocket.readyState === WebSocket.OPEN) {
        return;
    }
    
    // Skip if WebSocket just received data (avoid rate calculation interference)
    const now = Date.now();
    if (lastStateUpdate && now - lastStateUpdate < settings.dataRefreshRate * 0.8) {
        return;
    }
    
    try {
        const response = await fetch('/api/robot_arm/state');
        
        if (response.ok) {
            // Store the data for rendering and UI updates
            robotStateData = await response.json();
            
            // Always update TCP frame in 3D viewer immediately for smooth motion
            updateTCPFramePosition();
            
            // Request a 3D render
            needsRender = true;
            
            // Calculate update rate
            const now = Date.now();
            if (lastStateUpdate) {
                const timeBetween = now - lastStateUpdate;
                updateCount++;
                
                // Calculate rolling average of last 10 updates for more stable rate display
                if (updateCount % 10 === 0) {
                    // Store last 10 intervals to calculate average rate
                    if (!window.timeIntervals) window.timeIntervals = [];
                    window.timeIntervals.push(timeBetween);
                    // Keep only last 10 intervals
                    if (window.timeIntervals.length > 10) window.timeIntervals.shift();
                    
                    // Calculate average interval
                    const avgInterval = window.timeIntervals.reduce((sum, val) => sum + val, 0) / window.timeIntervals.length;
                    
                    // Calculate Hz correctly: 1000ms / avg_time_between_messages_ms
                    updateRate = Math.round(1000 / avgInterval * 10) / 10;
                }
            }
            lastStateUpdate = now;
            
            // Update UI if enough time has passed since last update
            if (now - lastUIUpdate >= settings.uiRefreshRate) {
                updateRobotStateDisplay(robotStateData);
                updateStateStatus(true);
                document.getElementById('updateRate').textContent = updateRate;
                lastUIUpdate = now;
            }
        } else {
            updateStateStatus(false);
            document.getElementById('stateStatus').textContent = 'No Data';
            document.getElementById('stateStatus').className = 'error-indicator';
            
            // Try WebSocket connection if polling fails
            if (!stateSocket || stateSocket.readyState !== WebSocket.OPEN) {
                connectWebSocket();
            }
        }
    } catch (error) {
        updateStateStatus(false);
        console.error('Error fetching robot state:', error);
    }
}

// Update TCP frame position in 3D scene
function updateTCPFramePosition() {
    if (!robotStateData || !robotStateData.TCPActualPosition || !tcpFrame) return;
    
    const tcp = robotStateData.TCPActualPosition;
    if (tcp.length >= 6) {
        // Update position
        tcpFrame.position.set(tcp[0], tcp[1], tcp[2]);
        
        // Rotation: Apply Rz*Ry*Rx transformation (matching update3DVisualization)
        // TCP data: [X, Y, Z, Rx, Ry, Rz] where rotations are in radians
        tcpFrame.rotation.order = 'ZYX';  // This applies rotations in Z, Y, X order = Rz*Ry*Rx
        tcpFrame.rotation.set(tcp[3], tcp[4], tcp[5]); // Set Rx, Ry, Rz
        
        // Log when TCP frame is updated with position and rotation
        console.log("TCP Frame Updated:",
            "Position:", 
            tcpFrame.position.x.toFixed(3),
            tcpFrame.position.y.toFixed(3),
            tcpFrame.position.z.toFixed(3),
            "| Rotation:",
            tcpFrame.rotation.x.toFixed(3),
            tcpFrame.rotation.y.toFixed(3),
            tcpFrame.rotation.z.toFixed(3),
            "Order:", tcpFrame.rotation.order
        );
    }
}

// Update robot state display
function updateRobotStateDisplay(state) {
    // Helper function to safely format values
    const formatValue = (value, decimals = 4) => {
        if (value === null || value === undefined || isNaN(value)) return '-';
        return parseFloat(value).toFixed(decimals);
    };

    // Helper function to create joint display (only 6 joints, not 7)
    const createJointDisplay = (values, unit = '', decimals = 4) => {
        if (!values || !Array.isArray(values)) return 'No data';
        // Only show first 6 joints (7th is reserved)
        const jointsToShow = values.slice(0, 6);
        return jointsToShow.map((value, index) => {
            const formattedValue = formatValue(value, decimals);
            return `<div class="compact-item"><span class="data-label">J${index + 1}:</span> <span class="data-value">${formattedValue}${unit}</span></div>`;
        }).join('');
    };

    // Helper function to create compact joint grid (like TCP data)
    const createCompactJointGrid = (values, unit = '', decimals = 4) => {
        if (!values || !Array.isArray(values)) return '<div class="compact-item"><span class="data-label">No data</span></div>';
        // Only show first 6 joints (7th is reserved)
        const jointsToShow = values.slice(0, 6);
        return jointsToShow.map((value, index) => {
            const formattedValue = formatValue(value, decimals);
            return `<div class="compact-item"><span class="data-label">J${index + 1}:</span> <span class="data-value">${formattedValue}${unit}</span></div>`;
        }).join('');
    };

    // Update TCP Actual Position
    if (state.TCPActualPosition && state.TCPActualPosition.length >= 6) {
        const tcp = state.TCPActualPosition;
        document.getElementById('tcpActualPosX').textContent = formatValue(tcp[0]);
        document.getElementById('tcpActualPosY').textContent = formatValue(tcp[1]);
        document.getElementById('tcpActualPosZ').textContent = formatValue(tcp[2]);
        document.getElementById('tcpActualPosRx').textContent = formatValue(tcp[3]);
        document.getElementById('tcpActualPosRy').textContent = formatValue(tcp[4]);
        document.getElementById('tcpActualPosRz').textContent = formatValue(tcp[5]);
    }

    // Update TCP Expected Position
    if (state.TCPExpectPosition && state.TCPExpectPosition.length >= 6) {
        const tcp = state.TCPExpectPosition;
        document.getElementById('tcpExpectedPosX').textContent = formatValue(tcp[0]);
        document.getElementById('tcpExpectedPosY').textContent = formatValue(tcp[1]);
        document.getElementById('tcpExpectedPosZ').textContent = formatValue(tcp[2]);
        document.getElementById('tcpExpectedPosRx').textContent = formatValue(tcp[3]);
        document.getElementById('tcpExpectedPosRy').textContent = formatValue(tcp[4]);
        document.getElementById('tcpExpectedPosRz').textContent = formatValue(tcp[5]);
    }

    // Update TCP Actual Velocity
    if (state.TCPActualVelocity && state.TCPActualVelocity.length >= 6) {
        const tcp = state.TCPActualVelocity;
        document.getElementById('tcpActualVelX').textContent = formatValue(tcp[0]);
        document.getElementById('tcpActualVelY').textContent = formatValue(tcp[1]);
        document.getElementById('tcpActualVelZ').textContent = formatValue(tcp[2]);
        document.getElementById('tcpActualVelRx').textContent = formatValue(tcp[3]);
        document.getElementById('tcpActualVelRy').textContent = formatValue(tcp[4]);
        document.getElementById('tcpActualVelRz').textContent = formatValue(tcp[5]);
    }

    // Update TCP Actual Acceleration
    if (state.TCPActualAccelera && state.TCPActualAccelera.length >= 6) {
        const tcp = state.TCPActualAccelera;
        document.getElementById('tcpActualAccX').textContent = formatValue(tcp[0]);
        document.getElementById('tcpActualAccY').textContent = formatValue(tcp[1]);
        document.getElementById('tcpActualAccZ').textContent = formatValue(tcp[2]);
        document.getElementById('tcpActualAccRx').textContent = formatValue(tcp[3]);
        document.getElementById('tcpActualAccRy').textContent = formatValue(tcp[4]);
        document.getElementById('tcpActualAccRz').textContent = formatValue(tcp[5]);
    }

    // Update TCP Actual Torque
    if (state.TCPActualTorque && state.TCPActualTorque.length >= 6) {
        const tcp = state.TCPActualTorque;
        document.getElementById('tcpActualTorqueX').textContent = formatValue(tcp[0]);
        document.getElementById('tcpActualTorqueY').textContent = formatValue(tcp[1]);
        document.getElementById('tcpActualTorqueZ').textContent = formatValue(tcp[2]);
        document.getElementById('tcpActualTorqueRx').textContent = formatValue(tcp[3]);
        document.getElementById('tcpActualTorqueRy').textContent = formatValue(tcp[4]);
        document.getElementById('tcpActualTorqueRz').textContent = formatValue(tcp[5]);
    }

    // Update Base Actual Torque
    if (state.baseActualTorque && state.baseActualTorque.length >= 6) {
        const base = state.baseActualTorque;
        document.getElementById('baseActualTorqueX').textContent = formatValue(base[0]);
        document.getElementById('baseActualTorqueY').textContent = formatValue(base[1]);
        document.getElementById('baseActualTorqueZ').textContent = formatValue(base[2]);
        document.getElementById('baseActualTorqueRx').textContent = formatValue(base[3]);
        document.getElementById('baseActualTorqueRy').textContent = formatValue(base[4]);
        document.getElementById('baseActualTorqueRz').textContent = formatValue(base[5]);
    }

    // Update Joint Actual Position
    if (state.jointActualPosition) {
        document.getElementById('jointActualPositions').innerHTML = createCompactJointGrid(state.jointActualPosition, ' rad');
        // Update URDF joint states from actual joint positions
        updateURDFFromActualJoints(state.jointActualPosition);
    }

    // Update Joint Actual Velocity
    if (state.jointActualVelocity) {
        document.getElementById('jointActualVelocities').innerHTML = createCompactJointGrid(state.jointActualVelocity, ' rad/s');
    }

    // Update Joint Actual Acceleration
    if (state.jointActualAccelera) {
        document.getElementById('jointActualAccelerations').innerHTML = createCompactJointGrid(state.jointActualAccelera, ' rad/s²');
    }

    // Update Joint Actual Torque
    if (state.jointActualTorque) {
        document.getElementById('jointActualTorques').innerHTML = createCompactJointGrid(state.jointActualTorque, ' Nm');
    }

    // Update Joint Expected Position
    if (state.jointExpectPosition) {
        document.getElementById('jointExpectedPositions').innerHTML = createCompactJointGrid(state.jointExpectPosition, ' rad');
    }

    // Update Joint Expected Velocity
    if (state.jointExpectVelocity) {
        document.getElementById('jointExpectedVelocities').innerHTML = createCompactJointGrid(state.jointExpectVelocity, ' rad/s');
    }

    // Update Joint Temperatures
    if (state.jointActualTemperature) {
        // Only show first 6 joints for temperature with color coding
        const jointsToShow = state.jointActualTemperature.slice(0, 6);
        const tempHtml = jointsToShow.map((temp, index) => {
            const tempClass = temp > 50 ? 'error-indicator' : (temp > 40 ? 'warning-indicator' : 'normal-indicator');
            return `<div class="compact-item"><span class="data-label">J${index + 1}:</span> <span class="${tempClass}">${formatValue(temp, 1)}°C</span></div>`;
        }).join('');
        document.getElementById('jointTemperatures').innerHTML = tempHtml;
    }

    // Update Joint Currents
    if (state.jointActualCurrent) {
        // Only show first 6 joints for current with color coding
        const jointsToShow = state.jointActualCurrent.slice(0, 6);
        const currentHtml = jointsToShow.map((current, index) => {
            const currentClass = current > 800 ? 'error-indicator' : (current > 600 ? 'warning-indicator' : 'normal-indicator');
            return `<div class="compact-item"><span class="data-label">J${index + 1}:</span> <span class="${currentClass}">${formatValue(current, 0)}‰</span></div>`;
        }).join('');
        document.getElementById('jointCurrents').innerHTML = currentHtml;
    }

    // Update Driver Status
    if (state.driverErrorID && state.driverState) {
        const hasErrors = state.driverErrorID.some(error => error !== 0);
        const errorElement = document.getElementById('driverErrors');
        if (hasErrors) {
            errorElement.textContent = 'ERRORS';
            errorElement.className = 'error-indicator';
        } else {
            errorElement.textContent = 'OK';
            errorElement.className = 'normal-indicator';
        }
        
        // Show error IDs
        const errorIds = state.driverErrorID.filter(id => id !== 0);
        document.getElementById('driverErrorIds').textContent = errorIds.length > 0 ? errorIds.join(', ') : 'None';
        
        // Show driver states
        document.getElementById('driverStates').textContent = state.driverState.join(', ');
    }

    // Update last update time
    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
    document.getElementById('dataStatus').textContent = 'Active';
    document.getElementById('dataStatus').className = 'normal-indicator';

    // Update 3D visualization - removed duplicate TCP update to prevent flickering
}

// Initialize 3D Viewer
function init3DViewer() {
    try {
        const container = document.getElementById('viewer3d');
        const canvas = document.getElementById('robot3dCanvas');
        
        if (!container || !canvas) {
            console.error('3D viewer container or canvas not found');
            return;
        }
        
        // Scene setup
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x2a2a2a);
        
        // Camera setup
        const containerWidth = container.offsetWidth;
        const containerHeight = container.offsetHeight;
        camera = new THREE.PerspectiveCamera(75, containerWidth / containerHeight, 0.1, 1000);
        // Position camera for Z-up coordinate system (robot perspective)
        camera.position.set(1, 1, 1);
        camera.lookAt(0, 0, 0);
        
        // Renderer setup
        renderer = new THREE.WebGLRenderer({ 
            canvas: canvas, 
            antialias: true,
            alpha: true 
        });
        renderer.setSize(containerWidth, containerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        
        // Controls setup
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = false;  // Disable inertia/damping
        controls.dampingFactor = 0.05;
        controls.target.set(0, 0, 0);
        controls.enableZoom = true;
        controls.enableRotate = true;
        controls.enablePan = true;
        
        // Make controls slower and smoother
        controls.rotateSpeed = 0.3;
        controls.panSpeed = 0.3;
        controls.zoomSpeed = 0.5;
        
        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        scene.add(ambientLight);
        
        directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(2, 2, 2);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        scene.add(directionalLight);
        
        // Grid - create in x-y plane instead of x-z plane
        const gridHelper = new THREE.GridHelper(2, 20, 0x444444, 0x444444);
        // Rotate grid to x-y plane (default GridHelper is in x-z plane)
        gridHelper.rotation.x = Math.PI / 2;
        
        // Create world group for global rotation (like glRotate in OpenGL)
        worldGroup = new THREE.Group();
        
        // Add grid to world group
        worldGroup.add(gridHelper);
        
        // Create coordinate frames and add them to world group
        createCoordinateFrames();
        
        // Add world group to scene
        scene.add(worldGroup);
        
        // Apply global rotation to make Z-up (like glRotatef(-90, 1, 0, 0) in OpenGL)
        worldGroup.rotation.x = -Math.PI / 2;
        
        // Start animation loop with optimal frame timing
        renderLoopActive = true;
        requestAnimationFrame(animate);
        
        // Handle window resize
        window.addEventListener('resize', onWindowResize);
        
        // Add keyboard shortcut for resetting view
        document.addEventListener('keydown', handle3DKeyboard);
        
        isViewer3DInitialized = true;
        logCommand('System', '3D viewer initialized successfully');
        
        // Force initial render
        renderer.render(scene, camera);
        
    } catch (error) {
        console.error('Error initializing 3D viewer:', error);
        logCommand('System', `Failed to initialize 3D viewer: ${error.message}`, 'error');
    }
}

// Initialize URDF viewer functionality
async function initURDFViewer() {
    try {
        console.log('=== URDF VIEWER INITIALIZATION START ===');
        
        // Wait for 3D viewer to be initialized
        if (!isViewer3DInitialized || !scene || !worldGroup) {
            console.log('3D viewer not ready, waiting...', {
                isViewer3DInitialized,
                scene: !!scene,
                worldGroup: !!worldGroup
            });
            logCommand('System', 'Waiting for 3D viewer to initialize...');
            // Wait a bit longer and try again
            setTimeout(initURDFViewer, 500);
            return;
        }
        
        console.log('✅ 3D viewer is ready, proceeding with URDF initialization...');
        
        // Load URDF data
        logCommand('System', 'Loading URDF data...');
        
        // Get URDF content
        console.log('📥 Fetching URDF content...');
        const urdfResponse = await fetch('/api/urdf');
        const urdfText = await urdfResponse.text();
        console.log('✅ URDF content loaded, length:', urdfText.length);
        
        // Get joints
        console.log('📥 Fetching joints...');
        const jointsResponse = await fetch('/api/joints');
        urdfJoints = await jointsResponse.json();
        console.log('✅ Joints loaded:', Object.keys(urdfJoints));
        
        // Get meshes
        console.log('📥 Fetching meshes...');
        const meshesResponse = await fetch('/api/meshes');
        urdfMeshes = await meshesResponse.json();
        console.log('✅ Meshes loaded:', Object.keys(urdfMeshes));
        
        // Get initial joint states
        console.log('📥 Fetching joint states...');
        const statesResponse = await fetch('/api/joint_states');
        urdfJointStates = await statesResponse.json();
        console.log('✅ Joint states loaded:', urdfJointStates);
        
        // Load URDF model into 3D scene
        console.log('🎨 Loading URDF model into 3D scene...');
        await loadURDFModel(urdfText);
        console.log('✅ URDF model loaded into 3D scene');
        
        console.log('🎉 URDF viewer initialization completed successfully');
        logCommand('System', 'URDF viewer initialized successfully');
        
    } catch (error) {
        console.error('❌ Error initializing URDF viewer:', error);
        console.error('📋 Error stack:', error.stack);
        logCommand('System', `Failed to initialize URDF viewer: ${error.message}`, 'error');
    }
}

// Update URDF joint value
async function updateURDFJoint(jointName, value) {
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
            urdfJointStates[jointName] = value;
            updateRobotPose();
        }
        
    } catch (error) {
        console.error('Error updating joint:', error);
    }
}

// Load URDF model into 3D scene
async function loadURDFModel(urdfText) {
    try {
        console.log('🔍 Starting loadURDFModel...');
        
        if (!scene) {
            console.log('❌ 3D scene not initialized');
            throw new Error('3D scene not initialized');
        }
        console.log('✅ 3D scene is initialized');
        
        if (!worldGroup) {
            console.log('❌ World group not initialized');
            throw new Error('World group not initialized');
        }
        console.log('✅ World group is initialized');
        
        if (!urdfText || urdfText.trim() === '') {
            console.log('❌ URDF text is empty');
            throw new Error('URDF text is empty');
        }
        console.log('✅ URDF text is valid, length:', urdfText.length);
        
        // Check if URDFLoader is available
        if (typeof URDFLoader === 'undefined') {
            console.log('❌ URDFLoader is not loaded');
            throw new Error('URDFLoader is not loaded');
        }
        console.log('✅ URDFLoader is available');
        
        // Create URDF loader
        console.log('🔧 Creating URDF loader...');
        const loader = new URDFLoader();
        console.log('✅ URDF loader created');
        
        // Configure loader options
        loader.parseCollision = false;
        loader.loadMeshCb = (path, manager, done) => {
            console.log('🔗 Loading mesh callback called for:', path);
            
            // Check if this is the correct signature
            if (typeof manager === 'function') {
                // If manager is actually the done callback, adjust
                done = manager;
                manager = null;
            }
            
            // Extract mesh filename from path
            const meshName = path.split('/').pop();
            console.log('📦 Looking for mesh:', meshName);
            
            if (urdfMeshes[meshName]) {
                console.log('✅ Found mesh in cache:', meshName);
                try {
                    // Decode base64 mesh data
                    const binaryData = atob(urdfMeshes[meshName]);
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
                    
                    console.log('✅ Successfully created mesh for:', meshName);
                    if (typeof done === 'function') {
                        done(mesh);
                    }
                } catch (error) {
                    console.error(`❌ Failed to load mesh ${meshName}:`, error);
                    if (typeof done === 'function') {
                        done(null);
                    }
                }
            } else {
                console.warn(`⚠️ Mesh not found: ${meshName}`);
                if (typeof done === 'function') {
                    done(null);
                }
            }
        };
        
        // Load the URDF
        console.log('🚀 Parsing URDF...');
        
        try {
            robotModel = loader.parse(urdfText);
        } catch (error) {
            console.error('❌ URDFLoader.parse() failed:', error);
            throw error; // Re-throw to trigger the outer catch
        }
        
        console.log('✅ URDF parsed, robotModel:', robotModel);
        
        if (!robotModel) {
            console.log('❌ Failed to parse URDF model');
            throw new Error('Failed to parse URDF model');
        }
        
        // Check robotModel properties
        console.log('📊 Robot model properties:');
        console.log('  - Type:', typeof robotModel);
        console.log('  - Constructor:', robotModel.constructor.name);
        console.log('  - Keys:', Object.keys(robotModel));
        console.log('  - Has children:', 'children' in robotModel);
        console.log('  - Children value:', robotModel.children);
        
        // Try to access children safely
        try {
            if (robotModel.children) {
                console.log('✅ Children property exists and has length:', robotModel.children.length);
                for (let i = 0; i < Math.min(3, robotModel.children.length); i++) {
                    const child = robotModel.children[i];
                    console.log('  Child', i, ':', child ? child.constructor.name : 'null');
                }
            } else {
                console.log('❌ Children property is null or undefined');
            }
        } catch (error) {
            console.log('❌ Error accessing children:', error.message);
        }
        
        // Store reference to robot for joint control
        window.robotModel = robotModel;
        console.log('✅ Robot model stored in window.robotModel');
        
        // Add robot to world group
        console.log('🌍 Adding robot to world group...');
        
        try {
            worldGroup.add(robotModel);
            console.log('✅ Robot added to world group');
        } catch (error) {
            console.log('❌ Error adding robot to world group:', error.message);
            throw error;
        }
        
        // Center the robot
        console.log('🎯 Centering robot...');
        
        try {
            const box = new THREE.Box3().setFromObject(worldGroup);
            const center = box.getCenter(new THREE.Vector3());
            worldGroup.position.set(-center.x, -center.y, -center.z);
            console.log('✅ Robot centered at:', center);
        } catch (error) {
            console.log('❌ Error centering robot:', error.message);
            throw error;
        }
        
        console.log('🎉 URDF robot loaded successfully!');
        console.log('🔧 Available joints:', Object.keys(robotModel.joints || {}));
        
        // Force render
        needsRender = true;
        
    } catch (error) {
        console.error('❌ Error loading URDF model:', error);
        console.error('📋 Error stack:', error.stack);
        throw error;
    }
}

// Update URDF joint states from actual joint positions
function updateURDFFromActualJoints(jointActualPosition) {
    if (!jointActualPosition || !Array.isArray(jointActualPosition)) {
        return;
    }
    
    // Map joint actual positions to URDF joint names
    const jointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
    
    for (let i = 0; i < Math.min(jointActualPosition.length, jointNames.length); i++) {
        const jointName = jointNames[i];
        const actualValue = jointActualPosition[i];
        
        // Update the URDF joint state
        urdfJointStates[jointName] = actualValue;
        
        // Update the corresponding slider if it exists
        const slider = document.getElementById(`slider-${jointName}`);
        if (slider) {
            slider.value = actualValue;
            
            // Update the value display
            const valueSpan = document.getElementById(`value-${jointName}`);
            if (valueSpan) {
                valueSpan.textContent = `${actualValue.toFixed(2)} rad`;
            }
        }
    }
    
    // Update the robot pose in the 3D visualization
    updateRobotPose();
}

// Update robot pose based on joint states
function updateRobotPose() {
    if (!window.robotModel) return;
    
    if (!window.robotModel.joints) {
        console.warn('Robot model has no joints property');
        return;
    }
    
    // Apply joint values using the URDF loader's built-in joint control
    for (const [jointName, jointValue] of Object.entries(urdfJointStates)) {
        const joint = window.robotModel.joints[jointName];
        if (joint && typeof joint.setJointValue === 'function') {
            joint.setJointValue(jointValue);
        } else if (joint) {
            console.warn(`Joint ${jointName} found but setJointValue method not available`);
        }
    }
    
    // Force render
    needsRender = true;
}

// Create coordinate frames for base and TCP
function createCoordinateFrames() {
    // Base frame (fixed at origin) - standard coordinate system
    baseFrame = createCoordinateFrame(0.3, 'Base');
    baseFrame.position.set(0, 0, 0);
    worldGroup.add(baseFrame); // Add to world group instead of scene
    
    // TCP frame (will be updated with robot data) - standard coordinate system
    tcpFrame = createCoordinateFrame(0.2, 'TCP');
    tcpFrame.position.set(0.5, 0.5, 0.5);
    worldGroup.add(tcpFrame); // Add to world group instead of scene
}

// Create a coordinate frame with colored axes
function createCoordinateFrame(size, label) {
    const frame = new THREE.Group();
    
    // X axis (red) - points in positive X direction
    const xGeometry = new THREE.CylinderGeometry(0.005, 0.005, size, 8);
    const xMaterial = new THREE.MeshLambertMaterial({ color: 0xff0000 });
    const xAxis = new THREE.Mesh(xGeometry, xMaterial);
    xAxis.rotation.z = -Math.PI / 2; // Rotate cylinder to point along X-axis
    xAxis.position.x = size / 2; // Center it at half the length
    frame.add(xAxis);
    
    // X arrow - points in positive X direction
    const xArrowGeometry = new THREE.ConeGeometry(0.02, 0.04, 8);
    const xArrow = new THREE.Mesh(xArrowGeometry, xMaterial);
    xArrow.rotation.z = -Math.PI / 2; // Rotate cone to point along X-axis
    xArrow.position.x = size; // Place at end of axis
    frame.add(xArrow);
    
    // Y axis (green) - points in positive Y direction
    const yGeometry = new THREE.CylinderGeometry(0.005, 0.005, size, 8);
    const yMaterial = new THREE.MeshLambertMaterial({ color: 0x00ff00 });
    const yAxis = new THREE.Mesh(yGeometry, yMaterial);
    // No rotation needed, cylinder default is along Y-axis
    yAxis.position.y = size / 2; // Center it at half the length
    frame.add(yAxis);
    
    // Y arrow - points in positive Y direction
    const yArrowGeometry = new THREE.ConeGeometry(0.02, 0.04, 8);
    const yArrow = new THREE.Mesh(yArrowGeometry, yMaterial);
    // No rotation needed, cone default points along Y-axis
    yArrow.position.y = size; // Place at end of axis
    frame.add(yArrow);
    
    // Z axis (blue) - points in positive Z direction
    const zGeometry = new THREE.CylinderGeometry(0.005, 0.005, size, 8);
    const zMaterial = new THREE.MeshLambertMaterial({ color: 0x0000ff });
    const zAxis = new THREE.Mesh(zGeometry, zMaterial);
    zAxis.rotation.x = Math.PI / 2; // Rotate cylinder to point along Z-axis
    zAxis.position.z = size / 2; // Center it at half the length
    frame.add(zAxis);
    
    // Z arrow - points in positive Z direction
    const zArrowGeometry = new THREE.ConeGeometry(0.02, 0.04, 8);
    const zArrow = new THREE.Mesh(zArrowGeometry, zMaterial);
    zArrow.rotation.x = Math.PI / 2; // Rotate cone to point along Z-axis
    zArrow.position.z = size; // Place at end of axis
    frame.add(zArrow);
    
    // Center marker
    if (label) {
        const sphereGeometry = new THREE.SphereGeometry(0.02, 16, 16);
        const sphereMaterial = new THREE.MeshLambertMaterial({ color: 0xffffff });
        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        frame.add(sphere);
    }
    
    return frame;
}

// Update 3D visualization with robot state - DEPRECATED
// This function is no longer used to avoid duplicate updates causing flickering
// All TCP frame updates are now handled by updateTCPFramePosition()
function update3DVisualization(state) {
    // Function kept for reference but no longer used
    return;
}

// Animation loop with frame limiting for better performance
let lastRenderTime = 0;
const frameInterval = 1000 / settings.render3DRate; // ms between frames based on target FPS

function animate(timestamp) {
    if (!renderLoopActive) return;
    
    // Request next frame first for smoother animation
    requestAnimationFrame(animate);
    
    // Skip frames to maintain target frame rate
    const elapsed = timestamp - lastRenderTime;
    if (elapsed < frameInterval && !needsRender) return;
    
    // Update controls (OrbitControls)
    if (controls) {
        controls.update();
    }
    
    // Update directional light to follow camera
    if (directionalLight && camera) {
        const cameraDirection = new THREE.Vector3();
        camera.getWorldDirection(cameraDirection);
        
        // Position light slightly above and to the side of the camera
        const lightOffset = new THREE.Vector3(1, 1, 1);
        directionalLight.position.copy(camera.position).add(lightOffset);
        
        // Make the light look at the same point as the camera
        directionalLight.target.position.copy(camera.position).add(cameraDirection.multiplyScalar(5));
        directionalLight.target.updateMatrixWorld();
    }
    
    // Only render if needed (on data updates) or if controls are active
    if ((needsRender || controls.enableDamping || controls.enabled) && renderer && scene && camera) {
        renderer.render(scene, camera);
        needsRender = false;
        lastRenderTime = timestamp;
    }
}

// Handle window resize
function onWindowResize() {
    if (!camera || !renderer) return;
    
    const container = document.getElementById('viewer3d');
    if (!container) return;
    
    const containerWidth = container.offsetWidth;
    const containerHeight = container.offsetHeight;
    
    camera.aspect = containerWidth / containerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(containerWidth, containerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
}

// Handle keyboard shortcuts for 3D viewer
function handle3DKeyboard(event) {
    if (!isViewer3DInitialized || !camera || !controls) return;
    
    // Reset view with 'R' key
    if (event.key === 'r' || event.key === 'R') {
        event.preventDefault();
        resetCamera();
        logCommand('3D Viewer', 'Camera view reset');
    }
}

// Reset camera to original view
function resetCamera() {
    if (!camera || !controls) return;
    
    // Reset camera position for Z-up coordinate system
    camera.position.set(1, 1, 1);
    camera.lookAt(0, 0, 0);
    
    // Reset controls target
    controls.target.set(0, 0, 0);
    controls.update();
}

// Update state monitoring status
function updateStateStatus(isActive) {
    const statusElement = document.getElementById('stateStatus');
    if (isActive) {
        statusElement.textContent = 'Monitoring';
        statusElement.className = 'normal-indicator';
    } else {
        statusElement.textContent = 'Disconnected';
        statusElement.className = 'error-indicator';
    }
}

// Fetch robot arm information
async function fetchRobotArmInfo() {
    try {
        const response = await fetch('/api/robot_arm/info');
        const data = await response.json();
        
        document.getElementById('deviceId').textContent = data.device_id;
        document.getElementById('topicName').textContent = data.topic;
        
        logCommand('System', `Connected to device ${data.device_id}`);
        updateConnectionStatus(true);
    } catch (error) {
        console.error('Error fetching robot arm info:', error);
        updateConnectionStatus(false);
        logCommand('System', 'Failed to connect to robot arm service', 'error');
    }
}

// Send command to robot arm
async function sendCommand(command) {
    const statusElement = document.getElementById('commandStatus');
    const button = document.getElementById(getButtonId(command));
    
    // Disable button and show loading state
    if (button) {
        button.disabled = true;
        button.style.opacity = '0.6';
    }
    
    statusElement.textContent = `Sending command: ${command}...`;
    statusElement.className = 'text-sm text-blue-600';
    
    try {
        const response = await fetch('/api/robot_arm/cmd', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                command: command
            })
        });
        
        const result = await response.json();
        
        if (response.ok) {
            statusElement.textContent = `Command sent successfully: ${command}`;
            statusElement.className = 'text-sm text-green-600';
            logCommand('Command', command, 'success');
        } else {
            statusElement.textContent = `Error: ${result.error || 'Unknown error'}`;
            statusElement.className = 'text-sm text-red-600';
            logCommand('Command', command, 'error');
        }
    } catch (error) {
        console.error('Error sending command:', error);
        statusElement.textContent = `Network error: ${error.message}`;
        statusElement.className = 'text-sm text-red-600';
        logCommand('Command', command, 'error');
    } finally {
        // Re-enable button
        if (button) {
            button.disabled = false;
            button.style.opacity = '1';
        }
        
        // Clear status after 3 seconds
        setTimeout(() => {
            statusElement.textContent = 'Ready to send commands';
            statusElement.className = 'text-sm text-gray-600';
        }, 3000);
    }
}

// Get button ID for a command
function getButtonId(command) {
    const buttonMap = {
        'power_on': 'powerOnBtn',
        'power_off': 'powerOffBtn',
        'enable': 'enableBtn',
        'disable': 'disableBtn'
    };
    return buttonMap[command];
}

// Update connection status indicator
function updateConnectionStatus(connected) {
    const statusIndicator = document.getElementById('connectionStatus');
    const statusText = document.getElementById('connectionText');
    
    if (connected) {
        statusIndicator.className = 'status-indicator status-connected';
        statusText.textContent = 'Connected';
    } else {
        statusIndicator.className = 'status-indicator status-disconnected';
        statusText.textContent = 'Disconnected';
    }
}

// Log command to the command log
function logCommand(type, command, status = 'info') {
    const logElement = document.getElementById('commandLog');
    
    // Safety check - if element doesn't exist, log to console and return
    if (!logElement) {
        console.log(`[${type}] ${command} (${status})`);
        return;
    }
    
    const timestamp = new Date().toLocaleTimeString();
    
    commandCounter++;
    
    let statusIcon = '📝';
    let statusColor = 'text-blue-600';
    
    if (status === 'success') {
        statusIcon = '✅';
        statusColor = 'text-green-600';
    } else if (status === 'error') {
        statusIcon = '❌';
        statusColor = 'text-red-600';
    }
    
    const logEntry = document.createElement('div');
    logEntry.className = 'mb-2 p-2 bg-white rounded border-l-4 border-gray-300';
    logEntry.innerHTML = `
        <div class="flex justify-between items-start">
            <div>
                <span class="${statusColor}">${statusIcon}</span>
                <span class="font-semibold">[${type}]</span>
                <span class="ml-2">${command}</span>
            </div>
            <div class="text-xs text-gray-500">${timestamp}</div>
        </div>
    `;
    
    // Clear the initial placeholder if it exists
    if (logElement.children.length === 1 && logElement.firstChild.textContent.includes('Command log will appear here')) {
        logElement.innerHTML = '';
    }
    
    logElement.appendChild(logEntry);
    
    // Auto-scroll to bottom
    logElement.scrollTop = logElement.scrollHeight;
    
    // Keep only last 50 entries
    while (logElement.children.length > 50) {
        logElement.removeChild(logElement.firstChild);
    }
}

// Clear command log
function clearLog() {
    const logElement = document.getElementById('commandLog');
    logElement.innerHTML = '<div class="text-gray-500">Command log cleared...</div>';
    commandCounter = 0;
    logCommand('System', 'Command log cleared');
}

// Keyboard shortcuts
document.addEventListener('keydown', function(event) {
    if (event.ctrlKey) {
        switch(event.key) {
            case '1':
                event.preventDefault();
                sendCommand('power_on');
                break;
            case '2':
                event.preventDefault();
                sendCommand('power_off');
                break;
            case '3':
                event.preventDefault();
                sendCommand('enable');
                break;
            case '4':
                event.preventDefault();
                sendCommand('disable');
                break;
        }
    }
});

// Add keyboard shortcut hints
document.addEventListener('DOMContentLoaded', function() {
    const buttons = [
        {id: 'powerOnBtn', shortcut: 'Ctrl+1'},
        {id: 'powerOffBtn', shortcut: 'Ctrl+2'},
        {id: 'enableBtn', shortcut: 'Ctrl+3'},
        {id: 'disableBtn', shortcut: 'Ctrl+4'}
    ];
    
    buttons.forEach(button => {
        const element = document.getElementById(button.id);
        if (element) {
            element.title = `Click to execute or press ${button.shortcut}`;
        }
    });
});

// Periodic connection check
setInterval(fetchRobotArmInfo, 30000); // Check every 30 seconds

// Update URDF joint states from actual joint positions
function updateURDFFromActualJoints(jointActualPosition) {
    if (!jointActualPosition || !Array.isArray(jointActualPosition)) {
        return;
    }
    
    // Map joint actual positions to URDF joint names
    const jointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
    
    for (let i = 0; i < Math.min(jointActualPosition.length, jointNames.length); i++) {
        const jointName = jointNames[i];
        const actualValue = jointActualPosition[i];
        
        // Update the URDF joint state
        urdfJointStates[jointName] = actualValue;
    }
    
    // Update the robot pose in the 3D visualization
    updateRobotPose();
}
