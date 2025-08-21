// Robot Arm Web Control JavaScript

let commandCounter = 0;
let stateUpdateInterval;
let renderLoopActive = false;
let lastStateUpdate = null;
let updateCount = 0;
let updateRate = 0;
let lastUIUpdate = 0;
let robotStateData = null;

// Camera variables
let cameraUpdateInterval;
let cameraConnected = false;
let lastCameraUpdate = 0;

// Force-Torque Sensor variables
let ftSensorSocket = null;
let ftSensorData = [];
let ftSensorChart = null;
let ftSensorCtx = null;
let ftDataBuffer = {
    timestamps: [],
    forces: [[], [], []], // Fx, Fy, Fz
    torques: [[], [], []]  // Tx, Ty, Tz
};
let ftUpdateCounter = 0;
let ftLastUpdateTime = 0;
let ftUpdateRate = 0;
const FT_BUFFER_SIZE = 300; // Increased to 300 data points (about 3 seconds at 100Hz)
const FT_COLORS = ['#FF4444', '#00AA00', '#4488FF', '#FF44FF', '#FF8800', '#00FFFF']; // Force: Red, Green, Blue; Torque: Magenta, Orange, Cyan

// User configurable settings
const settings = {
    dataRefreshRate: 100,     // How often to fetch data from API (ms)
    uiRefreshRate: 500,      // How often to update the UI elements (ms)
    render3DRate: 30,        // Target FPS for 3D rendering
    animationEnabled: true,  // Whether to enable continuous rendering
    debug: false,            // Enable debug mode
    ftSensorEnabled: true,   // Enable FT sensor data reception
    ftUdpPort: 5566,         // UDP port for FT sensor data
    cameraRefreshRate: 100   // How often to update camera feed (ms)
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
    
    // Initialize camera status as disconnected initially
    updateCameraConnectionStatus(false);
    
    // Initialize camera stream
    setTimeout(() => {
        console.log('Initializing direct camera stream...');
        initializeCameraStream();
    }, 150);
    
    // Initialize Force-Torque Sensor Chart
    setTimeout(() => {
        console.log('Initializing FT Sensor Chart...');
        initFTSensorChart();
    }, 50);
    
    // Initialize FT Sensor UDP connection
    setTimeout(() => {
        console.log('Starting FT Sensor data reception...');
        startFTSensorUDP();
    }, 200);
    
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
    
    // Initialize long press controls after DOM is ready
    setTimeout(() => {
        console.log('Initializing long press controls...');
        initializeLongPressControls();
        
        // Add global event listeners to handle edge cases
        document.addEventListener('mouseup', clearAllLongPress);
        document.addEventListener('mouseleave', clearAllLongPress);
        document.addEventListener('touchend', clearAllLongPress);
        document.addEventListener('touchcancel', clearAllLongPress);
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) {
                clearAllLongPress();
            }
        });
    }, 500);
    
    // Initialize FT Sensor button status
    setTimeout(() => {
        console.log('Initializing FT Sensor button...');
        const button = document.getElementById('ftSensorToggleBtn');
        const buttonText = document.getElementById('ftSensorBtnText');
        if (button && buttonText) {
            if (settings.ftSensorEnabled) {
                buttonText.textContent = 'Stop Sensor';
                button.className = 'robot-arm-button bg-cyan-500 hover:bg-cyan-600 text-white font-bold py-3 px-4 rounded-lg transition-all';
            } else {
                buttonText.textContent = 'Start Sensor';
                button.className = 'robot-arm-button bg-gray-500 hover:bg-gray-600 text-white font-bold py-3 px-4 rounded-lg transition-all';
            }
        }
    }, 600);
    
    // Initialize tool control button status
    setTimeout(() => {
        console.log('Initializing tool control buttons...');
        // Initially disable all tool control buttons until program status is checked
        updateToolControlButtons([false, false, false, false, false]); // All false = disabled state
    }, 650);
    
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
            
            // Always update URDF joints immediately for smooth motion
            updateURDFJointsFromState(stateData);
            
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
            
            // Always update URDF joints immediately for smooth motion
            updateURDFJointsFromState(robotStateData);
            
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

// Update URDF joints from robot state data at high frequency
function updateURDFJointsFromState(stateData) {
    if (!stateData || !stateData.jointActualPosition || !Array.isArray(stateData.jointActualPosition)) {
        return;
    }
    
    // Map joint actual positions to URDF joint names
    const jointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
    
    for (let i = 0; i < Math.min(stateData.jointActualPosition.length, jointNames.length); i++) {
        const jointName = jointNames[i];
        const actualValue = stateData.jointActualPosition[i];
        
        // Update the URDF joint state
        urdfJointStates[jointName] = actualValue;
    }
    
    // Update the robot pose in the 3D visualization immediately
    updateRobotPose();
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
        document.getElementById('jointActualPositions').innerHTML = createCompactJointGrid(state.jointActualPosition);
        // Update URDF joint states from actual joint positions
        updateURDFFromActualJoints(state.jointActualPosition);
    }

    // Update Joint Actual Velocity
    if (state.jointActualVelocity) {
        document.getElementById('jointActualVelocities').innerHTML = createCompactJointGrid(state.jointActualVelocity);
    }

    // Update Joint Actual Acceleration
    if (state.jointActualAccelera) {
        document.getElementById('jointActualAccelerations').innerHTML = createCompactJointGrid(state.jointActualAccelera);
    }

    // Update Joint Actual Torque
    if (state.jointActualTorque) {
        document.getElementById('jointActualTorques').innerHTML = createCompactJointGrid(state.jointActualTorque);
    }

    // Update Joint Expected Position
    if (state.jointExpectPosition) {
        document.getElementById('jointExpectedPositions').innerHTML = createCompactJointGrid(state.jointExpectPosition);
    }

    // Update Joint Expected Velocity
    if (state.jointExpectVelocity) {
        document.getElementById('jointExpectedVelocities').innerHTML = createCompactJointGrid(state.jointExpectVelocity);
    }

    // Update Joint Temperatures
    if (state.jointActualTemperature) {
        // Only show first 6 joints for temperature
        const jointsToShow = state.jointActualTemperature.slice(0, 6);
        const tempHtml = jointsToShow.map((temp, index) => {
            return `<div class="compact-item"><span class="data-label">J${index + 1}:</span> <span class="data-value">${formatValue(temp, 1)}</span></div>`;
        }).join('');
        document.getElementById('jointTemperatures').innerHTML = tempHtml;
    }

    // Update Joint Currents
    if (state.jointActualCurrent) {
        // Only show first 6 joints for current
        const jointsToShow = state.jointActualCurrent.slice(0, 6);
        const currentHtml = jointsToShow.map((current, index) => {
            return `<div class="compact-item"><span class="data-label">J${index + 1}:</span> <span class="data-value">${formatValue(current, 0)}</span></div>`;
        }).join('');
        document.getElementById('jointCurrents').innerHTML = currentHtml;
    }

    // Update Driver Status
    if (state.driverErrorID) {
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
    }

    // Update Move Control Section
    updateMoveControlDisplay(state);

    // Update Coordinate Systems
    if (state.activeToolCoordSystem && state.activeToolCoordSystem.length >= 6) {
        const coordHtml = state.activeToolCoordSystem.map((value, index) => {
            const label = index < 3 ? ['X', 'Y', 'Z'][index] : ['Rx', 'Ry', 'Rz'][index - 3];
            return `<div class="compact-item"><span class="data-label">${label}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('activeToolCoordSystem').innerHTML = coordHtml;
    }

    if (state.activeWorkpieceCoordSystem && state.activeWorkpieceCoordSystem.length >= 6) {
        const coordHtml = state.activeWorkpieceCoordSystem.map((value, index) => {
            const label = index < 3 ? ['X', 'Y', 'Z'][index] : ['Rx', 'Ry', 'Rz'][index - 3];
            return `<div class="compact-item"><span class="data-label">${label}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('activeWorkpieceCoordSystem').innerHTML = coordHtml;
    }

    // Update Speed Settings
    if (state.blendedSpeed !== undefined) {
        // Store blendedSpeed value but don't display it (keep for data continuity)
        window.blendedSpeedValue = state.blendedSpeed;
    }
    if (state.globalSpeed !== undefined) {
        document.getElementById('globalSpeed').textContent = state.globalSpeed + '%';
        document.getElementById('globalSpeed').className = 'data-value';
    }
    if (state.jogSpeed !== undefined) {
        document.getElementById('jogSpeed').textContent = state.jogSpeed + '%';
        document.getElementById('jogSpeed').className = 'data-value';
    }

    // Update Digital IO
    if (state.functionalDigitalIOInput && Array.isArray(state.functionalDigitalIOInput)) {
        const ioHtml = state.functionalDigitalIOInput.map((value, index) => {
            const status = value ? '✅' : '❌';
            return `<div class="compact-item"><span class="data-label">FDI${index + 1}:</span> <span class="data-value">${status}</span></div>`;
        }).join('');
        document.getElementById('functionalDigitalIOInput').innerHTML = ioHtml;
    }

    if (state.functionalDigitalIOOutput && Array.isArray(state.functionalDigitalIOOutput)) {
        const ioHtml = state.functionalDigitalIOOutput.map((value, index) => {
            const status = value ? '✅' : '❌';
            return `<div class="compact-item"><span class="data-label">FDO${index + 1}:</span> <span class="data-value">${status}</span></div>`;
        }).join('');
        document.getElementById('functionalDigitalIOOutput').innerHTML = ioHtml;
    }

    if (state.digitalIOInput && Array.isArray(state.digitalIOInput)) {
        // Show first 16 DI inputs
        const ioHtml = state.digitalIOInput.slice(0, 16).map((value, index) => {
            const status = value ? '✅' : '❌';
            return `<div class="compact-item"><span class="data-label">DI${index + 1}:</span> <span class="data-value">${status}</span></div>`;
        }).join('');
        document.getElementById('digitalIOInput').innerHTML = ioHtml;
    }

    if (state.digitalIOOutput && Array.isArray(state.digitalIOOutput)) {
        // Show first 16 DO outputs
        const ioHtml = state.digitalIOOutput.slice(0, 16).map((value, index) => {
            const status = value ? '✅' : '❌';
            return `<div class="compact-item"><span class="data-label">DO${index + 1}:</span> <span class="data-value">${status}</span></div>`;
        }).join('');
        document.getElementById('digitalIOOutput').innerHTML = ioHtml;
    }

    // Update Analog IO
    if (state.analogInput && Array.isArray(state.analogInput)) {
        const analogHtml = state.analogInput.map((value, index) => {
            return `<div class="compact-item"><span class="data-label">AI${index + 1}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('analogInput').innerHTML = analogHtml;
    }

    if (state.analogOutput && Array.isArray(state.analogOutput)) {
        const analogHtml = state.analogOutput.map((value, index) => {
            return `<div class="compact-item"><span class="data-label">AO${index + 1}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('analogOutput').innerHTML = analogHtml;
    }

    // Update Tool IO
    if (state.toolIOInput && Array.isArray(state.toolIOInput)) {
        const toolIOHtml = Array.from({length: 8}, (_, index) => {
            const value = state.toolIOInput[index] !== undefined ? state.toolIOInput[index] : false;
            const status = value ? '✅' : '❌';
            return `<div class="compact-item"><span class="data-label">TI${index + 1}:</span> <span class="data-value">${status}</span></div>`;
        }).join('');
        document.getElementById('toolIOInput').innerHTML = toolIOHtml;
    }

    if (state.toolIOOutput && Array.isArray(state.toolIOOutput)) {
        const toolIOHtml = Array.from({length: 8}, (_, index) => {
            const value = state.toolIOOutput[index] !== undefined ? state.toolIOOutput[index] : 0;
            return `<div class="compact-item"><span class="data-label">TO${index + 1}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('toolIOOutput').innerHTML = toolIOHtml;
    }

    if (state.toolAnalogInput && Array.isArray(state.toolAnalogInput)) {
        const toolAnalogHtml = Array.from({length: 8}, (_, index) => {
            const value = state.toolAnalogInput[index] !== undefined ? state.toolAnalogInput[index] : 0;
            return `<div class="compact-item"><span class="data-label">TAI${index + 1}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('toolAnalogInput').innerHTML = toolAnalogHtml;
    }

    if (state.toolAnalogOutput && Array.isArray(state.toolAnalogOutput)) {
        const toolAnalogHtml = Array.from({length: 8}, (_, index) => {
            const value = state.toolAnalogOutput[index] !== undefined ? state.toolAnalogOutput[index] : 0;
            return `<div class="compact-item"><span class="data-label">TAO${index + 1}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('toolAnalogOutput').innerHTML = toolAnalogHtml;
    }

    if (state.toolButtonStatus && Array.isArray(state.toolButtonStatus)) {
        const buttonHtml = state.toolButtonStatus.map((value, index) => {
            const status = value ? '🔴' : '⚪';
            return `<div class="compact-item"><span class="data-label">Button${index + 1}:</span> <span class="data-value">${status}</span></div>`;
        }).join('');
        document.getElementById('toolButtonStatus').innerHTML = buttonHtml;
    }

    // Update Robot Status
    if (state.simulationMode !== undefined) {
        document.getElementById('simulationMode').textContent = state.simulationMode ? 'Simulation' : 'Real Robot';
        document.getElementById('simulationMode').className = state.simulationMode ? 'warning-indicator' : 'normal-indicator';
    }

    if (state.robotOperationMode !== undefined) {
        const operationModes = {0: "Manual", 1: "Auto", 2: "Remote"};
        document.getElementById('robotOperationMode').textContent = operationModes[state.robotOperationMode] || `Unknown (${state.robotOperationMode})`;
        document.getElementById('robotOperationMode').className = 'data-value';
    }

    if (state.robotStatus !== undefined) {
        const robotStates = {0: "Start", 1: "Initialize", 2: "Logout", 3: "Login", 
                           4: "PowerOff", 5: "Disable/PowerOn", 6: "Enable"};
        const statusText = robotStates[state.robotStatus] || `Unknown (${state.robotStatus})`;
        document.getElementById('robotStatus').textContent = statusText;
        document.getElementById('robotStatus').className = state.robotStatus === 6 ? 'normal-indicator' : 'warning-indicator';
    }

    if (state.robotProgramRunStatus !== undefined) {
        const programStates = {0: "Stopped", 1: "Stopping", 2: "Running", 
                             3: "Paused", 4: "Pausing", 5: "TaskRunning"};
        const statusText = programStates[state.robotProgramRunStatus] || `Unknown (${state.robotProgramRunStatus})`;
        document.getElementById('robotProgramRunStatus').textContent = statusText;
        document.getElementById('robotProgramRunStatus').className = [2, 5].includes(state.robotProgramRunStatus) ? 'normal-indicator' : 'data-value';
    }

    if (state.safetyMonitorStatus !== undefined) {
        const safetyStates = {0: "INIT", 2: "WAIT", 3: "CONFIG", 4: "POWER_OFF", 
                           5: "RUN", 6: "RECOVERY", 7: "STOP2", 8: "STOP1", 
                           9: "STOP0", 10: "MODEL", 12: "REDUCE", 13: "BOOT", 
                           14: "FAIL", 15: "UPDATE"};
        const statusText = safetyStates[state.safetyMonitorStatus] || `Unknown (${state.safetyMonitorStatus})`;
        document.getElementById('safetyMonitorStatus').textContent = statusText;
        document.getElementById('safetyMonitorStatus').className = state.safetyMonitorStatus === 5 ? 'normal-indicator' : 'warning-indicator';
    }

    if (state.collisionDetectionTrigger !== undefined) {
        document.getElementById('collisionDetectionTrigger').textContent = state.collisionDetectionTrigger ? 'Triggered' : 'Normal';
        document.getElementById('collisionDetectionTrigger').className = state.collisionDetectionTrigger ? 'error-indicator' : 'normal-indicator';
    }

    if (state.collisionAxis !== undefined) {
        document.getElementById('collisionAxis').textContent = state.collisionAxis === 0 ? 'None' : `Joint ${state.collisionAxis}`;
        document.getElementById('collisionAxis').className = state.collisionAxis === 0 ? 'normal-indicator' : 'error-indicator';
    }

    if (state.robotErrorCode !== undefined) {
        document.getElementById('robotErrorCode').textContent = state.robotErrorCode === 0 ? 'None' : `0x${state.robotErrorCode.toString(16).toUpperCase()}`;
        document.getElementById('robotErrorCode').className = state.robotErrorCode === 0 ? 'normal-indicator' : 'error-indicator';
    }

    // Update Float Registers (first 8 values)
    if (state.floatRegisterInput && Array.isArray(state.floatRegisterInput)) {
        const regHtml = state.floatRegisterInput.slice(0, 8).map((value, index) => {
            return `<div class="compact-item"><span class="data-label">FR_I${index + 1}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('floatRegisterInput').innerHTML = regHtml;
    }

    if (state.floatRegisterOutput && Array.isArray(state.floatRegisterOutput)) {
        const regHtml = state.floatRegisterOutput.slice(0, 8).map((value, index) => {
            return `<div class="compact-item"><span class="data-label">FR_O${index + 1}:</span> <span class="data-value">${formatValue(value)}</span></div>`;
        }).join('');
        document.getElementById('floatRegisterOutput').innerHTML = regHtml;
    }

    // Update Tool Status (Bool Register Output bits 1-5 corresponding to frames 1104-1119)
    // Five items: 4 tools + 1 program status from first 5 bits of boolRegisterOutput
    if (state.boolRegisterOutput && Array.isArray(state.boolRegisterOutput)) {
        updateToolStatus(state.boolRegisterOutput);
    } else {
        // If no data available, set all tools to AWAY and program to RUNNING
        updateToolStatus([false, false, false, false, false]);
    }

    // Update last update time
    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
    document.getElementById('dataStatus').textContent = 'Active';
    document.getElementById('dataStatus').className = 'normal-indicator';

    // Update 3D visualization - removed duplicate TCP update to prevent flickering
}

// Update Tool Status based on Bool Register Output (frames 1104-1119, first 5 bits)
function updateToolStatus(boolRegisterOutput) {
    // Tool mapping: Gripper (bit 1), Frame (bit 2), StickP (bit 3), StickR (bit 4), Program (bit 5)
    // Using the first 5 bits of the boolRegisterOutput array
    const tools = [
        { id: 'gripperStatus', name: 'Gripper', bit: 0 },        // Bit 1 (index 0)
        { id: 'frameStatus', name: 'Frame', bit: 1 },  // Bit 2 (index 1)
        { id: 'stickPStatus', name: 'StickP', bit: 2 },  // Bit 3 (index 2)
        { id: 'stickRStatus', name: 'StickR', bit: 3 },  // Bit 4 (index 3)
        { id: 'programStatus', name: 'Program', bit: 4 } // Bit 5 (index 4) - inverted logic
    ];

    tools.forEach(tool => {
        const statusElement = document.getElementById(tool.id);
        if (statusElement) {
            let isActive;
            
            // Program status has inverted logic: 0=running, 1=completed
            if (tool.id === 'programStatus') {
                isActive = boolRegisterOutput[tool.bit] === false || boolRegisterOutput[tool.bit] === 0;
            } else {
                isActive = boolRegisterOutput[tool.bit] === true || boolRegisterOutput[tool.bit] === 1;
            }
            
            // Update status dot
            const statusDot = statusElement.querySelector('.status-dot');
            const statusText = statusElement.querySelector('.status-text');
            
            if (statusDot && statusText) {
                if (tool.id === 'programStatus') {
                    // Program status: active=running, inactive=completed
                    if (isActive) {
                        statusDot.classList.remove('inactive');
                        statusDot.classList.add('active');
                        statusText.classList.add('active');
                        statusText.textContent = 'COMPLETED';
                    } else {
                        statusDot.classList.remove('active');
                        statusDot.classList.add('inactive');
                        statusText.classList.remove('active');
                        statusText.textContent = 'RUNNING';
                    }
                } else {
                    // Tool status: active=home, inactive=away
                    if (isActive) {
                        statusDot.classList.remove('inactive');
                        statusDot.classList.add('active');
                        statusText.classList.add('active');
                        statusText.textContent = 'HOME';
                    } else {
                        statusDot.classList.remove('active');
                        statusDot.classList.add('inactive');
                        statusText.classList.remove('active');
                        statusText.textContent = 'AWAY';
                    }
                }
            }
        }
    });
    
    // Update tool control buttons based on program status
    updateToolControlButtons(boolRegisterOutput);
}

// Global variable to track if user clicked a tool button
let toolControlClickActive = false;
let toolControlClickTimeout = null;

// Update tool control buttons availability based on program status
function updateToolControlButtons(boolRegisterOutput, forceUpdate = false) {
    // Check if program is completed (bit 4 = false/0)
    const isProgramCompleted = boolRegisterOutput[4] === false || boolRegisterOutput[4] === 0;
    
    const toolButtons = [
        'gripperControlBtn',
        'frameControlBtn', 
        'stickPControlBtn',
        'stickRControlBtn',
        'homingToolsBtn',
        'rotateBtn',
        'pushBoxBtn'
    ];
    
    const statusElement = document.getElementById('toolControlStatus');
    
    toolButtons.forEach(buttonId => {
        const button = document.getElementById(buttonId);
        if (button) {
            button.disabled = !isProgramCompleted;
            
            if (isProgramCompleted) {
                button.classList.remove('disabled:from-gray-400', 'disabled:to-gray-500', 'disabled:cursor-not-allowed', 'disabled:shadow-none');
            } else {
                button.classList.add('disabled:from-gray-400', 'disabled:to-gray-500', 'disabled:cursor-not-allowed', 'disabled:shadow-none');
            }
        }
    });
    
    // Update status text with tool states - only if not showing click feedback or forced update
    if (statusElement && (!toolControlClickActive || forceUpdate)) {
        if (isProgramCompleted) {
            statusElement.textContent = 'Tool controls are enabled - Program completed';
            statusElement.className = 'text-sm text-green-600';
        } else {
            statusElement.textContent = 'Tool controls are disabled - Program must be completed';
            statusElement.className = 'text-sm text-gray-600';
        }
    }
}

// Find shortest path using BFS algorithm (similar to duco_test_tool.py)
function findShortestPath(currentState, targetState, validStates) {
    // Convert states to string for comparison
    const currentStateStr = currentState.join(',');
    const targetStateStr = targetState.join(',');
    
    // If already at target state
    if (currentStateStr === targetStateStr) {
        return [];
    }
    
    // BFS queue: [state, path]
    const queue = [[currentState.slice(), []]];
    const visited = new Set([currentStateStr]);
    
    // Define all possible actions (same as duco_test_tool.py)
    const actions = [
        {
            name: 'zero2gripper',
            apply: (state) => state[0] === 1 ? [0, state[1], state[2], state[3]] : null
        },
        {
            name: 'gripper2zero', 
            apply: (state) => state[0] === 0 ? [1, state[1], state[2], state[3]] : null
        },
        {
            name: 'zero2frame',
            apply: (state) => state[1] === 1 ? [state[0], 0, state[2], state[3]] : null
        },
        {
            name: 'frame2zero',
            apply: (state) => state[1] === 0 ? [state[0], 1, state[2], state[3]] : null
        },
        {
            name: 'zero2stickP',
            apply: (state) => state[2] === 1 ? [state[0], state[1], 0, state[3]] : null
        },
        {
            name: 'stickP2zero',
            apply: (state) => state[2] === 0 ? [state[0], state[1], 1, state[3]] : null
        },
        {
            name: 'zero2stickR',
            apply: (state) => state[3] === 1 ? [state[0], state[1], state[2], 0] : null
        },
        {
            name: 'stickR2zero',
            apply: (state) => state[3] === 0 ? [state[0], state[1], state[2], 1] : null
        }
    ];
    
    while (queue.length > 0) {
        const [currentState, path] = queue.shift();
        
        // Try all possible actions
        for (const action of actions) {
            const newState = action.apply(currentState);
            
            if (newState) {
                const newStateStr = newState.join(',');
                
                // Check if new state is valid and not visited
                if (validStates.has(newStateStr) && !visited.has(newStateStr)) {
                    const newPath = [...path, action.name];
                    
                    // If reached target state
                    if (newStateStr === targetStateStr) {
                        return newPath;
                    }
                    
                    queue.push([newState, newPath]);
                    visited.add(newStateStr);
                }
            }
        }
    }
    
    // No path found
    return ['No valid path found'];
}

// Handle tool control button clicks
function handleToolControlClick(toolType) {
    console.log(`Tool control clicked: ${toolType}`);
    
    // Clear any existing timeout
    if (toolControlClickTimeout) {
        clearTimeout(toolControlClickTimeout);
        toolControlClickTimeout = null;
    }
    
    // Set click active flag
    toolControlClickActive = true;
    
    // Define target states based on duco_test_tool.py
    const targetStates = {
        'gripper': [0, 1, 1, 1],      // A: gripper
        'frame': [1, 0, 1, 1],   // B: frame  
        'stickP': [0, 1, 0, 1],   // C: gripper + stickP
        'stickR': [0, 1, 1, 0],   // D: gripper + stickR
        'homing': [1, 1, 1, 1]    // Reset to default state
    };
    
    // Define valid states
    const validStates = new Set([
        '1,1,1,1',  // 1111 - All tools are at home position
        '0,1,1,1',  // 0111 - Get Gripper
        '1,0,1,1',  // 1011 - Get Frame
        '0,1,0,1',  // 0101 - Get StickP(must operate after get gripper)
        '0,1,1,0'   // 0110 - Get StickR(must operate after get gripper)
    ]);
    
    // Get current robot state to display tool status
    if (robotStateData && robotStateData.boolRegisterOutput) {
        const boolRegisterOutput = robotStateData.boolRegisterOutput;
        const gripperState = boolRegisterOutput[0] ? 1 : 0;
        const frameState = boolRegisterOutput[1] ? 1 : 0;
        const stickPState = boolRegisterOutput[2] ? 1 : 0;
        const stickRState = boolRegisterOutput[3] ? 1 : 0;
        
        const currentState = [gripperState, frameState, stickPState, stickRState];
        const targetState = targetStates[toolType] || [1, 1, 1, 1];
        
        // Find shortest path using BFS (similar to duco_test_tool.py)
        const path = findShortestPath(currentState, targetState, validStates);
        
        // Create tool name mapping for display
        const toolDisplayNames = {
            'gripper': 'Get Gripper',
            'frame': 'Get Frame',
            'stickP': 'Get StickP',
            'stickR': 'Get StickR',
            'homing': 'Homing Tools'
        };
        
        const toolDisplayName = toolDisplayNames[toolType] || toolType;
        const pathText = path.length > 0 ? `path: ${path.join(' → ')}` : 'Already at target state';
        
        const statusElement = document.getElementById('toolControlStatus');
        if (statusElement) {
            statusElement.innerHTML = `<div class="text-blue-600 font-medium">"${toolDisplayName}" clicked, ${pathText}</div>`;
            statusElement.className = 'text-sm';
            
            // Execute the action sequence if path is found and not empty
            if (path.length > 0 && path[0] !== 'No valid path found') {
                executeActionSequence(path, statusElement);
            }
            
            // Reset to normal status after 5 seconds (or longer if executing actions)
            const resetDelay = path.length > 0 && path[0] !== 'No valid path found' ? 
                               Math.max(5000, path.length * 2000) : 3000;
            toolControlClickTimeout = setTimeout(() => {
                toolControlClickActive = false;
                updateToolControlButtons(boolRegisterOutput, true); // Force update
            }, resetDelay);
        }
    }
    
    // Add your tool control logic here
    // For example, send commands to the robot based on toolType
    switch (toolType) {
        case 'gripper':
            console.log('Executing Get gripper command');
            // Add gripper control command here
            break;
        case 'frame':
            console.log('Executing Get frame command');
            // Add frame control command here
            break;
        case 'stickP':
            console.log('Executing Get stickP command');
            // Add stickP control command here
            break;
        case 'stickR':
            console.log('Executing Get stickR command');
            // Add stickR control command here
            break;
        case 'homing':
            console.log('Executing homing tools command');
            // Add homing tools command here
            break;
        default:
            console.log('Unknown tool type:', toolType);
    }
}

// Execute action sequence by sending task commands
async function executeActionSequence(actionPath, statusElement) {
    console.log('Executing action sequence:', actionPath);
    
    // Map BFS action names to task command names (matching sendTaskCommand mapping)
    const actionToTaskCommand = {
        'zero2gripper': 'task_zero2gripper',
        'gripper2zero': 'task_gripper2zero',
        'zero2frame': 'task_zero2frame',
        'frame2zero': 'task_frame2zero',
        'zero2stickP': 'task_zero2stickP',
        'stickP2zero': 'task_stickP2zero',
        'zero2stickR': 'task_zero2stickR',
        'stickR2zero': 'task_stickR2zero'
    };
    
    try {
        // Update status to show execution started
        if (statusElement) {
            const currentHTML = statusElement.innerHTML;
            statusElement.innerHTML = currentHTML + `<div id="action-progress" class="text-purple-600 text-sm font-medium mt-1">⚙️ Executing actions...</div>`;
        }
        
        // Execute each action in sequence
        for (let i = 0; i < actionPath.length; i++) {
            const action = actionPath[i];
            const taskCommand = actionToTaskCommand[action];
            
            if (taskCommand) {
                console.log(`Executing step ${i + 1}/${actionPath.length}: ${action} -> ${taskCommand}`);
                
                // Update status to show current action
                if (statusElement) {
                    const progressElement = statusElement.querySelector('#action-progress');
                    if (progressElement) {
                        progressElement.innerHTML = `⚙️ Executing ${i + 1}/${actionPath.length}: ${action}`;
                    }
                }
                
                // Send the task command
                await sendTaskCommand(taskCommand);
                
                // Wait for program to complete before sending next command
                console.log(`Waiting for program completion after ${action}...`);
                await waitForProgramCompletion();
                
                console.log(`Completed step ${i + 1}/${actionPath.length}: ${action}`);
            } else {
                console.error(`Unknown action: ${action}`);
                if (statusElement) {
                    const progressElement = statusElement.querySelector('#action-progress');
                    if (progressElement) {
                        progressElement.innerHTML = `❌ Error: Unknown action ${action}`;
                        progressElement.className = 'text-red-600 text-sm font-medium mt-1';
                    }
                }
                return;
            }
        }
        
        // Update status to show completion
        if (statusElement) {
            const progressElement = statusElement.querySelector('#action-progress');
            if (progressElement) {
                progressElement.innerHTML = `✅ All actions completed successfully`;
                progressElement.className = 'text-green-600 text-sm font-medium mt-1';
            }
        }
        
        console.log('Action sequence completed successfully');
        
    } catch (error) {
        console.error('Error executing action sequence:', error);
        if (statusElement) {
            const progressElement = statusElement.querySelector('#action-progress');
            if (progressElement) {
                progressElement.innerHTML = `❌ Error: ${error.message}`;
                progressElement.className = 'text-red-600 text-sm font-medium mt-1';
            } else {
                // Fallback if progress element not found
                const currentHTML = statusElement.innerHTML;
                statusElement.innerHTML = currentHTML + `<div class="text-red-600 text-sm font-medium mt-1">❌ Error: ${error.message}</div>`;
            }
        }
    }
}

// Send Tool Control API command (for use by external scripts via JSON API)
async function sendToolControlAPI(toolType, options = {}) {
    try {
        const response = await fetch('/api/tool_control/execute', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                tool_type: toolType,
                wait_completion: options.waitCompletion || false,
                timeout: options.timeout || 100000
            })
        });
        
        const result = await response.json();
        
        if (response.ok) {
            console.log(`Tool control API success: ${toolType}`, result);
            return {
                success: true,
                message: result.message || 'Tool control executed successfully',
                path: result.path || [],
                data: result
            };
        } else {
            console.error(`Tool control API error: ${toolType}`, result);
            return {
                success: false,
                error: result.error || 'Unknown error occurred',
                data: result
            };
        }
    } catch (error) {
        console.error('Tool control API network error:', error);
        return {
            success: false,
            error: `Network error: ${error.message}`,
            data: null
        };
    }
}

// Wait for robot program to complete execution by monitoring boolRegisterOutput[4]
async function waitForProgramCompletion(timeout = 100000) {
    return new Promise((resolve, reject) => {
        const startTime = Date.now();
        const maxWaitTime = timeout; // Maximum wait time in milliseconds
        let programStarted = false;
        let initialDelayCompleted = false;
        
        console.log('Starting program completion wait...');
        
        const checkCompletion = () => {
            // Check timeout
            if (Date.now() - startTime > maxWaitTime) {
                console.error('Timeout waiting for program completion');
                reject(new Error('Timeout waiting for program completion'));
                return;
            }
            
            // Add initial delay to allow command to be processed
            if (!initialDelayCompleted) {
                if (Date.now() - startTime < 200) { // Wait 200ms initially
                    setTimeout(checkCompletion, 50);
                    return;
                } else {
                    initialDelayCompleted = true;
                    console.log('Initial delay completed, starting status monitoring...');
                }
            }
            
            // Check if we have robot state data
            if (robotStateData && robotStateData.boolRegisterOutput && Array.isArray(robotStateData.boolRegisterOutput)) {
                // Program status logic: 
                // true/1 = program started/running, false/0 = program completed/idle
                const bit4Value = robotStateData.boolRegisterOutput[4];
                const isProgramActive = bit4Value === true || bit4Value === 1;
                const isProgramIdle = bit4Value === false || bit4Value === 0;
                
                console.log(`Program status check: boolRegisterOutput[4] = ${bit4Value}, active=${isProgramActive}, idle=${isProgramIdle}, started=${programStarted}`);
                
                // Wait for program to start (bit goes to 1)
                if (!programStarted && isProgramActive) {
                    programStarted = true;
                    console.log('Program execution started detected (boolRegisterOutput[4] = 1)');
                }
                
                // Wait for program to complete (bit goes back to 0) 
                if (programStarted && isProgramIdle) {
                    console.log('Program execution completed (boolRegisterOutput[4] = 0), ready for next command');
                    resolve();
                    return;
                } else if (programStarted) {
                    console.log('Program still running (boolRegisterOutput[4] = 1), waiting...');
                } else if (!programStarted && isProgramIdle) {
                    // If program shows as idle before we see it starting, wait for it to start
                    console.log('Waiting for program to start (currently idle, boolRegisterOutput[4] = 0)...');
                } else {
                    console.log('Waiting for program state change...');
                }
            } else {
                console.log('No robot state data available, waiting...');
            }
            
            // Check again after a short delay
            setTimeout(checkCompletion, 100); // Check every 100ms
        };
        
        // Start checking
        checkCompletion();
    });
}

// Update Move Control Display Section
function updateMoveControlDisplay(state) {
    // Helper function to safely format values with higher precision for Move Control
    const formatMoveValue = (value, decimals = 3) => {
        if (value === null || value === undefined || isNaN(value)) return '-';
        return parseFloat(value).toFixed(decimals);
    };

    // Helper function to convert radians to degrees
    const radToDeg = (rad) => {
        if (rad === null || rad === undefined || isNaN(rad)) return '-';
        return (parseFloat(rad) * 180 / Math.PI).toFixed(2) + '°';
    };

    // Update Joint Angles (convert from radians to degrees for better readability)
    if (state.jointActualPosition && state.jointActualPosition.length >= 6) {
        const joints = state.jointActualPosition;
        for (let i = 0; i < 6; i++) {
            const element = document.getElementById(`moveJoint${i + 1}`);
            if (element) {
                const degreeValue = parseFloat(joints[i]) * 180 / Math.PI;
                // Only update if the input is not currently focused (to avoid interrupting user input)
                if (document.activeElement !== element) {
                    element.value = degreeValue.toFixed(1);
                }
                // Update internal tracking variable
                currentJointValues[i] = degreeValue;
            }
        }
    }

    // Update TCP Position (with mm units for position, degrees for orientation)
    if (state.TCPActualPosition && state.TCPActualPosition.length >= 6) {
        const tcp = state.TCPActualPosition;
        
        // Position (assuming meters, convert to mm)
        const posElements = ['moveTcpX', 'moveTcpY', 'moveTcpZ'];
        const posAxes = ['x', 'y', 'z'];
        for (let i = 0; i < 3; i++) {
            const element = document.getElementById(posElements[i]);
            if (element) {
                const valueInMm = parseFloat(tcp[i]) * 1000;
                // Only update if the input is not currently focused
                if (document.activeElement !== element) {
                    element.value = valueInMm.toFixed(1);
                }
                // Update internal tracking variable
                currentTcpValues[posAxes[i]] = valueInMm;
            }
        }
        
        // Orientation (convert radians to degrees)
        const orientElements = ['moveTcpRx', 'moveTcpRy', 'moveTcpRz'];
        const orientAxes = ['rx', 'ry', 'rz'];
        for (let i = 3; i < 6; i++) {
            const element = document.getElementById(orientElements[i - 3]);
            if (element) {
                const degreeValue = parseFloat(tcp[i]) * 180 / Math.PI;
                // Only update if the input is not currently focused
                if (document.activeElement !== element) {
                    element.value = degreeValue.toFixed(1);
                }
                // Update internal tracking variable
                currentTcpValues[orientAxes[i - 3]] = degreeValue;
            }
        }
    }

    // Update Move Control status and timestamp - removed as requested
}

// ===== Force-Torque Sensor Functions =====

// Initialize FT Sensor Chart
function initFTSensorChart() {
    try {
        ftSensorChart = document.getElementById('ftSensorChart');
        if (!ftSensorChart) {
            console.error('FT Sensor chart canvas not found');
            return;
        }
        
        ftSensorCtx = ftSensorChart.getContext('2d');
        
        // Set canvas size
        const container = ftSensorChart.parentElement;
        const rect = container.getBoundingClientRect();
        ftSensorChart.width = rect.width || 480;
        ftSensorChart.height = 400;
        
        console.log('FT Sensor chart initialized successfully');
        
        // Start rendering loop
        requestAnimationFrame(renderFTSensorChart);
        
    } catch (error) {
        console.error('Failed to initialize FT Sensor chart:', error);
    }
}

// Start FT Sensor UDP data reception
function startFTSensorUDP() {
    // Only try WebSocket if FT sensor is enabled
    if ("WebSocket" in window && settings.ftSensorEnabled) {
        console.log('Attempting to connect to FT Sensor WebSocket...');
        connectFTSensorWebSocket();
    } else {
        console.log('FT Sensor disabled or WebSocket not supported, using simulation...');
        startFTSensorSimulation();
    }
}

// Connect to FT Sensor WebSocket
function connectFTSensorWebSocket() {
    // Close existing socket if any
    if (ftSensorSocket) {
        ftSensorSocket.close();
    }
    
    // Create WebSocket connection for FT sensor data
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/ft_sensor`;
    
    ftSensorSocket = new WebSocket(wsUrl);
    
    ftSensorSocket.onopen = function(e) {
        console.log('FT Sensor WebSocket connection opened');
        // Request latest data
        ftSensorSocket.send('get_latest');
    };
    
    ftSensorSocket.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            if (data.FTSensorData) {
                processFTSensorData(data);
            }
        } catch (error) {
            console.error('Error parsing FT sensor data:', error);
        }
    };
    
    ftSensorSocket.onclose = function(event) {
        console.log('FT Sensor WebSocket connection closed');
        // Only attempt to reconnect if FT sensor is enabled and we're not using simulation
        setTimeout(() => {
            if (settings.ftSensorEnabled && !ftSensorSocket) {
                console.log('Attempting to reconnect FT Sensor WebSocket...');
                connectFTSensorWebSocket();
            }
        }, 5000); // Increased to 5 seconds to reduce log spam
    };
    
    ftSensorSocket.onerror = function(error) {
        console.error('FT Sensor WebSocket error:', error);
        // Close the socket and fall back to simulation
        if (ftSensorSocket) {
            ftSensorSocket.close();
            ftSensorSocket = null;
        }
        console.log('Falling back to FT sensor simulation...');
        startFTSensorSimulation();
    };
}

// Simulate FT Sensor data (for testing purposes)
function startFTSensorSimulation() {
    // Only start simulation if FT sensor is enabled and no WebSocket connection exists
    if (!settings.ftSensorEnabled || (ftSensorSocket && ftSensorSocket.readyState === WebSocket.OPEN)) {
        return;
    }
    
    console.log('Starting FT Sensor data simulation...');
    
    const simulateData = () => {
        // Stop simulation if FT sensor is disabled or WebSocket connection is established
        if (!settings.ftSensorEnabled || (ftSensorSocket && ftSensorSocket.readyState === WebSocket.OPEN)) {
            console.log('Stopping FT Sensor simulation');
            return;
        }
        
        const now = Date.now();
        const time = now / 1000; // Convert to seconds
        
        // Simulate FT sensor data with some realistic patterns
        const ftData = {
            FTSensorData: [
                Math.sin(time * 0.5) * 10 + Math.random() * 2,      // Fx
                Math.cos(time * 0.3) * 15 + Math.random() * 2,      // Fy  
                Math.sin(time * 0.2) * 8 + Math.random() * 1,       // Fz
                Math.cos(time * 0.4) * 5 + Math.random() * 0.5,     // Tx
                Math.sin(time * 0.6) * 7 + Math.random() * 0.5,     // Ty
                Math.cos(time * 0.8) * 4 + Math.random() * 0.5      // Tz
            ]
        };
        
        processFTSensorData(ftData);
        
        setTimeout(simulateData, 50); // 20Hz simulation (reduced from 50Hz)
    };
    
    simulateData();
}

// Process incoming FT Sensor data
function processFTSensorData(data) {
    if (!data.FTSensorData || !Array.isArray(data.FTSensorData)) {
        return;
    }
    
    const timestamp = Date.now();
    const ftValues = data.FTSensorData;
    
    // Update data buffer
    ftDataBuffer.timestamps.push(timestamp);
    
    // Add force data (first 3 values)
    for (let i = 0; i < 3; i++) {
        if (i < ftValues.length) {
            ftDataBuffer.forces[i].push(ftValues[i]);
        }
    }
    
    // Add torque data (last 3 values)
    for (let i = 0; i < 3; i++) {
        if (i + 3 < ftValues.length) {
            ftDataBuffer.torques[i].push(ftValues[i + 3]);
        }
    }
    
    // Trim buffer to max size with smooth removal (remove older data in batches)
    if (ftDataBuffer.timestamps.length > FT_BUFFER_SIZE) {
        const excessCount = ftDataBuffer.timestamps.length - FT_BUFFER_SIZE;
        const removeCount = Math.min(excessCount, Math.max(1, Math.floor(FT_BUFFER_SIZE * 0.1))); // Remove 10% at most
        
        for (let j = 0; j < removeCount; j++) {
            ftDataBuffer.timestamps.shift();
            for (let i = 0; i < 3; i++) {
                ftDataBuffer.forces[i].shift();
                ftDataBuffer.torques[i].shift();
            }
        }
    }
    
    // Update statistics
    ftUpdateCounter++;
    const now = Date.now();
    if (now - ftLastUpdateTime >= 1000) {
        ftUpdateRate = ftUpdateCounter;
        ftUpdateCounter = 0;
        ftLastUpdateTime = now;
    }
}

// Render FT Sensor Chart
// Render FT Sensor Chart with frame rate limiting and anti-flicker optimization
let lastFTRenderTime = 0;
const FT_RENDER_INTERVAL = 66; // Reduced to ~15 FPS (1000ms / 15) for ultra-smooth appearance

function renderFTSensorChart(timestamp) {
    if (!ftSensorCtx || !ftSensorChart) {
        return;
    }
    
    // Limit frame rate to 15 FPS for maximum smoothness
    if (timestamp - lastFTRenderTime < FT_RENDER_INTERVAL) {
        requestAnimationFrame(renderFTSensorChart);
        return;
    }
    lastFTRenderTime = timestamp;
    
    const ctx = ftSensorCtx;
    const width = ftSensorChart.width;
    const height = ftSensorChart.height;
    
    // Clear canvas with smooth transition
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);
    
    // Draw grid
    drawGrid(ctx, width, height);
    
    // Draw data if available
    if (ftDataBuffer.timestamps.length > 1) {
        const timeRange = 3000; // Increased to 3 seconds for smoother transitions
        const currentTime = Date.now();
        const startTime = currentTime - timeRange;
        
        // Draw force data (top half)
        drawFTData(ctx, ftDataBuffer.forces, ftDataBuffer.timestamps, 
                  0, 0, width, height / 2, startTime, currentTime, [-50, 50], 'Forces (N)', 0);
        
        // Draw torque data (bottom half)
        drawFTData(ctx, ftDataBuffer.torques, ftDataBuffer.timestamps,
                  0, height / 2, width, height / 2, startTime, currentTime, [-20, 20], 'Torques (Nm)', 3);
    }
    
    // Continue animation
    requestAnimationFrame(renderFTSensorChart);
}

// Draw grid on chart with softer appearance
function drawGrid(ctx, width, height) {
    // Minor grid lines
    ctx.strokeStyle = '#333333';
    ctx.lineWidth = 0.5;
    ctx.globalAlpha = 0.5;
    
    // Vertical lines (time) - minor
    for (let i = 1; i < 20; i++) {
        if (i % 4 !== 0) { // Skip major grid lines
            const x = (i / 20) * width;
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
            ctx.stroke();
        }
    }
    
    // Horizontal lines - minor
    for (let i = 1; i < 20; i++) {
        if (i % 4 !== 0) { // Skip major grid lines
            const y = (i / 20) * height;
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }
    }
    
    // Major grid lines
    ctx.strokeStyle = '#555555';
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.8;
    
    // Vertical lines (time) - major
    for (let i = 0; i <= 20; i += 4) {
        const x = (i / 20) * width;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }
    
    // Horizontal lines - major
    for (let i = 0; i <= 20; i += 4) {
        const y = (i / 20) * height;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }
    
    // Center line (zero line)
    ctx.strokeStyle = '#AAAAAA';
    ctx.lineWidth = 1.5;
    ctx.globalAlpha = 1.0;
    ctx.beginPath();
    ctx.moveTo(0, height / 2);
    ctx.lineTo(width, height / 2);
    ctx.stroke();
    
    // Reset alpha
    ctx.globalAlpha = 1.0;
}

// Draw FT data curves with smooth transitions
function drawFTData(ctx, dataArrays, timestamps, x, y, w, h, startTime, endTime, range, title, colorOffset = 0) {
    ctx.save();
    ctx.translate(x, y);
    
    // Reserve space for labels at top and bottom
    const topMargin = 15;
    const bottomMargin = 10;
    const chartHeight = h - topMargin - bottomMargin;
    
    // Draw title
    ctx.fillStyle = '#FFFFFF';
    ctx.font = '14px Arial';
    ctx.fillText(title, 10, 20);
    
    // Draw Y-axis scale labels with better distribution
    ctx.fillStyle = '#CCCCCC';
    ctx.font = '9px Arial';
    ctx.textAlign = 'right';
    
    const numTicks = 5; // Number of tick marks
    for (let i = 0; i <= numTicks; i++) {
        const ratio = i / numTicks;
        const value = range[0] + (range[1] - range[0]) * ratio;
        // Calculate y position with margins
        const y = topMargin + chartHeight - (ratio * chartHeight);
        
        // Draw tick mark
        ctx.strokeStyle = '#666666';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(w - 45, y);
        ctx.lineTo(w - 40, y);
        ctx.stroke();
        
        // Draw label
        ctx.fillText(value.toFixed(1), w - 47, y + 3);
    }
    
    // Reset text align for other text
    ctx.textAlign = 'left';
    
    // Draw data curves with smooth transitions and fade effects
    for (let curveIndex = 0; curveIndex < dataArrays.length; curveIndex++) {
        const data = dataArrays[curveIndex];
        if (data.length < 2) continue;
        
        // Collect all points with extended time range for smooth fade
        const fadeMargin = (endTime - startTime) * 0.15; // 15% fade margin
        const extendedStartTime = startTime - fadeMargin;
        const extendedEndTime = endTime + fadeMargin;
        
        const allPoints = [];
        for (let i = 0; i < data.length; i++) {
            if (i >= timestamps.length) break;
            
            const timestamp = timestamps[i];
            if (timestamp < extendedStartTime || timestamp > extendedEndTime) continue;
            
            const timeRatio = (timestamp - startTime) / (endTime - startTime);
            const valueRatio = Math.max(0, Math.min(1, (data[i] - range[0]) / (range[1] - range[0])));
            
            const px = timeRatio * w;
            const py = topMargin + chartHeight - (valueRatio * chartHeight);
            
            // Calculate alpha based on position for fade effect with enhanced smoothness
            let alpha = 1.0;
            if (timeRatio < 0) {
                // Left fade-in: smooth exponential curve from 0 to 1
                const leftProgress = Math.abs(timeRatio) / 0.15; // 0 to 1 as we move from -0.15 to 0
                alpha = Math.max(0, 1 - Math.pow(leftProgress, 0.8)); // Smoother curve
            } else if (timeRatio > 1) {
                // Right fade-out: smooth exponential curve from 1 to 0 with enhanced decay
                const rightProgress = (timeRatio - 1) / 0.15; // 0 to 1 as we move from 1 to 1.15
                alpha = Math.max(0, 1 - Math.pow(rightProgress, 0.6)); // Even smoother fade-out
            } else if (timeRatio > 0.85) {
                // Enhanced right edge fade: start fading earlier for ultra-smooth transition
                const earlyFadeProgress = (timeRatio - 0.85) / 0.15; // 0 to 1 as we move from 0.85 to 1
                const earlyFadeFactor = 1 - Math.pow(earlyFadeProgress, 2) * 0.2; // Gentle early fade
                alpha = Math.min(alpha, earlyFadeFactor);
            }
            
            allPoints.push({x: px, y: py, timestamp, timeRatio, alpha});
        }
        
        if (allPoints.length < 2) continue;
        
        // Draw curve with gradient transparency
        const baseColor = FT_COLORS[curveIndex + colorOffset];
        
        // Extract RGB values from hex color
        const r = parseInt(baseColor.substr(1, 2), 16);
        const g = parseInt(baseColor.substr(3, 2), 16);
        const b = parseInt(baseColor.substr(5, 2), 16);
        
        // Draw line segments with enhanced opacity blending for smoother transitions
        for (let i = 0; i < allPoints.length - 1; i++) {
            const currentPoint = allPoints[i];
            const nextPoint = allPoints[i + 1];
            
            // Use weighted average alpha for ultra-smooth transitions, especially at edges
            const avgAlpha = (currentPoint.alpha + nextPoint.alpha) / 2;
            const segmentAlpha = Math.max(currentPoint.alpha, nextPoint.alpha) * 0.7 + avgAlpha * 0.3;
            
            if (segmentAlpha > 0.005) { // Even lower threshold for smoother fade
                ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, ${segmentAlpha})`;
                ctx.lineWidth = 2;
                ctx.lineCap = 'round';
                ctx.lineJoin = 'round';
                ctx.beginPath();
                ctx.moveTo(currentPoint.x, currentPoint.y);
                ctx.lineTo(nextPoint.x, nextPoint.y);
                ctx.stroke();
            }
        }
    }
    
    ctx.restore();
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

// Send FTC command to robot arm
async function sendFTCCommand(command, parameter = null) {
    const statusElement = document.getElementById('ftcCommandStatus');
    const button = document.getElementById(getFTCButtonId(command, parameter));
    
    // Disable button and show loading state
    if (button) {
        button.disabled = true;
        button.style.opacity = '0.6';
    }
    
    statusElement.textContent = `Sending FTC command: ${command}...`;
    statusElement.className = 'text-sm text-blue-600';
    
    try {
        let endpoint;
        let body;
        
        // Determine endpoint and body based on command
        switch(command) {
            case 'FTC_start':
                endpoint = '/api/ftc/start';
                body = JSON.stringify({ command: 'start' });
                break;
            case 'FTC_stop':
                endpoint = '/api/ftc/stop';
                body = JSON.stringify({ command: 'stop' });
                break;
            case 'FTC_setindex':
                endpoint = '/api/ftc/setindex';
                body = JSON.stringify({ command: 'setindex', index: 0 }); // Default index
                break;
            case 'FTC_SetDKAssemFlag':
                endpoint = '/api/ftc/setdkassemflag';
                body = JSON.stringify({ command: 'setdkassemflag', flag: parameter });
                break;
            default:
                throw new Error(`Unknown FTC command: ${command}`);
        }
        
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: body
        });
        
        const result = await response.json();
        
        if (response.ok) {
            const commandText = parameter !== null ? `${command}(${parameter})` : command;
            statusElement.textContent = `FTC command sent successfully: ${commandText}`;
            statusElement.className = 'text-sm text-green-600';
            logCommand('FTC', commandText, 'success');
        } else {
            statusElement.textContent = `Error: ${result.error || 'Unknown error'}`;
            statusElement.className = 'text-sm text-red-600';
            logCommand('FTC', command, 'error');
        }
    } catch (error) {
        console.error('Error sending FTC command:', error);
        statusElement.textContent = `Network error: ${error.message}`;
        statusElement.className = 'text-sm text-red-600';
        logCommand('FTC', command, 'error');
    } finally {
        // Re-enable button
        if (button) {
            button.disabled = false;
            button.style.opacity = '1';
        }
        
        // Clear status after 3 seconds
        setTimeout(() => {
            statusElement.textContent = 'Ready to send FTC commands';
            statusElement.className = 'text-sm text-gray-600';
        }, 3000);
    }
}

// Send Task command to robot arm using appropriate robot commands
async function sendTaskCommand(command) {
    // Map task commands to actual robot commands
    const taskCommands = {
        // 'arm_move2zero': [
        //     'movej2 [0.0,0.0,0.0,0.0,0.0,0.0] 1.5 1.0 0.0 true',
        //     'movej2 [1.1504,-0.4533,1.3090,0.8081,-1.6195,-1.9460] 1.5 1.0 0.0 true'  // Convert degrees to radians
        // ],
        'task_zero2stickP': 'run_program zero2stickP.jspf true',
        'task_stickP2zero': 'run_program stickP2zero.jspf true',
        'task_zero2stickR': 'run_program zero2stickR.jspf true',
        'task_stickR2zero': 'run_program stickR2zero.jspf true',
        'task_zero2gripper': 'run_program zero2jaw.jspf true',
        'task_gripper2zero': 'run_program jaw2zero.jspf true',
        'task_zero2frame': 'run_program zero2holder.jspf true',
        'task_frame2zero': 'run_program holder2zero.jspf true',
        'task_rotate': 'run_program task_rotate.jspf true',
        'task_pushbox': 'run_program task_pushbox.jspf true'
    };
    
    if (command in taskCommands) {
        const robotCommands = taskCommands[command];
        
        if (Array.isArray(robotCommands)) {
            // For arm_move2zero, send multiple commands sequentially
            for (const robotCommand of robotCommands) {
                await sendCommand(robotCommand);
                // Add a small delay between commands
                await new Promise(resolve => setTimeout(resolve, 500));
            }
        } else {
            // For other tasks, send single command
            await sendCommand(robotCommands);
        }
    } else {
        console.error(`Unknown task command: ${command}`);
    }
}

// Toggle FT Sensor function
function toggleFTSensor() {
    const button = document.getElementById('ftSensorToggleBtn');
    const buttonText = document.getElementById('ftSensorBtnText');
    
    settings.ftSensorEnabled = !settings.ftSensorEnabled;
    
    if (settings.ftSensorEnabled) {
        // Enable FT Sensor
        console.log('Enabling FT Sensor...');
        startFTSensorUDP();
        buttonText.textContent = 'Stop FT';
        button.className = 'robot-arm-button bg-cyan-500 hover:bg-cyan-600 text-white font-bold py-3 px-4 rounded-lg transition-all';
        logCommand('FT Sensor', 'Started', 'success');
    } else {
        // Disable FT Sensor
        console.log('Disabling FT Sensor...');
        if (ftSensorSocket) {
            ftSensorSocket.close();
            ftSensorSocket = null;
        }
        buttonText.textContent = 'Start Sensor';
        button.className = 'robot-arm-button bg-gray-500 hover:bg-gray-600 text-white font-bold py-3 px-4 rounded-lg transition-all';
        logCommand('FT Sensor', 'Stopped', 'info');
    }
    
    console.log('FT Sensor enabled:', settings.ftSensorEnabled);
}

// Get button ID for FTC commands
function getFTCButtonId(command, parameter = null) {
    const buttonMap = {
        'FTC_start': 'ftcStartBtn',
        'FTC_stop': 'ftcStopBtn',
        'FTC_setindex': 'ftcSetIndexBtn',
        'FTC_SetDKAssemFlag': parameter === 1 ? 'ftcEnableProgramBtn' : 'ftcStopProgramBtn'
    };
    return buttonMap[command];
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
        statusText.className = 'status-bar-value connected';
    } else {
        statusIndicator.className = 'status-indicator status-disconnected';
        statusText.textContent = 'Disconnected';
        statusText.className = 'status-bar-value disconnected';
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
            // FTC shortcuts
            case '5':
                event.preventDefault();
                sendFTCCommand('FTC_start');
                break;
            case '6':
                event.preventDefault();
                sendFTCCommand('FTC_stop');
                break;
            case '7':
                event.preventDefault();
                openSetIndexDialog();
                break;
            case '8':
                event.preventDefault();
                sendFTCCommand('FTC_SetDKAssemFlag', 1);
                break;
            case '9':
                event.preventDefault();
                sendFTCCommand('FTC_SetDKAssemFlag', 0);
                break;
            case '0':
                event.preventDefault();
                openSetRTDialog();
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
        {id: 'disableBtn', shortcut: 'Ctrl+4'},
        {id: 'ftcStartBtn', shortcut: 'Ctrl+5'},
        {id: 'ftcStopBtn', shortcut: 'Ctrl+6'},
        {id: 'ftcSetIndexBtn', shortcut: 'Ctrl+7'},
        {id: 'ftcEnableProgramBtn', shortcut: 'Ctrl+8'},
        {id: 'ftcStopProgramBtn', shortcut: 'Ctrl+9'},
        {id: 'ftcSetRTBtn', shortcut: 'Ctrl+0'}
    ];
    
    buttons.forEach(button => {
        const element = document.getElementById(button.id);
        if (element) {
            element.title = `Click to execute or press ${button.shortcut}`;
        }
    });
    
    // Initialize move control input fields with default values
    initializeMoveControlInputs();
    
    // Add event listeners for Set Index dialog
    const indexInput = document.getElementById('ftcIndexInput');
    if (indexInput) {
        indexInput.addEventListener('keydown', function(event) {
            if (event.key === 'Enter') {
                event.preventDefault();
                confirmSetIndex();
            } else if (event.key === 'Escape') {
                event.preventDefault();
                closeSetIndexDialog();
            }
        });
    }
    
    // Close modal when clicking outside
    const setIndexModal = document.getElementById('setIndexModal');
    if (setIndexModal) {
        setIndexModal.addEventListener('click', function(event) {
            if (event.target === setIndexModal) {
                closeSetIndexDialog();
            }
        });
    }
    
    // Close Set RT modal when clicking outside
    const setRTModal = document.getElementById('setRTModal');
    if (setRTModal) {
        setRTModal.addEventListener('click', function(event) {
            if (event.target === setRTModal) {
                closeSetRTDialog();
            }
        });
        
        // Add Escape key listener for Set RT modal
        document.addEventListener('keydown', function(event) {
            if (event.key === 'Escape' && setRTModal.style.display === 'flex') {
                closeSetRTDialog();
            }
        });
    }
});

// Set RT Dialog Functions
let rtParametersInitialized = false; // Flag to track if RT parameters have been initialized

function openSetRTDialog() {
    const modal = document.getElementById('setRTModal');
    if (modal) {
        modal.style.display = 'flex';
        // Only initialize with default values on first open or if no saved parameters exist
        if (!rtParametersInitialized) {
            const loaded = loadRTParameters();
            if (!loaded) {
                initializeRTDefaults();
            }
            rtParametersInitialized = true;
        }
    }
}

function closeSetRTDialog() {
    const modal = document.getElementById('setRTModal');
    if (modal) {
        modal.style.display = 'none';
    }
}

// Function to reset RT parameters to default values
function resetRTToDefaults() {
    if (confirm('Are you sure you want to reset all parameters to default values? This will overwrite your current settings.')) {
        initializeRTDefaults();
        saveRTParameters(); // Save the reset values
    }
}

// Function to save current RT parameters to localStorage
function saveRTParameters() {
    try {
        const parameters = {};
        
        // Save single element values
        const singleElementIds = [
            'rtIsProgram', 'rtFtcProgram', 'rtOnlyMonitor', 'rtGraCalcIndex',
            'rtDisEndLimit', 'rtTimeEndLimit', 'rtFtcEndType',
            'rtIfDKStopOnMaxForce1', 'rtIfRobotStopOnMaxForce1',
            'rtIfDKStopOnMaxForce2', 'rtIfRobotStopOnMaxForce2',
            'rtIfDKStopOnTimeDisMon', 'rtIfRobotStopOnTimeDisMon',
            'rtIfNeedInit', 'rtWithGroup', 'rtFtcSetGroup', 'rtIgnoreSensor'
        ];
        
        singleElementIds.forEach(id => {
            const element = document.getElementById(id);
            if (element) {
                parameters[id] = element.value;
            }
        });
        
        // Save 6-dimensional array values
        const array6DPrefixes = [
            'rtFtEnabled', 'rtFtSet', 'rtDeadZone', 'rtQuickSetIndex',
            'rtFtEndLimit', 'rtDisAng6DEndLimit', 'rtB', 'rtM',
            'rtVelLimit', 'rtCorPosLimit', 'rtMaxForce1', 'rtMaxForce2'
        ];
        
        array6DPrefixes.forEach(prefix => {
            const values = [];
            for (let i = 0; i < 6; i++) {
                const element = document.getElementById(`${prefix}${i}`) || 
                              document.getElementById(`${prefix}_${i}`);
                if (element) {
                    values.push(element.value);
                }
            }
            parameters[prefix] = values;
        });
        
        localStorage.setItem('ftcRTParameters', JSON.stringify(parameters));
        console.log('RT parameters saved to localStorage');
    } catch (error) {
        console.error('Error saving RT parameters:', error);
    }
}

// Function to load RT parameters from localStorage
function loadRTParameters() {
    try {
        const saved = localStorage.getItem('ftcRTParameters');
        if (!saved) return false;
        
        const parameters = JSON.parse(saved);
        
        // Load single element values
        Object.entries(parameters).forEach(([key, value]) => {
            if (Array.isArray(value)) {
                // Handle 6-dimensional arrays
                for (let i = 0; i < 6; i++) {
                    const element = document.getElementById(`${key}${i}`) || 
                                  document.getElementById(`${key}_${i}`);
                    if (element && value[i] !== undefined) {
                        element.value = value[i];
                    }
                }
            } else {
                // Handle single values
                const element = document.getElementById(key);
                if (element) {
                    element.value = value;
                }
            }
        });
        
        console.log('RT parameters loaded from localStorage');
        return true;
    } catch (error) {
        console.error('Error loading RT parameters:', error);
        return false;
    }
}

function initializeRTDefaults() {
    // Set default values for the RT parameters form based on user configuration
    const singleElements = {
        'rtIsProgram': 'false',
        'rtFtcProgram': '',
        'rtOnlyMonitor': 'false',
        'rtGraCalcIndex': '5',
        'rtDisEndLimit': '5000',
        'rtTimeEndLimit': '60',
        'rtFtcEndType': '6',
        'rtIfDKStopOnMaxForce1': 'false',
        'rtIfRobotStopOnMaxForce1': 'false',
        'rtIfDKStopOnMaxForce2': 'false',
        'rtIfRobotStopOnMaxForce2': 'false',
        'rtIfDKStopOnTimeDisMon': 'false',
        'rtIfRobotStopOnTimeDisMon': 'false',
        'rtIfNeedInit': 'true',
        'rtWithGroup': 'false',
        'rtFtcSetGroup': '17',
        'rtIgnoreSensor': 'false'
    };
    
    // 6-dimensional arrays with their default values
    const array6DElements = {
        'rtFtEnabled': [true, true, true, true, true, true],
        'rtFtSet': [0, 0, 0, 0, 0, 0],
        'rtDeadZone': [1, 1, 1, 0.1, 0.1, 0.1],
        'rtQuickSetIndex': [0, 0, 0, 0, 0, 0],
        'rtFtEndLimit': [0, 0, 0, 0, 0, 0],
        'rtDisAng6DEndLimit': [0, 0, 0, 0, 0, 0],
        'rtB': [6000, 6000, 6000, 4500, 4500, 4500],
        'rtM': [20, 20, 20, 25, 25, 25],
        'rtVelLimit': [500, 500, 500, 500, 500, 500],
        'rtCorPosLimit': [10, 10, 10, 5, 5, 5],
        'rtMaxForce1': [0, 0, 0, 0, 0, 0],
        'rtMaxForce2': [0, 0, 0, 0, 0, 0]
    };
    
    // Set single element values
    Object.entries(singleElements).forEach(([id, value]) => {
        const element = document.getElementById(id);
        if (element) {
            element.value = value;
        }
    });
    
    // Set 6-dimensional array values
    Object.entries(array6DElements).forEach(([prefix, values]) => {
        for (let i = 0; i < 6; i++) {
            const element = document.getElementById(`${prefix}${i}`) || 
                          document.getElementById(`${prefix}_${i}`);
            if (element) {
                if (element.tagName === 'SELECT' && prefix === 'rtFtEnabled') {
                    element.value = values[i] ? 'true' : 'false';
                } else {
                    element.value = values[i];
                }
            }
        }
    });
}

function parseArrayString(str) {
    try {
        // Remove whitespace and parse as JSON array
        const cleanStr = str.trim();
        if (cleanStr.startsWith('[') && cleanStr.endsWith(']')) {
            return JSON.parse(cleanStr);
        } else {
            // If not in array format, try to split by comma
            return cleanStr.split(',').map(x => parseFloat(x.trim())).filter(x => !isNaN(x));
        }
    } catch (e) {
        console.warn('Failed to parse array string:', str);
        return [0,0,0,0,0,0]; // Default array
    }
}

async function confirmSetRT() {
    try {
        // Helper function to collect 6D array values
        function collect6DArray(prefix) {
            const values = [];
            for (let i = 0; i < 6; i++) {
                const element = document.getElementById(`${prefix}${i}`) || 
                              document.getElementById(`${prefix}_${i}`);
                if (element) {
                    if (element.type === 'checkbox') {
                        values.push(element.checked);
                    } else if (element.tagName === 'SELECT') {
                        if (prefix === 'rtFtEnabled') {
                            values.push(element.value === 'true');
                        } else {
                            values.push(parseFloat(element.value) || 0);
                        }
                    } else {
                        values.push(parseFloat(element.value) || 0);
                    }
                } else {
                    values.push(0);
                }
            }
            return values;
        }

        // Collect all parameters from the form
        const parameters = {
            isProgram: document.getElementById('rtIsProgram').value === 'true',
            ftcProgram: document.getElementById('rtFtcProgram').value.trim() === '' ? null : document.getElementById('rtFtcProgram').value,
            onlyMonitor: document.getElementById('rtOnlyMonitor').value === 'true',
            graCalcIndex: parseInt(document.getElementById('rtGraCalcIndex').value) || 0,
            ftEnabled: collect6DArray('rtFtEnabled'),
            ftSet: collect6DArray('rtFtSet'),
            deadZone: collect6DArray('rtDeadZone'),
            disEndLimit: parseFloat(document.getElementById('rtDisEndLimit').value) || 0.0,
            timeEndLimit: parseFloat(document.getElementById('rtTimeEndLimit').value) || 0.0,
            ftEndLimit: collect6DArray('rtFtEndLimit'),
            disAng6DEndLimit: collect6DArray('rtDisAng6DEndLimit'),
            ftcEndType: parseInt(document.getElementById('rtFtcEndType').value) || 0,
            quickSetIndex: collect6DArray('rtQuickSetIndex'),
            B: collect6DArray('rtB'),
            M: collect6DArray('rtM'),
            velLimit: collect6DArray('rtVelLimit'),
            corPosLimit: collect6DArray('rtCorPosLimit'),
            maxForce1: collect6DArray('rtMaxForce1'),
            ifDKStopOnMaxForce1: document.getElementById('rtIfDKStopOnMaxForce1').value === 'true',
            ifRobotStopOnMaxForce1: document.getElementById('rtIfRobotStopOnMaxForce1').value === 'true',
            maxForce2: collect6DArray('rtMaxForce2'),
            ifDKStopOnMaxForce2: document.getElementById('rtIfDKStopOnMaxForce2').value === 'true',
            ifRobotStopOnMaxForce2: document.getElementById('rtIfRobotStopOnMaxForce2').value === 'true',
            ifDKStopOnTimeDisMon: document.getElementById('rtIfDKStopOnTimeDisMon').value === 'true',
            ifRobotStopOnTimeDisMon: document.getElementById('rtIfRobotStopOnTimeDisMon').value === 'true',
            ifNeedInit: document.getElementById('rtIfNeedInit').value === 'true',
            withGroup: document.getElementById('rtWithGroup').value === 'true',
            ftcSetGroup: document.getElementById('rtFtcSetGroup').value,
            ignoreSensor: document.getElementById('rtIgnoreSensor').value === 'true'
        };
        
        // Save current parameters before closing dialog
        saveRTParameters();
        
        // Close the dialog first
        closeSetRTDialog();
        
        // Send the FTC set RT command with all parameters
        await sendFTCRTCommand(parameters);
        
    } catch (error) {
        console.error('Error setting FTC RT parameters:', error);
        alert(`Error: ${error.message}`);
    }
}

// FTC RT command function
async function sendFTCRTCommand(parameters) {
    const statusElement = document.getElementById('ftcCommandStatus');
    const button = document.getElementById('ftcSetRTBtn');
    
    // Disable button and show loading state
    if (button) {
        button.disabled = true;
        button.style.opacity = '0.6';
    }
    
    statusElement.textContent = 'Sending FTC RT parameters...';
    statusElement.className = 'text-sm text-blue-600';
    
    try {
        const response = await fetch('/api/ftc/setftsetallrt', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ parameters: parameters })
        });
        
        const result = await response.json();
        
        if (response.ok) {
            statusElement.textContent = 'FTC RT parameters set successfully';
            statusElement.className = 'text-sm text-green-600';
            logCommand('FTC', 'Set RT Parameters', 'success');
        } else {
            statusElement.textContent = `Error: ${result.error || 'Unknown error'}`;
            statusElement.className = 'text-sm text-red-600';
            logCommand('FTC', 'Set RT Parameters', 'error');
        }
    } catch (error) {
        console.error('Error sending FTC RT command:', error);
        statusElement.textContent = `Network error: ${error.message}`;
        statusElement.className = 'text-sm text-red-600';
        logCommand('FTC', 'Set RT Parameters', 'error');
    } finally {
        // Re-enable button
        if (button) {
            button.disabled = false;
            button.style.opacity = '1';
        }
        
        // Clear status after 3 seconds
        setTimeout(() => {
            statusElement.textContent = 'Ready to send FTC commands';
            statusElement.className = 'text-sm text-gray-600';
        }, 3000);
    }
}

// Function to initialize move control input fields
function initializeMoveControlInputs() {
    // Initialize joint input fields
    for (let i = 1; i <= 6; i++) {
        const element = document.getElementById(`moveJoint${i}`);
        if (element) {
            element.value = currentJointValues[i - 1].toFixed(1);
        }
    }
    
    // Initialize TCP input fields
    const tcpAxes = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
    tcpAxes.forEach(axis => {
        const elementId = `moveTcp${axis.charAt(0).toUpperCase() + axis.slice(1)}`;
        const element = document.getElementById(elementId);
        if (element) {
            element.value = currentTcpValues[axis].toFixed(1);
        }
    });
}

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

// Move control variables for tracking current values
let currentJointValues = [0, 0, 0, 0, 0, 0];
let currentTcpValues = {x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0};

// Adjustment step sizes
const jointStepSize = 0.5; // degrees  
const positionStepSize = 5.0; // mm
const rotationStepSize = 2.0; // degrees

// Long press functionality variables
let longPressTimers = new Map(); // 存储每个按钮的计时器
let longPressIntervals = new Map(); // 存储每个按钮的重复执行间隔
const LONG_PRESS_DELAY = 500; // 长按触发延迟 (ms)
const REPEAT_INTERVAL = 20; // 重复执行间隔 (ms)

// Function to adjust joint values
function adjustJoint(jointIndex, direction) {
    const elementId = `moveJoint${jointIndex}`;
    const element = document.getElementById(elementId);
    
    if (!element) return;
    
    // Get current value or use 0 if not set
    let currentValue = parseFloat(element.value) || 0;
    
    // Apply adjustment
    currentValue += direction * jointStepSize;
    
    // Update display
    element.value = currentValue.toFixed(1);
    
    // Store the value
    currentJointValues[jointIndex - 1] = currentValue;
    
    // Log the action
    logCommand('Move Control', `Joint ${jointIndex} adjusted to ${currentValue.toFixed(1)}°`);
    
    // Send servoj command to robot
    sendServoJCommand();
}

// Function to adjust TCP values
function adjustTcp(axis, direction) {
    const elementId = `moveTcp${axis.charAt(0).toUpperCase() + axis.slice(1)}`;
    const element = document.getElementById(elementId);
    
    if (!element) return;
    
    // Get current value or use 0 if not set
    let currentValue = parseFloat(element.value) || 0;
    
    // Determine step size based on axis
    let stepSize;
    if (axis === 'x' || axis === 'y' || axis === 'z') {
        stepSize = positionStepSize;
    } else {
        stepSize = rotationStepSize;
    }
    
    // Apply adjustment
    currentValue += direction * stepSize;
    
    // Update display
    element.value = currentValue.toFixed(1);
    
    // Store the value
    currentTcpValues[axis] = currentValue;
    
    // Log the action
    const unit = (axis === 'x' || axis === 'y' || axis === 'z') ? 'mm' : '°';
    logCommand('Move Control', `TCP ${axis.toUpperCase()} adjusted to ${currentValue.toFixed(1)}${unit}`);
    
    // Send servo_tcp command to robot with incremental movement
    sendServoTcpCommandIncremental(axis, direction);
}

// Function to send servoj command to robot
async function sendServoJCommand() {
    try {
        // Convert degrees to radians for the robot
        const jointRadians = currentJointValues.map(deg => deg * Math.PI / 180);
        
        // Create servoj command with parameters: joints, velocity, acceleration, blocking, kp, kd
        const command = `servoj [${jointRadians.join(',')}] 1.0 1.0 False 200 65`;
        
        const response = await fetch('/api/robot_arm/cmd', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });
        
        if (response.ok) {
            const result = await response.json();
            console.log('ServoJ command sent successfully:', result);
        } else {
            console.error('Failed to send ServoJ command:', response.statusText);
            logCommand('Move Control', `❌ Failed to send ServoJ command: ${response.statusText}`, 'error');
        }
    } catch (error) {
        console.error('Error sending ServoJ command:', error);
        logCommand('Move Control', `❌ Error sending ServoJ command: ${error.message}`, 'error');
    }
}

// Function to send servo_tcp command to robot
async function sendServoTcpCommand() {
    try {
        // servo_tcp expects pose offset (relative movement), not absolute position
        // Calculate small incremental movements based on the step size
        const poseOffset = [
            positionStepSize / 1000,  // Convert mm to meters for small incremental movement
            0,  // Only move in one axis at a time for safety
            0,
            0,  // No rotation change for now
            0,
            0
        ];
        
        // For now, let's just do a small movement test
        // TODO: Implement proper incremental movement logic
        
        // Create servo_tcp command with parameters: pose_offset, velocity, acceleration, tool, blocking, kp, kd
        const command = `servo_tcp [${poseOffset.join(',')}] 1.0 1.0 "" False 200 25`;
        
        console.log('Sending servo_tcp command:', command);
        
        const response = await fetch('/api/robot_arm/cmd', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });
        
        if (response.ok) {
            const result = await response.json();
            console.log('ServoTCP command sent successfully:', result);
        } else {
            console.error('Failed to send ServoTCP command:', response.statusText);
            logCommand('Move Control', `❌ Failed to send ServoTCP command: ${response.statusText}`, 'error');
        }
    } catch (error) {
        console.error('Error sending ServoTCP command:', error);
        logCommand('Move Control', `❌ Error sending ServoTCP command: ${error.message}`, 'error');
    }
}

// Function to send incremental servo_tcp command to robot
async function sendServoTcpCommandIncremental(axis, direction) {
    try {
        // Create pose offset for incremental movement
        // servo_tcp expects relative movement, not absolute position
        let poseOffset = [0, 0, 0, 0, 0, 0];
        
        if (axis === 'x' || axis === 'y' || axis === 'z') {
            // Position movement - convert mm to meters
            const axisIndex = axis === 'x' ? 0 : (axis === 'y' ? 1 : 2);
            poseOffset[axisIndex] = direction * positionStepSize / 1000; // Convert mm to meters
        } else {
            // Rotation movement - convert degrees to radians
            const axisIndex = axis === 'rx' ? 3 : (axis === 'ry' ? 4 : 5);
            poseOffset[axisIndex] = direction * rotationStepSize * Math.PI / 180; // Convert degrees to radians
        }
        
        // Create servo_tcp command with parameters: pose_offset, velocity, acceleration, tool, blocking, kp, kd
        const command = `servo_tcp [${poseOffset.join(',')}] 1.0 1.0 "" False 150 35`;
        
        console.log(`Sending incremental servo_tcp command for ${axis} axis:`, command);
        
        const response = await fetch('/api/robot_arm/cmd', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });
        
        if (response.ok) {
            const result = await response.json();
            console.log('Incremental ServoTCP command sent successfully:', result);
        } else {
            console.error('Failed to send incremental ServoTCP command:', response.statusText);
            logCommand('Move Control', `❌ Failed to send incremental ServoTCP command: ${response.statusText}`, 'error');
        }
    } catch (error) {
        console.error('Error sending incremental ServoTCP command:', error);
        logCommand('Move Control', `❌ Error sending incremental ServoTCP command: ${error.message}`, 'error');
    }
}

// Long press functionality for control buttons
function startLongPress(buttonType, jointIndex, axis, direction) {
    const buttonId = buttonType === 'joint' ? `joint_${jointIndex}_${direction}` : `tcp_${axis}_${direction}`;
    
    // Clear any existing timer for this button
    clearLongPress(buttonId);
    
    // Set a timer to start long press after delay
    const timer = setTimeout(() => {
        console.log(`Starting long press for ${buttonId}`);
        
        // Add visual feedback class to the button
        const button = getCurrentButton(buttonType, jointIndex, axis, direction);
        if (button) {
            button.classList.add('long-pressing');
        }
        
        // Start repeating the action
        const interval = setInterval(() => {
            if (buttonType === 'joint') {
                adjustJoint(jointIndex, direction);
            } else {
                adjustTcp(axis, direction);
            }
        }, REPEAT_INTERVAL);
        
        longPressIntervals.set(buttonId, interval);
    }, LONG_PRESS_DELAY);
    
    longPressTimers.set(buttonId, timer);
}

function clearLongPress(buttonId) {
    // Clear the timer
    if (longPressTimers.has(buttonId)) {
        clearTimeout(longPressTimers.get(buttonId));
        longPressTimers.delete(buttonId);
    }
    
    // Clear the interval
    if (longPressIntervals.has(buttonId)) {
        clearInterval(longPressIntervals.get(buttonId));
        longPressIntervals.delete(buttonId);
    }
}

function clearAllLongPress() {
    // Clear all timers and intervals
    longPressTimers.forEach((timer) => clearTimeout(timer));
    longPressIntervals.forEach((interval) => clearInterval(interval));
    longPressTimers.clear();
    longPressIntervals.clear();
    
    // Remove visual feedback from all buttons
    document.querySelectorAll('.control-button.long-pressing').forEach(button => {
        button.classList.remove('long-pressing');
    });
}

// Helper function to get the current button element
function getCurrentButton(buttonType, jointIndex, axis, direction) {
    let selector;
    if (buttonType === 'joint') {
        // Find button by onclick attribute (for joints)
        selector = `button[data-action="adjustJoint"][data-joint="${jointIndex}"][data-direction="${direction}"]`;
        // Fallback to searching by title
        if (!document.querySelector(selector)) {
            const title = direction > 0 ? `Increase Joint ${jointIndex}` : `Decrease Joint ${jointIndex}`;
            selector = `button[title="${title}"]`;
        }
    } else {
        // Find button by onclick attribute (for TCP)
        selector = `button[data-action="adjustTcp"][data-axis="${axis}"][data-direction="${direction}"]`;
        // Fallback to searching by title
        if (!document.querySelector(selector)) {
            const title = direction > 0 ? `Increase ${axis.toUpperCase()}` : `Decrease ${axis.toUpperCase()}`;
            selector = `button[title="${title}"]`;
        }
    }
    
    return document.querySelector(selector);
}

// Enhanced adjust functions that support both single click and long press
function adjustJointWithLongPress(jointIndex, direction, isLongPress = false) {
    // Always execute the adjustment
    adjustJoint(jointIndex, direction);
    
    // If this is not a long press action, it's a single click
    if (!isLongPress) {
        console.log(`Single click adjustment for Joint ${jointIndex}`);
    }
}

function adjustTcpWithLongPress(axis, direction, isLongPress = false) {
    // Always execute the adjustment
    adjustTcp(axis, direction);
    
    // If this is not a long press action, it's a single click
    if (!isLongPress) {
        console.log(`Single click adjustment for TCP ${axis}`);
    }
}

// Event handlers for mouse/touch events
function handleControlButtonMouseDown(event, buttonType, jointIndex, axis, direction) {
    event.preventDefault();
    
    // Add visual feedback class
    event.target.classList.add('long-pressing');
    
    // Execute immediate action (single click behavior)
    if (buttonType === 'joint') {
        adjustJointWithLongPress(jointIndex, direction, false);
    } else {
        adjustTcpWithLongPress(axis, direction, false);
    }
    
    // Start long press timer
    startLongPress(buttonType, jointIndex, axis, direction);
}

function handleControlButtonMouseUp(event) {
    event.preventDefault();
    
    // Remove visual feedback class
    if (event.target) {
        event.target.classList.remove('long-pressing');
    }
    
    // Clear all long press operations
    clearAllLongPress();
}

function handleControlButtonMouseLeave(event) {
    event.preventDefault();
    
    // Remove visual feedback class
    if (event.target) {
        event.target.classList.remove('long-pressing');
    }
    
    // Clear all long press operations when mouse leaves the button
    clearAllLongPress();
}

// Initialize long press functionality when page loads
function initializeLongPressControls() {
    console.log('Initializing long press controls...');
    
    // Add event listeners for joint control buttons
    for (let jointIndex = 1; jointIndex <= 6; jointIndex++) {
        // Decrease buttons
        const decreaseBtn = document.querySelector(`button[onclick="adjustJoint(${jointIndex}, -1)"]`);
        if (decreaseBtn) {
            // Add data attributes for identification
            decreaseBtn.setAttribute('data-action', 'adjustJoint');
            decreaseBtn.setAttribute('data-joint', jointIndex);
            decreaseBtn.setAttribute('data-direction', '-1');
            
            decreaseBtn.addEventListener('mousedown', (e) => handleControlButtonMouseDown(e, 'joint', jointIndex, null, -1));
            decreaseBtn.addEventListener('mouseup', handleControlButtonMouseUp);
            decreaseBtn.addEventListener('mouseleave', handleControlButtonMouseLeave);
            decreaseBtn.addEventListener('touchstart', (e) => handleControlButtonMouseDown(e, 'joint', jointIndex, null, -1));
            decreaseBtn.addEventListener('touchend', handleControlButtonMouseUp);
            decreaseBtn.addEventListener('touchcancel', handleControlButtonMouseUp);
            
            // Remove onclick to prevent double execution
            decreaseBtn.removeAttribute('onclick');
        }
        
        // Increase buttons
        const increaseBtn = document.querySelector(`button[onclick="adjustJoint(${jointIndex}, 1)"]`);
        if (increaseBtn) {
            // Add data attributes for identification
            increaseBtn.setAttribute('data-action', 'adjustJoint');
            increaseBtn.setAttribute('data-joint', jointIndex);
            increaseBtn.setAttribute('data-direction', '1');
            
            increaseBtn.addEventListener('mousedown', (e) => handleControlButtonMouseDown(e, 'joint', jointIndex, null, 1));
            increaseBtn.addEventListener('mouseup', handleControlButtonMouseUp);
            increaseBtn.addEventListener('mouseleave', handleControlButtonMouseLeave);
            increaseBtn.addEventListener('touchstart', (e) => handleControlButtonMouseDown(e, 'joint', jointIndex, null, 1));
            increaseBtn.addEventListener('touchend', handleControlButtonMouseUp);
            increaseBtn.addEventListener('touchcancel', handleControlButtonMouseUp);
            
            // Remove onclick to prevent double execution
            increaseBtn.removeAttribute('onclick');
        }
    }
    
    // Add event listeners for TCP control buttons
    const tcpAxes = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
    tcpAxes.forEach(axis => {
        // Decrease buttons
        const decreaseBtn = document.querySelector(`button[onclick="adjustTcp('${axis}', -1)"]`);
        if (decreaseBtn) {
            // Add data attributes for identification
            decreaseBtn.setAttribute('data-action', 'adjustTcp');
            decreaseBtn.setAttribute('data-axis', axis);
            decreaseBtn.setAttribute('data-direction', '-1');
            
            decreaseBtn.addEventListener('mousedown', (e) => handleControlButtonMouseDown(e, 'tcp', null, axis, -1));
            decreaseBtn.addEventListener('mouseup', handleControlButtonMouseUp);
            decreaseBtn.addEventListener('mouseleave', handleControlButtonMouseLeave);
            decreaseBtn.addEventListener('touchstart', (e) => handleControlButtonMouseDown(e, 'tcp', null, axis, -1));
            decreaseBtn.addEventListener('touchend', handleControlButtonMouseUp);
            decreaseBtn.addEventListener('touchcancel', handleControlButtonMouseUp);
            
            // Remove onclick to prevent double execution
            decreaseBtn.removeAttribute('onclick');
        }
        
        // Increase buttons
        const increaseBtn = document.querySelector(`button[onclick="adjustTcp('${axis}', 1)"]`);
        if (increaseBtn) {
            // Add data attributes for identification
            increaseBtn.setAttribute('data-action', 'adjustTcp');
            increaseBtn.setAttribute('data-axis', axis);
            increaseBtn.setAttribute('data-direction', '1');
            
            increaseBtn.addEventListener('mousedown', (e) => handleControlButtonMouseDown(e, 'tcp', null, axis, 1));
            increaseBtn.addEventListener('mouseup', handleControlButtonMouseUp);
            increaseBtn.addEventListener('mouseleave', handleControlButtonMouseLeave);
            increaseBtn.addEventListener('touchstart', (e) => handleControlButtonMouseDown(e, 'tcp', null, axis, 1));
            increaseBtn.addEventListener('touchend', handleControlButtonMouseUp);
            increaseBtn.addEventListener('touchcancel', handleControlButtonMouseUp);
            
            // Remove onclick to prevent double execution
            increaseBtn.removeAttribute('onclick');
        }
    });
    
    console.log('Long press controls initialized successfully');
}

// Function to handle joint input changes
function updateJointFromInput(jointIndex) {
    const elementId = `moveJoint${jointIndex}`;
    const element = document.getElementById(elementId);
    
    if (!element) return;
    
    let inputValue = parseFloat(element.value) || 0;
    
    // Store the value
    currentJointValues[jointIndex - 1] = inputValue;
    
    // Log the action
    logCommand('Move Control', `Joint ${jointIndex} set to ${inputValue.toFixed(1)}° (direct input)`);
    
    // Send servoj command to robot
    sendServoJCommand();
}

// Function to handle TCP input changes
function updateTcpFromInput(axis) {
    const elementId = `moveTcp${axis.charAt(0).toUpperCase() + axis.slice(1)}`;
    const element = document.getElementById(elementId);
    
    if (!element) return;
    
    let inputValue = parseFloat(element.value) || 0;
    
    // Store the value
    currentTcpValues[axis] = inputValue;
    
    // Log the action
    const unit = (axis === 'x' || axis === 'y' || axis === 'z') ? 'mm' : '°';
    logCommand('Move Control', `TCP ${axis.toUpperCase()} set to ${inputValue.toFixed(1)}${unit} (direct input)`);
    
    // Send servo_tcp command to robot with absolute position
    sendServoTcpCommandAbsolute();
}

// Function to handle keydown events in input fields
function handleInputKeydown(event, type, indexOrAxis) {
    // Allow Enter key to confirm input
    if (event.key === 'Enter') {
        event.target.blur(); // Remove focus to trigger onchange
        return;
    }
    
    // Allow arrow keys for fine adjustments
    if (event.key === 'ArrowUp') {
        event.preventDefault();
        if (type === 'joint') {
            adjustJoint(indexOrAxis, 1);
        } else if (type === 'tcp') {
            adjustTcp(indexOrAxis, 1);
        }
    } else if (event.key === 'ArrowDown') {
        event.preventDefault();
        if (type === 'joint') {
            adjustJoint(indexOrAxis, -1);
        } else if (type === 'tcp') {
            adjustTcp(indexOrAxis, -1);
        }
    }
}

// Function to send servo_tcp command with offset positioning
async function sendServoTcpCommandAbsolute() {
    try {
        // Check if we have current robot state data
        if (!robotStateData || !robotStateData.TCPActualPosition || robotStateData.TCPActualPosition.length < 6) {
            logCommand('Move Control', '❌ Cannot get current TCP position for offset calculation', 'error');
            return;
        }
        
        const currentTcp = robotStateData.TCPActualPosition;
        
        // Calculate offset between input values and current position
        // Position: convert current from meters to mm, calculate offset in mm, then convert to meters
        const currentXmm = currentTcp[0] * 1000; // Current X in mm
        const currentYmm = currentTcp[1] * 1000; // Current Y in mm  
        const currentZmm = currentTcp[2] * 1000; // Current Z in mm
        
        // Special offset calculation logic: for positive values use -(target - current), for negative values use (target - current)
        let offsetX, offsetY, offsetZ;
        if (currentTcpValues.x >= 0) {
            offsetX = -(currentTcpValues.x - currentXmm) / 1000; // For positive values: -(target - current) in meters
        } else {
            offsetX = (currentTcpValues.x - currentXmm) / 1000; // For negative values: (target - current) in meters
        }
        
        if (currentTcpValues.y >= 0) {
            offsetY = -(currentTcpValues.y - currentYmm) / 1000; // For positive values: -(target - current) in meters
        } else {
            offsetY = (currentTcpValues.y - currentYmm) / 1000; // For negative values: (target - current) in meters
        }
        
        if (currentTcpValues.z >= 0) {
            offsetZ = -(currentTcpValues.z - currentZmm) / 1000; // For positive values: -(target - current) in meters
        } else {
            offsetZ = (currentTcpValues.z - currentZmm) / 1000; // For negative values: (target - current) in meters
        }
        
        // Rotation: convert current from radians to degrees, calculate offset in degrees, then convert to radians
        const currentRxDeg = currentTcp[3] * 180 / Math.PI; // Current Rx in degrees
        const currentRyDeg = currentTcp[4] * 180 / Math.PI; // Current Ry in degrees
        const currentRzDeg = currentTcp[5] * 180 / Math.PI; // Current Rz in degrees
        
        // Special offset calculation logic for rotation: for positive values use -(target - current), for negative values use (target - current)
        let offsetRx, offsetRy, offsetRz;
        if (currentTcpValues.rx >= 0) {
            offsetRx = -(currentTcpValues.rx - currentRxDeg) * Math.PI / 180; // For positive values: -(target - current) in radians
        } else {
            offsetRx = (currentTcpValues.rx - currentRxDeg) * Math.PI / 180; // For negative values: (target - current) in radians
        }
        
        if (currentTcpValues.ry >= 0) {
            offsetRy = -(currentTcpValues.ry - currentRyDeg) * Math.PI / 180; // For positive values: -(target - current) in radians
        } else {
            offsetRy = (currentTcpValues.ry - currentRyDeg) * Math.PI / 180; // For negative values: (target - current) in radians
        }
        
        if (currentTcpValues.rz >= 0) {
            offsetRz = -(currentTcpValues.rz - currentRzDeg) * Math.PI / 180; // For positive values: -(target - current) in radians
        } else {
            offsetRz = (currentTcpValues.rz - currentRzDeg) * Math.PI / 180; // For negative values: (target - current) in radians
        }
        
        // Create servo_tcp command with offset parameters
        const command = `servo_tcp [${offsetX},${offsetY},${offsetZ},${offsetRx},${offsetRy},${offsetRz}] 1.0 1.0 "" False 150 35`;
        
        console.log('Current TCP (raw):', currentTcp);
        console.log('Current TCP (converted):', {
            x: currentXmm, y: currentYmm, z: currentZmm,
            rx: currentRxDeg, ry: currentRyDeg, rz: currentRzDeg
        });
        console.log('Target values:', currentTcpValues);
        console.log('Calculated offsets:', {offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz});
        console.log('Sending servo_tcp command:', command);

        const response = await fetch('/api/robot_arm/cmd', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });

        if (!response.ok) {
            throw new Error(`Failed to send servo_tcp command: ${response.statusText}`);
        }

        const result = await response.json();
        logCommand('Move Control', `TCP moved with offsets: X=${offsetX.toFixed(3)}m, Y=${offsetY.toFixed(3)}m, Z=${offsetZ.toFixed(3)}m, Rx=${(offsetRx*180/Math.PI).toFixed(2)}°, Ry=${(offsetRy*180/Math.PI).toFixed(2)}°, Rz=${(offsetRz*180/Math.PI).toFixed(2)}°`);
        
    } catch (error) {
        console.error('Error sending servo_tcp command:', error);
        logCommand('Move Control', `❌ Error sending servo_tcp command: ${error.message}`, 'error');
    }
}

// Movej2 Dialog Functions
function openMovej2Dialog() {
    const modal = document.getElementById('movej2Modal');
    
    // Initialize with current joint positions if available
    if (robotStateData && robotStateData.joint_actual_position) {
        for (let i = 1; i <= 6; i++) {
            const input = document.getElementById(`movej2Joint${i}`);
            const currentAngleRad = robotStateData.joint_actual_position[i - 1];
            const currentAngleDeg = currentAngleRad * 180 / Math.PI;
            input.value = currentAngleDeg.toFixed(1);
        }
    } else {
        // Use current move control values as fallback
        for (let i = 1; i <= 6; i++) {
            const moveInput = document.getElementById(`moveJoint${i}`);
            const targetInput = document.getElementById(`movej2Joint${i}`);
            if (moveInput) {
                targetInput.value = moveInput.value;
            } else {
                targetInput.value = "0";
            }
        }
    }
    
    modal.style.display = 'flex';
    
    // Focus on first input
    document.getElementById('movej2Joint1').focus();
    
    // Add escape key listener
    document.addEventListener('keydown', handleMovej2ModalKeydown);
}

function closeMovej2Dialog() {
    const modal = document.getElementById('movej2Modal');
    modal.style.display = 'none';
    
    // Remove escape key listener
    document.removeEventListener('keydown', handleMovej2ModalKeydown);
}

function handleMovej2ModalKeydown(event) {
    if (event.key === 'Escape') {
        closeMovej2Dialog();
    }
}

async function confirmMovej2() {
    try {
        // Get target joint angles from inputs
        const targetAngles = [];
        for (let i = 1; i <= 6; i++) {
            const input = document.getElementById(`movej2Joint${i}`);
            const angleDeg = parseFloat(input.value);
            if (isNaN(angleDeg)) {
                throw new Error(`Invalid value for Joint ${i}: ${input.value}`);
            }
            // Convert degrees to radians for the command
            const angleRad = angleDeg * Math.PI / 180;
            targetAngles.push(angleRad);
        }
        
        // Create movej2 command with target joint angles
        const jointAnglesStr = targetAngles.map(angle => angle.toFixed(6)).join(',');
        const command = `movej2 [${jointAnglesStr}] 1.0 1.0 1.0 True`;
        
        console.log('Movej2 target joint angles (degrees):', targetAngles.map(rad => (rad * 180 / Math.PI).toFixed(1)));
        console.log('Sending movej2 command:', command);

        const response = await fetch('/api/robot_arm/cmd', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });

        if (!response.ok) {
            throw new Error(`Failed to send movej2 command: ${response.statusText}`);
        }

        const result = await response.json();
        
        // Log the successful command
        const targetAnglesDeg = targetAngles.map(rad => (rad * 180 / Math.PI).toFixed(1));
        logCommand('Movej2', `Moving to position: J1=${targetAnglesDeg[0]}°, J2=${targetAnglesDeg[1]}°, J3=${targetAnglesDeg[2]}°, J4=${targetAnglesDeg[3]}°, J5=${targetAnglesDeg[4]}°, J6=${targetAnglesDeg[5]}°`);
        
        // Close the dialog
        closeMovej2Dialog();
        
    } catch (error) {
        console.error('Error sending movej2 command:', error);
        logCommand('Movej2', `❌ Error: ${error.message}`, 'error');
        
        // Show error to user but don't close dialog
        alert(`Error: ${error.message}`);
    }
}

// Move TCP Dialog Functions
function openMoveTcpDialog() {
    const modal = document.getElementById('moveTcpModal');
    
    // Initialize with zero offset values
    const offsetIds = ['moveTcpOffsetX', 'moveTcpOffsetY', 'moveTcpOffsetZ', 'moveTcpOffsetRx', 'moveTcpOffsetRy', 'moveTcpOffsetRz'];
    offsetIds.forEach(id => {
        const input = document.getElementById(id);
        input.value = "0";
    });
    
    modal.style.display = 'flex';
    
    // Focus on first input
    document.getElementById('moveTcpOffsetX').focus();
    
    // Add escape key listener
    document.addEventListener('keydown', handleMoveTcpModalKeydown);
}

function closeMoveTcpDialog() {
    const modal = document.getElementById('moveTcpModal');
    modal.style.display = 'none';
    
    // Remove escape key listener
    document.removeEventListener('keydown', handleMoveTcpModalKeydown);
}

function handleMoveTcpModalKeydown(event) {
    if (event.key === 'Escape') {
        closeMoveTcpDialog();
    }
}

async function confirmMoveTcp() {
    try {
        // Get TCP offset values from inputs
        const offsetX = parseFloat(document.getElementById('moveTcpOffsetX').value)/1000;
        const offsetY = parseFloat(document.getElementById('moveTcpOffsetY').value)/1000;
        const offsetZ = parseFloat(document.getElementById('moveTcpOffsetZ').value)/1000;
        const offsetRx = parseFloat(document.getElementById('moveTcpOffsetRx').value);
        const offsetRy = parseFloat(document.getElementById('moveTcpOffsetRy').value);
        const offsetRz = parseFloat(document.getElementById('moveTcpOffsetRz').value);
        
        // Validate input values
        const offsets = [offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz];
        const labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'];
        for (let i = 0; i < offsets.length; i++) {
            if (isNaN(offsets[i])) {
                throw new Error(`Invalid value for ${labels[i]} offset: ${offsets[i]}`);
            }
        }
        
        // Convert rotation offsets from degrees to radians
        const offsetRxRad = offsetRx * Math.PI / 180;
        const offsetRyRad = offsetRy * Math.PI / 180;
        const offsetRzRad = offsetRz * Math.PI / 180;
        
        // Create tcp_move command with offset values
        const poseOffsetStr = [offsetX, offsetY, offsetZ, offsetRxRad, offsetRyRad, offsetRzRad]
            .map(val => val.toFixed(6)).join(',');
        const command = `tcp_move [${poseOffsetStr}] 0.3 0.2 0.0 "" True`;
        
        console.log('TCP Move offset values:', {
            position: `X=${offsetX}mm, Y=${offsetY}mm, Z=${offsetZ}mm`,
            rotation: `Rx=${offsetRx}°, Ry=${offsetRy}°, Rz=${offsetRz}°`
        });
        console.log('Sending tcp_move command:', command);

        const response = await fetch('/api/robot_arm/cmd', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });

        if (!response.ok) {
            throw new Error(`Failed to send tcp_move command: ${response.statusText}`);
        }

        const result = await response.json();
        
        // Log the successful command
        logCommand('TCP Move', `Moving TCP by offset: X=${offsetX}mm, Y=${offsetY}mm, Z=${offsetZ}mm, Rx=${offsetRx}°, Ry=${offsetRy}°, Rz=${offsetRz}°`);
        
        // Close the dialog
        closeMoveTcpDialog();
        
    } catch (error) {
        console.error('Error sending tcp_move command:', error);
        logCommand('TCP Move', `❌ Error: ${error.message}`, 'error');
        
        // Show error to user but don't close dialog
        alert(`Error: ${error.message}`);
    }
}

// Add click outside to close modal functionality
document.addEventListener('click', function(event) {
    const movej2Modal = document.getElementById('movej2Modal');
    if (event.target === movej2Modal) {
        closeMovej2Dialog();
    }
    
    const moveTcpModal = document.getElementById('moveTcpModal');
    if (event.target === moveTcpModal) {
        closeMoveTcpDialog();
    }
});

// Set Index Dialog Functions
function openSetIndexDialog() {
    const modal = document.getElementById('setIndexModal');
    if (modal) {
        modal.style.display = 'flex';
        // Focus on the input field
        const input = document.getElementById('ftcIndexInput');
        if (input) {
            input.focus();
            input.select();
        }
    }
}

function closeSetIndexDialog() {
    const modal = document.getElementById('setIndexModal');
    if (modal) {
        modal.style.display = 'none';
    }
}

async function confirmSetIndex() {
    try {
        const indexInput = document.getElementById('ftcIndexInput');
        if (!indexInput) {
            throw new Error('Index input field not found');
        }
        
        const index = parseInt(indexInput.value);
        if (isNaN(index) || index < 0) {
            throw new Error('Please enter a valid non-negative integer for index');
        }
        
        // Close the dialog first
        closeSetIndexDialog();
        
        // Send the FTC set index command with the specified index
        await sendFTCCommandWithIndex('FTC_setindex', index);
        
    } catch (error) {
        console.error('Error setting FTC index:', error);
        alert(`Error: ${error.message}`);
    }
}

// Enhanced FTC command function that accepts index parameter
async function sendFTCCommandWithIndex(command, index) {
    const statusElement = document.getElementById('ftcCommandStatus');
    const button = document.getElementById('ftcSetIndexBtn');
    
    // Disable button and show loading state
    if (button) {
        button.disabled = true;
        button.style.opacity = '0.6';
    }
    
    statusElement.textContent = `Sending FTC command: ${command} with index ${index}...`;
    statusElement.className = 'text-sm text-blue-600';
    
    try {
        const response = await fetch('/api/ftc/setindex', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: 'setindex', index: index })
        });
        
        const result = await response.json();
        
        if (response.ok) {
            const commandText = `${command}(${index})`;
            statusElement.textContent = `FTC command sent successfully: ${commandText}`;
            statusElement.className = 'text-sm text-green-600';
            logCommand('FTC', commandText, 'success');
        } else {
            statusElement.textContent = `Error: ${result.error || 'Unknown error'}`;
            statusElement.className = 'text-sm text-red-600';
            logCommand('FTC', command, 'error');
        }
    } catch (error) {
        console.error('Error sending FTC command:', error);
        statusElement.textContent = `Network error: ${error.message}`;
        statusElement.className = 'text-sm text-red-600';
        logCommand('FTC', command, 'error');
    } finally {
        // Re-enable button
        if (button) {
            button.disabled = false;
            button.style.opacity = '1';
        }
        
        // Clear status after 3 seconds
        setTimeout(() => {
            statusElement.textContent = 'Ready to send FTC commands';
            statusElement.className = 'text-sm text-gray-600';
        }, 3000);
    }
}

// ===== Camera Functions =====

// Initialize camera stream
function initializeCameraStream() {
    console.log('Setting up direct camera stream...');
    
    const cameraImage = document.getElementById('cameraImage');
    const cameraPlaceholder = document.getElementById('cameraPlaceholder');
    
    if (cameraImage && cameraPlaceholder) {
        // Clear any existing src to start fresh
        cameraImage.src = '';
        
        // Handle image load events
        cameraImage.onload = () => {
            console.log('Camera stream connected successfully');
            cameraImage.style.display = 'block';
            cameraPlaceholder.style.display = 'none';
            updateCameraConnectionStatus(true);
        };
        
        // Handle image error events
        cameraImage.onerror = () => {
            console.log('Camera stream connection failed');
            cameraImage.style.display = 'none';
            cameraPlaceholder.style.display = 'block';
            updateCameraConnectionStatus(false);
            
            // Retry after a delay
            setTimeout(() => {
                console.log('Retrying camera stream connection...');
                cameraImage.src = '/video_feed?' + new Date().getTime(); // Add timestamp to force refresh
            }, 3000);
        };
        
        // Start the stream after event handlers are set
        setTimeout(() => {
            console.log('Starting camera stream...');
            cameraImage.src = '/video_feed';
        }, 500);
    }
    
    // Check camera status periodically
    setInterval(() => {
        updateCameraStatus();
    }, 10000); // Check every 10 seconds
}

// Start camera monitoring (legacy function kept for compatibility)
function startCameraMonitoring() {
    console.log('Starting camera monitoring...');
    initializeCameraStream();
}

// Update camera feed (legacy function - now simplified)
async function updateCameraFeed() {
    // This function is now mostly handled by setupCameraStream
    // We keep it for compatibility but it doesn't need to do much
    return;
}

// Update camera status
async function updateCameraStatus() {
    try {
        const response = await fetch('/api/camera/status');
        const status = await response.json();
        
        updateCameraConnectionStatus(status.connected);
        
    } catch (error) {
        console.error('Error fetching camera status:', error);
        updateCameraConnectionStatus(false);
    }
}

// Update camera connection status display
function updateCameraConnectionStatus(connected) {
    const statusBarCamera = document.getElementById('cameraStatusBar');
    
    cameraConnected = connected; // Update global variable
    
    // Update camera status in the status bar
    if (statusBarCamera) {
        if (connected) {
            statusBarCamera.textContent = 'Connected';
            statusBarCamera.className = 'status-bar-value connected';
        } else {
            statusBarCamera.textContent = 'Disconnected';
            statusBarCamera.className = 'status-bar-value disconnected';
        }
    }
    
    if (!connected) {
        showCameraPlaceholder();
    }
}

// Show camera placeholder
function showCameraPlaceholder() {
    const cameraImage = document.getElementById('cameraImage');
    const cameraPlaceholder = document.getElementById('cameraPlaceholder');
    
    if (cameraImage && cameraPlaceholder) {
        cameraImage.style.display = 'none';
        cameraPlaceholder.style.display = 'block';
        
        // Clean up image URL
        if (cameraImage.src && cameraImage.src.startsWith('blob:')) {
            URL.revokeObjectURL(cameraImage.src);
            cameraImage.src = '';
        }
    }
}
