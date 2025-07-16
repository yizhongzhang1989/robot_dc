// Robot Arm Web Control JavaScript

let commandCounter = 0;
let stateUpdateInterval;
let lastStateUpdate = null;
let updateCount = 0;
let updateRate = 0;

// 3D Visualization variables
let scene, camera, renderer, controls;
let baseFrame, tcpFrame;
let isViewer3DInitialized = false;

// Initialize the page
document.addEventListener('DOMContentLoaded', function() {
    // Check if Three.js is loaded
    if (typeof THREE === 'undefined') {
        console.error('Three.js is not loaded');
        logCommand('System', 'Three.js library not loaded', 'error');
        return;
    }
    
    fetchRobotArmInfo();
    updateConnectionStatus(true);
    startStateMonitoring();
    
    // Initialize 3D viewer with a small delay to ensure DOM is ready
    setTimeout(init3DViewer, 100);
    
    logCommand('System', 'Web interface initialized');
});

// Start robot state monitoring
function startStateMonitoring() {
    // Update robot state every 500ms
    stateUpdateInterval = setInterval(fetchRobotState, 500);
}

// Fetch robot state
async function fetchRobotState() {
    try {
        const response = await fetch('/api/robot_arm/state');
        
        if (response.ok) {
            const stateData = await response.json();
            updateRobotStateDisplay(stateData);
            updateStateStatus(true);
            
            // Calculate update rate
            const now = Date.now();
            if (lastStateUpdate) {
                updateCount++;
                if (updateCount % 10 === 0) {
                    updateRate = Math.round(10000 / (now - lastStateUpdate) * 10) / 10;
                    document.getElementById('updateRate').textContent = updateRate;
                }
            }
            lastStateUpdate = now;
            
        } else {
            updateStateStatus(false);
            document.getElementById('stateStatus').textContent = 'No Data';
            document.getElementById('stateStatus').className = 'error-indicator';
        }
    } catch (error) {
        updateStateStatus(false);
        console.error('Error fetching robot state:', error);
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

    // Update 3D visualization
    if (isViewer3DInitialized) {
        update3DVisualization(state);
    }
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
        controls.enableDamping = true;
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
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(2, 2, 2);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        scene.add(directionalLight);
        
        // Grid
        const gridHelper = new THREE.GridHelper(2, 20, 0x444444, 0x444444);
        scene.add(gridHelper);
        
        // Create coordinate frames
        createCoordinateFrames();
        
        // Start animation loop
        animate();
        
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

// Create coordinate frames for base and TCP
function createCoordinateFrames() {
    // Base frame (fixed at origin) - no rotation, standard coordinate system
    baseFrame = createCoordinateFrame(0.3, 'Base');
    baseFrame.position.set(0, 0, 0);
    scene.add(baseFrame);
    
    // TCP frame (will be updated with robot data) - no rotation, standard coordinate system
    tcpFrame = createCoordinateFrame(0.2, 'TCP');
    tcpFrame.position.set(0.5, 0.5, 0.5);
    scene.add(tcpFrame);
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

// Update 3D visualization with robot state
function update3DVisualization(state) {
    if (!tcpFrame) return;
    
    // Update TCP frame position and orientation
    if (state.TCPActualPosition && state.TCPActualPosition.length >= 6) {
        const tcp = state.TCPActualPosition;
        
        // Position: directly use robot coordinates (no transformation)
        tcpFrame.position.set(tcp[0], tcp[1], tcp[2]);
        
        // Rotation: Apply Rz*Ry*Rx transformation
        // TCP data: [X, Y, Z, Rx, Ry, Rz] where rotations are in radians
        tcpFrame.rotation.order = 'ZYX';  // This applies rotations in Z, Y, X order = Rz*Ry*Rx
        tcpFrame.rotation.set(tcp[3], tcp[4], tcp[5]); // Set Rx, Ry, Rz
    }
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    
    if (controls) {
        controls.update();
    }
    
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
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
