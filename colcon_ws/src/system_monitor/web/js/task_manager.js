// Task Manager JavaScript

// Modal functions
function showModal(title, message, type = 'info') {
    const modal = document.getElementById('messageModal');
    const modalTitle = document.getElementById('modalTitle');
    const modalMessage = document.getElementById('modalMessage');
    const modalCloseBtn = document.getElementById('modalCloseBtn');
    
    if (!modal || !modalTitle || !modalMessage || !modalCloseBtn) return;
    
    // Set title and message
    modalTitle.textContent = title;
    modalMessage.textContent = message;
    
    // Set button color based on type
    modalCloseBtn.className = 'px-4 py-2 text-white text-base font-medium rounded-md shadow-sm focus:outline-none focus:ring-2';
    if (type === 'error') {
        modalCloseBtn.classList.add('bg-red-500', 'hover:bg-red-600', 'focus:ring-red-300');
    } else if (type === 'warning') {
        modalCloseBtn.classList.add('bg-yellow-500', 'hover:bg-yellow-600', 'focus:ring-yellow-300');
    } else if (type === 'success') {
        modalCloseBtn.classList.add('bg-green-500', 'hover:bg-green-600', 'focus:ring-green-300');
    } else {
        modalCloseBtn.classList.add('bg-blue-500', 'hover:bg-blue-600', 'focus:ring-blue-300');
    }
    
    // Show modal
    modal.classList.remove('hidden');
    modal.style.display = 'block';
}

function closeModal() {
    const modal = document.getElementById('messageModal');
    if (modal) {
        modal.classList.add('hidden');
        modal.style.display = 'none';
    }
}

// Initialize when page loads
document.addEventListener('DOMContentLoaded', function() {
    console.log('Task Manager Interface loaded');
    
    // Initialize status bar
    initializeStatusBar();
    
    // Initialize robot monitor
    initializeRobotMonitor();
    
    // Initialize courier robot monitor
    initializeCourierRobotMonitor();

    // Immediately fetch robot status multiple times for fast initial display
    fetchRobotStatus();
    setTimeout(() => fetchRobotStatus(), 100);
    setTimeout(() => fetchRobotStatus(), 300);
    setTimeout(() => fetchRobotStatus(), 500);
    
    // Fetch RTDE data for monitor
    fetchRtdeData();
    
    // Fetch courier robot status for monitor
    fetchCourierRobotStatus();
    
    // Check device connections
    checkDeviceConnections();
    
    // Initialize Quick Navigation status
    initializeQuickNavStatus();
    
    // Immediately check service statuses for faster initial display
    updateAllServiceStatuses();
    
    // Setup button click handlers
    setupButtonHandlers();
    
    // Start periodic updates
    startPeriodicUpdates();
});

// Initialize status bar with default values
function initializeStatusBar() {
    console.log('Initializing status bar...');
    
    // Set initial values
    updateStatusValue('statusBarSystemStatus', 'Ready', 'connected');
    updateStatusValue('statusBarActiveTasks', '0');
    updateStatusValue('statusBarPendingTasks', '0');
    updateStatusValue('statusBarCompletedTasks', '0');
    updateStatusValue('statusBarRobotConnection', 'Disconnected', 'disconnected');
    updateStatusValue('statusBarLastUpdate', getCurrentTime());
}

// Update status bar value
function updateStatusValue(elementId, value, className = null) {
    const element = document.getElementById(elementId);
    if (element) {
        element.textContent = value;
        
        // Update class if provided
        if (className) {
            element.className = 'status-bar-value';
            if (className) {
                element.classList.add(className);
            }
        }
    }
}

// Get current time as HH:MM:SS
function getCurrentTime() {
    const now = new Date();
    return now.toLocaleTimeString('en-US', { hour12: false });
}

// Removed duplicate function - merged into the main startPeriodicUpdates function

// Fetch system status from backend
async function fetchSystemStatus() {
    try {
        const response = await fetch('/api/status');
        if (response.ok) {
            const data = await response.json();
            updateSystemStatus(data);
        }
    } catch (error) {
        console.error('Error fetching system status:', error);
    }
}

// Update system status with data from backend
function updateSystemStatus(data) {
    if (data.system_status) {
        updateStatusValue('statusBarSystemStatus', data.system_status, 
            data.system_status === 'Running' ? 'connected' : 'disconnected');
    }
    
    if (data.active_tasks !== undefined) {
        updateStatusValue('statusBarActiveTasks', data.active_tasks.toString());
    }
    
    if (data.pending_tasks !== undefined) {
        updateStatusValue('statusBarPendingTasks', data.pending_tasks.toString());
    }
    
    if (data.completed_tasks !== undefined) {
        updateStatusValue('statusBarCompletedTasks', data.completed_tasks.toString());
    }
    
    if (data.robot_connection) {
        updateStatusValue('statusBarRobotConnection', data.robot_connection,
            data.robot_connection === 'Connected' ? 'connected' : 'disconnected');
    }
}

// Fetch robot status (robot_mode and safety_mode) from backend
async function fetchRobotStatus() {
    try {
        const response = await fetch('/api/robot_status');
        if (response.ok) {
            const data = await response.json();
            if (data.success) {
                updateRobotStatus(data);
            }
        }
    } catch (error) {
        console.error('Error fetching robot status:', error);
    }
}

// Update robot status display
function updateRobotStatus(data) {
    // Update RTDE Connection status
    const rtdeConnectionElement = document.getElementById('statusBarRtdeConnection');
    if (rtdeConnectionElement) {
        if (data.rtde_connected) {
            rtdeConnectionElement.textContent = 'Connected';
            rtdeConnectionElement.className = 'status-bar-value connected';
        } else if (data.robot_mode === null && data.safety_mode === null) {
            rtdeConnectionElement.textContent = 'Connecting...';
            rtdeConnectionElement.className = 'status-bar-value warning';
        } else {
            rtdeConnectionElement.textContent = 'Disconnected';
            rtdeConnectionElement.className = 'status-bar-value disconnected';
        }
    }
    
    // Update Robot Mode (System Status)
    const robotModeElement = document.getElementById('statusBarRobotMode');
    if (robotModeElement) {
        if (!data.rtde_connected && data.robot_mode === null) {
            // Still connecting
            robotModeElement.textContent = 'Connecting...';
            robotModeElement.className = 'status-bar-value warning';
        } else if (!data.rtde_connected) {
            // Connection failed
            robotModeElement.textContent = 'RTDE Failed';
            robotModeElement.className = 'status-bar-value disconnected';
        } else if (data.robot_mode_str) {
            // Convert to title case (only first letter capitalized)
            const titleCaseText = data.robot_mode_str.charAt(0).toUpperCase() + 
                                 data.robot_mode_str.slice(1).toLowerCase();
            robotModeElement.textContent = titleCaseText;
            
            // Color coding based on robot mode
            if (data.robot_mode === 7) {  // RUNNING
                robotModeElement.className = 'status-bar-value connected';
            } else if (data.robot_mode === 9) {  // BACKDRIVE
                robotModeElement.className = 'status-bar-value blue';
            } else if (data.robot_mode === 5 || data.robot_mode === 4) {  // IDLE or POWER_ON
                robotModeElement.className = 'status-bar-value orange';
            } else {
                robotModeElement.className = 'status-bar-value disconnected';
            }
        }
    }
    
    // Update Safety Mode
    const safetyModeElement = document.getElementById('statusBarSafetyMode');
    if (safetyModeElement) {
        if (!data.rtde_connected && data.safety_mode === null) {
            // Still connecting
            safetyModeElement.textContent = 'Connecting...';
            safetyModeElement.className = 'status-bar-value warning';
        } else if (!data.rtde_connected) {
            // Connection failed
            safetyModeElement.textContent = 'RTDE Failed';
            safetyModeElement.className = 'status-bar-value disconnected';
        } else if (data.safety_mode_str) {
            safetyModeElement.textContent = data.safety_mode_str;
            
            // Color coding based on safety mode
            if (data.safety_mode === 1) {  // Normal
                safetyModeElement.className = 'status-bar-value connected';
            } else if (data.safety_mode === 2) {  // Reduced
                safetyModeElement.className = 'status-bar-value';
            } else {
                safetyModeElement.className = 'status-bar-value disconnected';
            }
        }
    }
}

// Setup button click handlers
function setupButtonHandlers() {
    // Fetch web URLs from backend
    fetchWebURLs();
    
    // Setup Demo Operation Panel button handlers
    setupDemoOperationButtons();
    
    // Setup Position Control Panel button handlers
    setupPositionControlButtons();
}

// Fetch web URLs from backend and setup button handlers
async function fetchWebURLs() {
    try {
        const response = await fetch('/api/web_urls');
        const data = await response.json();
        
        if (data.success) {
            // UR15 button - open UR15 Web interface
            const btnUR15 = document.getElementById('btnUR15');
            if (btnUR15) {
                btnUR15.addEventListener('click', function() {
                    window.open(data.ur15_web_url, '_blank');
                });
            }
            
            // AMR button - open AMR Web interface
            const btnAMR = document.getElementById('btnAMR');
            if (btnAMR) {
                btnAMR.addEventListener('click', function() {
                    window.open(data.amr_web_url, '_blank');
                });
            }
            
            // Courier button - open Courier Web interface
            const btnCourier = document.getElementById('btnCourier');
            if (btnCourier) {
                btnCourier.addEventListener('click', function() {
                    window.open(data.courier_web_url, '_blank');
                });
            }
            
            // Robot Status button - open Robot Status Redis
            const btnRobotStatus = document.getElementById('btnRobotStatus');
            if (btnRobotStatus) {
                btnRobotStatus.addEventListener('click', function() {
                    window.open(data.robot_status_url, '_blank');
                });
            }
            
            // Positioning button - open Positioning Service
            const btnPositioning = document.getElementById('btnPositioning');
            if (btnPositioning) {
                btnPositioning.addEventListener('click', function() {
                    window.open(data.positioning_3d_url, '_blank');
                });
            }
            
            // Labeling button - open Labeling Service
            const btnLabeling = document.getElementById('btnLabeling');
            if (btnLabeling) {
                btnLabeling.addEventListener('click', function() {
                    window.open(data.image_labeling_url, '_blank');
                });
            }
            
            // Workflow button - open Workflow Service
            const btnWorkflow = document.getElementById('btnWorkflow');
            if (btnWorkflow) {
                btnWorkflow.addEventListener('click', function() {
                    window.open(data.workflow_config_url, '_blank');
                });
            }
            
            // FFPP button - open FFPP Server
            const btnFFPP = document.getElementById('btnFFPP');
            if (btnFFPP) {
                btnFFPP.addEventListener('click', function() {
                    window.open(data.ffpp_server_url, '_blank');
                });
            }
        }
    } catch (error) {
        console.error('Error fetching web URLs:', error);
        // Fallback to default URLs
        const btnUR15 = document.getElementById('btnUR15');
        if (btnUR15) {
            btnUR15.addEventListener('click', function() {
                window.open('http://msra-yizhong.guest.corp.microsoft.com:8030/', '_blank');
            });
        }
        
        const btnAMR = document.getElementById('btnAMR');
        if (btnAMR) {
            btnAMR.addEventListener('click', function() {
                window.open('http://msra-yizhong.guest.corp.microsoft.com:5000/', '_blank');
            });
        }
        
        const btnCourier = document.getElementById('btnCourier');
        if (btnCourier) {
            btnCourier.addEventListener('click', function() {
                window.open('http://192.168.1.3:8090', '_blank');
            });
        }
    }
}

// Check web services connection status
async function checkWebServicesStatus() {
    try {
        const response = await fetch('/api/web_urls');
        const data = await response.json();
        
        if (data.success) {
            // Check Courier Web
            checkWebService(data.courier_web_url, 'statusBarCourierWeb', 'Courier Web');
            
            // Check AMR Web
            checkWebService(data.amr_web_url, 'statusBarAMRWeb', 'AMR Web');
            
            // Check UR15 Web
            checkWebService(data.ur15_web_url, 'statusBarUR15Web', 'UR15 Web');
            
            // Check Robot Status Service
            checkWebService(data.robot_status_url, 'statusBarRedisService', 'Redis Service');
            
            // Check Positioning 3D Service
            checkWebService(data.positioning_3d_url, 'statusBarPositioningService', 'Positioning Service');
            
            // Check Camera Calibration Service
            checkWebService(data.camcalib_web_url, 'statusBarCalibrationService', 'Calibration Service');
            
            // Check Image Labeling Service
            checkWebService(data.image_labeling_url, 'statusBarLabelingService', 'Labeling Service');
            
            // Check Workflow Config Service
            checkWebService(data.workflow_config_url, 'statusBarWorkflowConfig', 'Workflow Config');
            
            // Check FFPP Server
            checkWebService(data.ffpp_server_url, 'statusBarFFPPServer', 'FFPP Server');
        }
    } catch (error) {
        console.error('Error fetching web URLs for status check:', error);
    }
}

// Check individual web service
async function checkWebService(url, elementId, serviceName) {
    const element = document.getElementById(elementId);
    if (!element) return;
    
    try {
        // Try to fetch from the service with timeout
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 3000);
        
        const response = await fetch(url, {
            method: 'GET',
            mode: 'no-cors', // Allow cross-origin requests
            signal: controller.signal
        });
        
        clearTimeout(timeoutId);
        
        // For no-cors mode, we can't check response status
        // If fetch doesn't throw, we assume service is reachable
        element.textContent = 'Connected';
        element.className = 'status-bar-value connected';
        
    } catch (error) {
        // Connection failed or timeout
        if (error.name === 'AbortError') {
            element.textContent = 'Timeout';
            element.className = 'status-bar-value disconnected';
        } else {
            element.textContent = 'Disconnected';
            element.className = 'status-bar-value disconnected';
        }
    }
}

// UR15 Robot Monitor Functions
function initializeRobotMonitor() {
    console.log('Initializing UR15 Robot Monitor...');
    
    // Initialize all monitor fields with default values
    const fields = [
        'robotMode', 'safetyMode', 'runtimeState', 'speedScaling',
        'tcpPosX', 'tcpPosY', 'tcpPosZ', 'tcpRotX', 'tcpRotY', 'tcpRotZ',
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
        'mainVoltage', 'robotCurrent', 'toolTemperature', 'tcpForceScalar',
        'digitalInputBits', 'digitalOutputBits', 'analogInput0', 'analogInput1',
        'tcpLinearSpeed', 'tcpAngularSpeed',
        'jointVel1', 'jointVel2', 'jointVel3', 'jointVel4', 'jointVel5', 'jointVel6',
        'jointCur1', 'jointCur2', 'jointCur3', 'jointCur4', 'jointCur5', 'jointCur6',
        'jointTemp1', 'jointTemp2', 'jointTemp3', 'jointTemp4', 'jointTemp5', 'jointTemp6',
        'toolAnalogInput0', 'toolAnalogInput1', 'toolOutputVoltage', 'toolOutputCurrent',
        'payloadMass', 'actualMomentum', 'actualExecutionTime', 'targetExecutionTime',
        'forceFx', 'forceFy', 'forceFz', 'torqueMx', 'torqueMy', 'torqueMz'
    ];
    
    fields.forEach(fieldId => {
        const element = document.getElementById(fieldId);
        if (element) {
            element.textContent = '--';
        }
    });
    
    // Initialize force sensor charts
    initializeForceSensorCharts();
}

// Global chart instances
let forceChart = null;
let torqueChart = null;

// Force sensor data buffers (keep last 50 data points for display)
const maxDataPoints = 50;
const forceDataBuffer = {
    timestamps: [],
    fx: [],
    fy: [],
    fz: []
};
const torqueDataBuffer = {
    timestamps: [],
    mx: [],
    my: [],
    mz: []
};

// Initialize force sensor charts
function initializeForceSensorCharts() {
    console.log('Initializing force sensor charts...');
    
    // Force Chart
    const forceCtx = document.getElementById('forceChart');
    if (forceCtx) {
        forceChart = new Chart(forceCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Fx',
                        data: [],
                        borderColor: 'rgb(59, 130, 246)',
                        backgroundColor: 'rgba(59, 130, 246, 0.1)',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 0
                    },
                    {
                        label: 'Fy',
                        data: [],
                        borderColor: 'rgb(16, 185, 129)',
                        backgroundColor: 'rgba(16, 185, 129, 0.1)',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 0
                    },
                    {
                        label: 'Fz',
                        data: [],
                        borderColor: 'rgb(249, 115, 22)',
                        backgroundColor: 'rgba(249, 115, 22, 0.1)',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 0
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: {
                    duration: 0
                },
                scales: {
                    x: {
                        display: false
                    },
                    y: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Force (N)'
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        position: 'top'
                    }
                }
            }
        });
    }
    
    // Torque Chart
    const torqueCtx = document.getElementById('torqueChart');
    if (torqueCtx) {
        torqueChart = new Chart(torqueCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Mx',
                        data: [],
                        borderColor: 'rgb(139, 92, 246)',
                        backgroundColor: 'rgba(139, 92, 246, 0.1)',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 0
                    },
                    {
                        label: 'My',
                        data: [],
                        borderColor: 'rgb(236, 72, 153)',
                        backgroundColor: 'rgba(236, 72, 153, 0.1)',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 0
                    },
                    {
                        label: 'Mz',
                        data: [],
                        borderColor: 'rgb(234, 179, 8)',
                        backgroundColor: 'rgba(234, 179, 8, 0.1)',
                        borderWidth: 2,
                        tension: 0.3,
                        pointRadius: 0
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: {
                    duration: 0
                },
                scales: {
                    x: {
                        display: false
                    },
                    y: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Torque (Nm)'
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        position: 'top'
                    }
                }
            }
        });
    }
}

// Update force sensor charts with new data
function updateForceSensorCharts(forceData) {
    if (!forceData || !Array.isArray(forceData) || forceData.length < 6) {
        return;
    }
    
    // Get current timestamp
    const now = new Date();
    const timeLabel = now.toLocaleTimeString('en-US', { hour12: false });
    
    // Add new data to buffers
    forceDataBuffer.timestamps.push(timeLabel);
    forceDataBuffer.fx.push(forceData[0]);
    forceDataBuffer.fy.push(forceData[1]);
    forceDataBuffer.fz.push(forceData[2]);
    
    torqueDataBuffer.timestamps.push(timeLabel);
    torqueDataBuffer.mx.push(forceData[3]);
    torqueDataBuffer.my.push(forceData[4]);
    torqueDataBuffer.mz.push(forceData[5]);
    
    // Keep only last maxDataPoints
    if (forceDataBuffer.timestamps.length > maxDataPoints) {
        forceDataBuffer.timestamps.shift();
        forceDataBuffer.fx.shift();
        forceDataBuffer.fy.shift();
        forceDataBuffer.fz.shift();
        
        torqueDataBuffer.timestamps.shift();
        torqueDataBuffer.mx.shift();
        torqueDataBuffer.my.shift();
        torqueDataBuffer.mz.shift();
    }
    
    // Update force chart
    if (forceChart) {
        forceChart.data.labels = forceDataBuffer.timestamps;
        forceChart.data.datasets[0].data = forceDataBuffer.fx;
        forceChart.data.datasets[1].data = forceDataBuffer.fy;
        forceChart.data.datasets[2].data = forceDataBuffer.fz;
        forceChart.update('none');
    }
    
    // Update torque chart
    if (torqueChart) {
        torqueChart.data.labels = torqueDataBuffer.timestamps;
        torqueChart.data.datasets[0].data = torqueDataBuffer.mx;
        torqueChart.data.datasets[1].data = torqueDataBuffer.my;
        torqueChart.data.datasets[2].data = torqueDataBuffer.mz;
        torqueChart.update('none');
    }
}

// Fetch RTDE data from backend
async function fetchRtdeData() {
    try {
        const response = await fetch('/api/rtde_data');
        if (response.ok) {
            const data = await response.json();
            if (data.success && data.rtde_data) {
                updateRobotMonitor(data.rtde_data);
            } else {
                console.log('RTDE data fetch failed or no data available');
                clearRobotMonitorData();
            }
        } else {
            console.error('Failed to fetch RTDE data:', response.status);
            clearRobotMonitorData();
        }
    } catch (error) {
        console.error('Error fetching RTDE data:', error);
        clearRobotMonitorData();
    }
}

// Update robot monitor display
function updateRobotMonitor(rtdeData) {
    try {
        // Robot Status
        if (rtdeData.robot_mode !== undefined) {
            const robotModes = {
                '-1': 'No controller',
                '0': 'Disconnected',
                '1': 'Confirm safety',
                '2': 'Booting',
                '3': 'Power off',
                '4': 'Power on',
                '5': 'Idle',
                '6': 'Backdrive',
                '7': 'Running',
                '8': 'Updating firmware'
            };
            const modeText = robotModes[rtdeData.robot_mode.toString()] || `Unknown(${rtdeData.robot_mode})`;
            updateMonitorField('robotMode', modeText);
        }
        
        if (rtdeData.safety_mode !== undefined) {
            const safetyModes = {
                '1': 'Normal',
                '2': 'Reduced',
                '3': 'Protective Stop',
                '4': 'Recovery',
                '5': 'Safeguard Stop',
                '6': 'System Emergency Stop',
                '7': 'Robot Emergency Stop',
                '8': 'Emergency Stop',
                '9': 'Violation',
                '10': 'Fault',
                '11': 'Validate Stop'
            };
            const safetyText = safetyModes[rtdeData.safety_mode.toString()] || `Unknown(${rtdeData.safety_mode})`;
            updateMonitorField('safetyMode', safetyText);
        }
        
        if (rtdeData.runtime_state !== undefined) {
            const runtimeStates = {
                '0': 'Stopping',
                '1': 'Stopped',
                '2': 'Playing',
                '3': 'Pausing',
                '4': 'Paused',
                '5': 'Resuming'
            };
            const runtimeText = runtimeStates[rtdeData.runtime_state.toString()] || `Unknown(${rtdeData.runtime_state})`;
            updateMonitorField('runtimeState', runtimeText);
        }
        
        if (rtdeData.speed_scaling !== undefined) {
            updateMonitorField('speedScaling', `${(rtdeData.speed_scaling * 100).toFixed(1)}%`);
        }
        
        // TCP Position
        if (rtdeData.actual_TCP_pose && Array.isArray(rtdeData.actual_TCP_pose) && rtdeData.actual_TCP_pose.length >= 6) {
            updateMonitorField('tcpPosX', rtdeData.actual_TCP_pose[0].toFixed(4));
            updateMonitorField('tcpPosY', rtdeData.actual_TCP_pose[1].toFixed(4));
            updateMonitorField('tcpPosZ', rtdeData.actual_TCP_pose[2].toFixed(4));
            updateMonitorField('tcpRotX', rtdeData.actual_TCP_pose[3].toFixed(4));
            updateMonitorField('tcpRotY', rtdeData.actual_TCP_pose[4].toFixed(4));
            updateMonitorField('tcpRotZ', rtdeData.actual_TCP_pose[5].toFixed(4));
        }
        
        // Joint Positions (convert from radians to degrees)
        if (rtdeData.actual_q && Array.isArray(rtdeData.actual_q) && rtdeData.actual_q.length >= 6) {
            const jointIds = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
            for (let i = 0; i < 6; i++) {
                const degrees = (rtdeData.actual_q[i] * 180 / Math.PI).toFixed(1);
                updateMonitorField(jointIds[i], degrees);
            }
        }
        
        // Power and Temperature
        if (rtdeData.actual_main_voltage !== undefined) {
            updateMonitorField('mainVoltage', rtdeData.actual_main_voltage.toFixed(1));
        }
        
        if (rtdeData.actual_robot_current !== undefined) {
            updateMonitorField('robotCurrent', rtdeData.actual_robot_current.toFixed(2));
        }
        
        if (rtdeData.tool_temperature !== undefined) {
            updateMonitorField('toolTemperature', rtdeData.tool_temperature.toFixed(1));
        }
        
        if (rtdeData.tcp_force_scalar !== undefined) {
            updateMonitorField('tcpForceScalar', rtdeData.tcp_force_scalar.toFixed(1));
        }
        
        // I/O Status
        if (rtdeData.actual_digital_input_bits !== undefined) {
            updateMonitorField('digitalInputBits', `0b${rtdeData.actual_digital_input_bits.toString(2).padStart(8, '0')}`);
        }
        
        if (rtdeData.actual_digital_output_bits !== undefined) {
            updateMonitorField('digitalOutputBits', `0b${rtdeData.actual_digital_output_bits.toString(2).padStart(8, '0')}`);
        }
        
        if (rtdeData.standard_analog_input0 !== undefined) {
            updateMonitorField('analogInput0', rtdeData.standard_analog_input0.toFixed(3));
        }
        
        if (rtdeData.standard_analog_input1 !== undefined) {
            updateMonitorField('analogInput1', rtdeData.standard_analog_input1.toFixed(3));
        }
        
        // TCP Speed
        if (rtdeData.actual_TCP_speed && Array.isArray(rtdeData.actual_TCP_speed) && rtdeData.actual_TCP_speed.length >= 6) {
            const linearSpeed = Math.sqrt(
                rtdeData.actual_TCP_speed[0]**2 + 
                rtdeData.actual_TCP_speed[1]**2 + 
                rtdeData.actual_TCP_speed[2]**2
            );
            const angularSpeed = Math.sqrt(
                rtdeData.actual_TCP_speed[3]**2 + 
                rtdeData.actual_TCP_speed[4]**2 + 
                rtdeData.actual_TCP_speed[5]**2
            );
            updateMonitorField('tcpLinearSpeed', linearSpeed.toFixed(4));
            updateMonitorField('tcpAngularSpeed', angularSpeed.toFixed(4));
        }
        
        // Joint Velocities
        if (rtdeData.actual_qd && Array.isArray(rtdeData.actual_qd) && rtdeData.actual_qd.length >= 6) {
            const jointVelIds = ['jointVel1', 'jointVel2', 'jointVel3', 'jointVel4', 'jointVel5', 'jointVel6'];
            for (let i = 0; i < 6; i++) {
                updateMonitorField(jointVelIds[i], rtdeData.actual_qd[i].toFixed(3));
            }
        }
        
        // Joint Currents
        if (rtdeData.actual_current && Array.isArray(rtdeData.actual_current) && rtdeData.actual_current.length >= 6) {
            const jointCurIds = ['jointCur1', 'jointCur2', 'jointCur3', 'jointCur4', 'jointCur5', 'jointCur6'];
            for (let i = 0; i < 6; i++) {
                updateMonitorField(jointCurIds[i], rtdeData.actual_current[i].toFixed(2));
            }
        }
        
        // Joint Temperatures
        if (rtdeData.joint_temperatures && Array.isArray(rtdeData.joint_temperatures) && rtdeData.joint_temperatures.length >= 6) {
            const jointTempIds = ['jointTemp1', 'jointTemp2', 'jointTemp3', 'jointTemp4', 'jointTemp5', 'jointTemp6'];
            for (let i = 0; i < 6; i++) {
                updateMonitorField(jointTempIds[i], rtdeData.joint_temperatures[i].toFixed(1));
            }
        }
        
        // Tool Data
        if (rtdeData.tool_analog_input0 !== undefined) {
            updateMonitorField('toolAnalogInput0', rtdeData.tool_analog_input0.toFixed(3));
        }
        
        if (rtdeData.tool_analog_input1 !== undefined) {
            updateMonitorField('toolAnalogInput1', rtdeData.tool_analog_input1.toFixed(3));
        }
        
        if (rtdeData.tool_output_voltage !== undefined) {
            updateMonitorField('toolOutputVoltage', rtdeData.tool_output_voltage.toFixed(1));
        }
        
        if (rtdeData.tool_output_current !== undefined) {
            updateMonitorField('toolOutputCurrent', rtdeData.tool_output_current.toFixed(1));
        }
        
        // Payload Information
        if (rtdeData.payload !== undefined) {
            updateMonitorField('payloadMass', rtdeData.payload.toFixed(3));
        }
        
        if (rtdeData.actual_momentum !== undefined) {
            updateMonitorField('actualMomentum', rtdeData.actual_momentum.toFixed(4));
        }
        
        // Execution Time
        if (rtdeData.actual_execution_time !== undefined) {
            updateMonitorField('actualExecutionTime', rtdeData.actual_execution_time.toFixed(2));
        }
        
        if (rtdeData.target_execution_time !== undefined) {
            updateMonitorField('targetExecutionTime', rtdeData.target_execution_time.toFixed(2));
        }
        
        // Force Sensor Data (actual_TCP_force)
        if (rtdeData.actual_TCP_force && Array.isArray(rtdeData.actual_TCP_force) && rtdeData.actual_TCP_force.length >= 6) {
            // Update individual force/torque values
            updateMonitorField('forceFx', rtdeData.actual_TCP_force[0].toFixed(2));
            updateMonitorField('forceFy', rtdeData.actual_TCP_force[1].toFixed(2));
            updateMonitorField('forceFz', rtdeData.actual_TCP_force[2].toFixed(2));
            updateMonitorField('torqueMx', rtdeData.actual_TCP_force[3].toFixed(2));
            updateMonitorField('torqueMy', rtdeData.actual_TCP_force[4].toFixed(2));
            updateMonitorField('torqueMz', rtdeData.actual_TCP_force[5].toFixed(2));
            
            // Update charts
            updateForceSensorCharts(rtdeData.actual_TCP_force);
        }
        
    } catch (error) {
        console.error('Error updating robot monitor:', error);
    }
}

// Helper function to update a monitor field
function updateMonitorField(fieldId, value) {
    const element = document.getElementById(fieldId);
    if (element) {
        element.textContent = value;
    }
}

// Clear robot monitor data (show -- for all fields)
function clearRobotMonitorData() {
    const fields = [
        'robotMode', 'safetyMode', 'runtimeState', 'speedScaling',
        'tcpPosX', 'tcpPosY', 'tcpPosZ', 'tcpRotX', 'tcpRotY', 'tcpRotZ',
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
        'mainVoltage', 'robotCurrent', 'toolTemperature', 'tcpForceScalar',
        'digitalInputBits', 'digitalOutputBits', 'analogInput0', 'analogInput1',
        'tcpLinearSpeed', 'tcpAngularSpeed',
        'jointVel1', 'jointVel2', 'jointVel3', 'jointVel4', 'jointVel5', 'jointVel6',
        'jointCur1', 'jointCur2', 'jointCur3', 'jointCur4', 'jointCur5', 'jointCur6',
        'jointTemp1', 'jointTemp2', 'jointTemp3', 'jointTemp4', 'jointTemp5', 'jointTemp6',
        'toolAnalogInput0', 'toolAnalogInput1', 'toolOutputVoltage', 'toolOutputCurrent',
        'payloadMass', 'actualMomentum', 'actualExecutionTime', 'targetExecutionTime',
        'forceFx', 'forceFy', 'forceFz', 'torqueMx', 'torqueMy', 'torqueMz'
    ];
    
    fields.forEach(fieldId => {
        updateMonitorField(fieldId, '--');
    });
}

// Check device connections
async function checkDeviceConnections() {
    try {
        const response = await fetch('/api/device_connections');
        if (response.ok) {
            const data = await response.json();
            if (data.success) {
                updateConnectionStatus('statusBarUR15Connection', data.connections.ur15, 'UR15');
                updateConnectionStatus('statusBarCourierConnection', data.connections.courier, 'Courier');
                updateConnectionStatus('statusBarAMRConnection', data.connections.amr, 'AMR');
            } else {
                console.error('Device connections check failed:', data.message);
                updateConnectionStatus('statusBarUR15Connection', false, 'UR15');
                updateConnectionStatus('statusBarCourierConnection', false, 'Courier');
                updateConnectionStatus('statusBarAMRConnection', false, 'AMR');
            }
        } else {
            console.error('Failed to fetch device connections:', response.status);
        }
    } catch (error) {
        console.error('Error checking device connections:', error);
        // Set all to disconnected on error
        updateConnectionStatus('statusBarUR15Connection', false, 'UR15');
        updateConnectionStatus('statusBarCourierConnection', false, 'Courier');
        updateConnectionStatus('statusBarAMRConnection', false, 'AMR');
    }
}

// Update connection status display
function updateConnectionStatus(elementId, isConnected, deviceName) {
    const element = document.getElementById(elementId);
    if (element) {
        if (isConnected) {
            element.textContent = 'Connected';
            element.className = 'status-bar-value connected';
        } else {
            element.textContent = 'Disconnected';
            element.className = 'status-bar-value disconnected';
        }
    }
    
    // Update Quick Navigation status indicators
    updateQuickNavStatus(deviceName, isConnected);
}

// Update Quick Navigation status indicators (only for device connections)
function updateQuickNavStatus(deviceName, isConnected) {
    // This function is no longer used for Quick Navigation
    // Quick Navigation now uses web service status checks
}

// Initialize Quick Navigation status indicators
function initializeQuickNavStatus() {
    // Set all to checking status initially
    const services = ['UR15', 'AMR', 'Courier', 'Redis', 'Positioning', 'Labeling', 'Workflow', 'FFPP'];
    
    services.forEach(service => {
        const indicatorId = 'statusIndicator' + service;
        const textId = 'statusText' + service;
        
        const indicator = document.getElementById(indicatorId);
        const text = document.getElementById(textId);
        
        if (indicator && text) {
            indicator.className = 'w-3 h-3 rounded-full bg-yellow-500';
            text.textContent = 'Checking';
            text.className = 'text-sm text-yellow-600 font-medium';
        }
    });
}

// Update Quick Navigation service status
function updateQuickNavServiceStatus(serviceName, isOnline) {
    const indicatorId = 'statusIndicator' + serviceName;
    const textId = 'statusText' + serviceName;
    
    const indicator = document.getElementById(indicatorId);
    const text = document.getElementById(textId);
    
    if (indicator && text) {
        if (isOnline) {
            indicator.className = 'w-3 h-3 rounded-full bg-green-500';
            text.textContent = 'Online';
            text.className = 'text-sm text-green-600 font-medium';
        } else {
            indicator.className = 'w-3 h-3 rounded-full bg-red-500';
            text.textContent = 'Offline';
            text.className = 'text-sm text-red-600 font-medium';
        }
    }
}

// Start periodic updates
function startPeriodicUpdates() {
    console.log('Starting periodic updates...');
    
    // Fetch robot status every 100ms for fast updates
    setInterval(() => {
        fetchRobotStatus();
    }, 100);
    
    // Fetch RTDE data every 100ms for robot monitor (10 Hz)
    setInterval(() => {
        fetchRtdeData();
    }, 100);
    
    // Fetch courier robot status every 200ms for courier robot monitor
    setInterval(() => {
        fetchCourierRobotStatus();
    }, 200);
    
    // Update device connections every 5 seconds
    setInterval(() => {
        checkDeviceConnections();
    }, 5000);
    
    // Update web services status every 5 seconds
    setInterval(() => {
        updateAllServiceStatuses();
    }, 5000);
    
    // Update system status every 3 seconds
    setInterval(() => {
        fetchSystemStatus();
    }, 3000);
}

// Update all service statuses for Quick Navigation
async function updateAllServiceStatuses() {
    try {
        // Get the web URLs to test service availability
        const response = await fetch('/api/web_urls');
        if (response.ok) {
            const data = await response.json();
            
            // Test all services in parallel for faster response
            const serviceChecks = [
                testServiceStatus('UR15', data.ur15_web_url),
                testServiceStatus('AMR', data.amr_web_url),
                testServiceStatus('Courier', data.courier_web_url),
                testServiceStatus('Redis', data.robot_status_url),
                testServiceStatus('Positioning', data.positioning_3d_url),
                testServiceStatus('Labeling', data.image_labeling_url),
                testServiceStatus('Workflow', data.workflow_config_url),
                testServiceStatus('FFPP', data.ffpp_server_url)
            ];
            
            // Wait for all checks to complete (or timeout)
            await Promise.allSettled(serviceChecks);
        }
    } catch (error) {
        console.error('Error checking service statuses:', error);
    }
}

// Test individual service status
async function testServiceStatus(serviceName, serviceUrl) {
    try {
        if (!serviceUrl) {
            updateQuickNavServiceStatus(serviceName, false);
            return;
        }
        
        // Simple fetch to test if service is responding
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 1000); // 1 second timeout
        
        const response = await fetch(serviceUrl, {
            method: 'GET', // Changed from HEAD to GET for better compatibility
            signal: controller.signal,
            mode: 'no-cors' // Allow cross-origin requests
        });
        
        clearTimeout(timeoutId);
        updateQuickNavServiceStatus(serviceName, true);
    } catch (error) {
        console.warn(`Service ${serviceName} (${serviceUrl}) is not available:`, error.name);
        updateQuickNavServiceStatus(serviceName, false);
    }
}

// Courier Robot Monitor Functions
function initializeCourierRobotMonitor() {
    
    // Initialize all courier monitor fields with default values
    const courierFields = [
        'platformTaskState', 'platformMovementState', 'platformControlMode', 'platformForceLimitStatus',
        'pushrodMovementState',
        'courierHeight', 'courierLeftForce', 'courierRightForce', 'courierCombinedForce',
        'courierServerId', 'courierFrequency', 'courierMaxForceLimit'
    ];
    
    courierFields.forEach(fieldId => {
        const element = document.getElementById(fieldId);
        if (element) {
            element.textContent = '--';
        }
    });
}

// Fetch courier robot status from the courier robot web API
async function fetchCourierRobotStatus() {
    try {
        // Use the courier robot web API endpoint
        const response = await fetch('/api/courier_robot_status');
        if (response.ok) {
            const data = await response.json();
            if (data.success) {
                updateCourierRobotMonitor(data);
            } else {
                console.log('Courier robot status fetch failed or no data available');
                clearCourierRobotMonitorData();
            }
        } else {
            console.error('Failed to fetch courier robot status:', response.status);
            clearCourierRobotMonitorData();
        }
    } catch (error) {
        console.error('Error fetching courier robot status:', error);
        clearCourierRobotMonitorData();
    }
}

// Update courier robot monitor display
function updateCourierRobotMonitor(data) {
    try {
        // Update Platform Status
        if (data.platform) {
            // Handle platform task state with color coding
            // Note: task_state is now at top level (not in platform object)
            const platformTaskElement = document.getElementById('platformTaskState');
            if (platformTaskElement && data.task_state) {
                platformTaskElement.textContent = data.task_state;
                
                // Color coding based on task state
                if (data.task_state === 'completed') {
                    platformTaskElement.className = 'font-mono text-green-600 font-bold';
                } else if (data.task_state === 'running') {
                    platformTaskElement.className = 'font-mono text-red-600 font-bold';
                } else {
                    platformTaskElement.className = 'font-mono text-blue-600 font-bold';
                }
            } else if (platformTaskElement) {
                platformTaskElement.textContent = '--';
                platformTaskElement.className = 'font-mono text-blue-600 font-bold';
            }
            
            // Handle platform movement state with color coding
            const platformMovementElement = document.getElementById('platformMovementState');
            if (platformMovementElement && data.platform.movement_state) {
                platformMovementElement.textContent = data.platform.movement_state;
                
                // Color coding based on movement state (same as pushrod)
                if (data.platform.movement_state === 'stop') {
                    platformMovementElement.className = 'font-mono text-red-600 font-bold';
                } else if (data.platform.movement_state === 'up') {
                    platformMovementElement.className = 'font-mono text-orange-600 font-bold';
                } else if (data.platform.movement_state === 'down') {
                    platformMovementElement.className = 'font-mono text-green-600 font-bold';
                } else {
                    platformMovementElement.className = 'font-mono text-green-600 font-bold';
                }
            } else if (platformMovementElement) {
                platformMovementElement.textContent = '--';
                platformMovementElement.className = 'font-mono text-green-600 font-bold';
            }
            
            updateCourierMonitorField('platformControlMode', data.platform.control_mode);
            
            // Handle force limit status with color coding
            const forceLimitElement = document.getElementById('platformForceLimitStatus');
            if (forceLimitElement && data.platform.force_limit_status) {
                forceLimitElement.textContent = data.platform.force_limit_status;
                
                // Color coding based on force limit status
                if (data.platform.force_limit_status === 'ok') {
                    forceLimitElement.className = 'font-mono text-green-600 font-bold';
                } else if (data.platform.force_limit_status === 'exceeded') {
                    forceLimitElement.className = 'font-mono text-red-600 font-bold';
                } else if (data.platform.force_limit_status === 'disabled') {
                    forceLimitElement.className = 'font-mono text-orange-600 font-bold';
                } else {
                    forceLimitElement.className = 'font-mono text-purple-600 font-bold';
                }
            }
            
            updateCourierMonitorField('courierMaxForceLimit', 
                data.platform.max_force_limit !== undefined ? 
                data.platform.max_force_limit.toFixed(1) : '--');
        }
        
        // Update Pushrod Status
        if (data.pushrod) {
            
            // Handle pushrod movement state with color coding
            const pushrodMovementElement = document.getElementById('pushrodMovementState');
            if (pushrodMovementElement && data.pushrod.movement_state) {
                pushrodMovementElement.textContent = data.pushrod.movement_state;
                
                // Color coding based on movement state
                if (data.pushrod.movement_state === 'stop') {
                    pushrodMovementElement.className = 'font-mono text-red-600 font-bold';
                } else if (data.pushrod.movement_state === 'up') {
                    pushrodMovementElement.className = 'font-mono text-orange-600 font-bold';
                } else if (data.pushrod.movement_state === 'down') {
                    pushrodMovementElement.className = 'font-mono text-green-600 font-bold';
                } else {
                    pushrodMovementElement.className = 'font-mono text-green-600 font-bold';
                }
            } else if (pushrodMovementElement) {
                pushrodMovementElement.textContent = '--';
                pushrodMovementElement.className = 'font-mono text-green-600 font-bold';
            }
        }
        
        // Update Sensor Data
        if (data.sensors) {
            updateCourierMonitorField('courierHeight', 
                data.sensors.height !== undefined && data.sensors.height !== null ? 
                data.sensors.height.toFixed(1) : '--');
            updateCourierMonitorField('courierLeftForce', 
                data.sensors.left_force !== undefined && data.sensors.left_force !== null ? 
                data.sensors.left_force.toFixed(2) : '--');
            updateCourierMonitorField('courierRightForce', 
                data.sensors.right_force !== undefined && data.sensors.right_force !== null ? 
                data.sensors.right_force.toFixed(2) : '--');
            updateCourierMonitorField('courierCombinedForce', 
                data.sensors.combined_force !== undefined && data.sensors.combined_force !== null ? 
                data.sensors.combined_force.toFixed(2) : '--');
            updateCourierMonitorField('courierFrequency', 
                data.sensors.freq_hz !== undefined && data.sensors.freq_hz !== null ? 
                data.sensors.freq_hz.toFixed(1) : '--');
        }
        
        // Update System Information
        updateCourierMonitorField('courierServerId', 
            data.server_id !== undefined ? data.server_id : '--');
        
    } catch (error) {
        console.error('Error updating courier robot monitor:', error);
        clearCourierRobotMonitorData();
    }
}

// Clear courier robot monitor data when connection fails
function clearCourierRobotMonitorData() {
    const courierFields = [
        'platformTaskState', 'platformMovementState', 'platformControlMode', 'platformForceLimitStatus',
        'pushrodMovementState',
        'courierHeight', 'courierLeftForce', 'courierRightForce', 'courierCombinedForce',
        'courierServerId', 'courierFrequency', 'courierMaxForceLimit'
    ];
    
    courierFields.forEach(fieldId => {
        updateCourierMonitorField(fieldId, '--');
    });
}

// Helper function to update courier monitor fields
function updateCourierMonitorField(fieldId, value) {
    const element = document.getElementById(fieldId);
    if (element) {
        element.textContent = value || '--';
    }
}

// ============================= Demo Operation Panel Functions =============================

// Setup Demo Operation Panel button handlers
// Global variable to store server index
let currentServerIndex = null;

function setupDemoOperationButtons() {
    console.log('Setting up Demo Operation button handlers...');
    
    // Setup Operation Unit input and button
    const btnSetOperationUnit = document.getElementById('btnSetOperationUnit');
    if (btnSetOperationUnit) {
        btnSetOperationUnit.addEventListener('click', setOperationUnit);
    }
    
    // Setup individual step buttons and checkboxes (1-27)
    for (let i = 1; i <= 27; i++) {
        const btnId = `btnStep${i}`;
        const chkId = `chkStep${i}`;
        const btn = document.getElementById(btnId);
        const chk = document.getElementById(chkId);
        
        if (btn) {
            btn.addEventListener('click', () => executeStep(i));
            // Initially disable all step buttons
            btn.disabled = true;
            btn.classList.add('opacity-50', 'cursor-not-allowed');
            btn.classList.remove('hover:bg-blue-600');
        }
        
        if (chk) {
            // Initially disable all checkboxes
            chk.disabled = true;
        }
    }
    
    // Setup Select All button
    const btnSelectAll = document.getElementById('btnSelectAll');
    if (btnSelectAll) {
        btnSelectAll.addEventListener('click', selectAllSteps);
        // Initially disable
        btnSelectAll.disabled = true;
        btnSelectAll.classList.add('opacity-50', 'cursor-not-allowed');
        btnSelectAll.classList.remove('hover:bg-blue-600');
    }
    
    // Setup Execute Selected button
    const btnExecuteSelected = document.getElementById('btnExecuteSelected');
    if (btnExecuteSelected) {
        btnExecuteSelected.addEventListener('click', executeSelectedSteps);
        // Initially disable
        btnExecuteSelected.disabled = true;
        btnExecuteSelected.classList.add('opacity-50', 'cursor-not-allowed');
        btnExecuteSelected.classList.remove('hover:bg-orange-600');
    }
    
    // Setup Execute All button
    const btnExecuteAll = document.getElementById('btnExecuteAll');
    if (btnExecuteAll) {
        btnExecuteAll.addEventListener('click', executeAllSteps);
        // Initially disable Execute All button
        btnExecuteAll.disabled = true;
        btnExecuteAll.classList.add('opacity-50', 'cursor-not-allowed');
        btnExecuteAll.classList.remove('hover:bg-green-600');
    }
}

// Setup Position Control Panel button handlers
function setupPositionControlButtons() {
    console.log('Setting up Position Control button handlers...');
    
    // Setup Task Position button
    const btnTaskPosition = document.getElementById('btnTaskPosition');
    if (btnTaskPosition) {
        btnTaskPosition.addEventListener('click', () => moveToPosition('task'));
    }
    
    // Setup Home Position button
    const btnHomePosition = document.getElementById('btnHomePosition');
    if (btnHomePosition) {
        btnHomePosition.addEventListener('click', () => moveToPosition('home'));
    }
}

// Move robot to specified position
async function moveToPosition(positionType) {
    console.log(`Moving to ${positionType} position...`);
    
    // Disable button to prevent multiple clicks
    const btnId = positionType === 'task' ? 'btnTaskPosition' : 'btnHomePosition';
    const btn = document.getElementById(btnId);
    const originalText = btn ? btn.textContent : '';
    
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
        btn.textContent = positionType === 'task' ? '⏳ Moving to Task...' : '⏳ Moving to Home...';
    }
    
    try {
        const response = await fetch('/api/move_to_position', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                position_type: positionType
            })
        });
        
        const data = await response.json();
        
        if (data.success) {
            showModal('Success', `Successfully moved to ${positionType} position`, 'success');
        } else {
            showModal('Error', data.message || `Failed to move to ${positionType} position`, 'error');
        }
    } catch (error) {
        console.error(`Error moving to ${positionType} position:`, error);
        showModal('Error', `Failed to move to ${positionType} position: ${error.message}`, 'error');
    } finally {
        // Re-enable button
        if (btn) {
            btn.disabled = false;
            btn.classList.remove('opacity-50', 'cursor-not-allowed');
            btn.textContent = originalText;
        }
    }
}

// Set Operation Unit (Server Index)
function setOperationUnit() {
    const input = document.getElementById('operationUnitInput');
    const statusDiv = document.getElementById('operationUnitStatus');
    
    if (!input || !statusDiv) return;
    
    const value = parseInt(input.value);
    
    if (isNaN(value) || value < 8 || value > 25) {
        showModal('Invalid Input', 'Please enter a valid server index (8-25)', 'warning');
        return;
    }
    
    // Set the global server index
    currentServerIndex = value;
    
    // Update status display
    statusDiv.innerHTML = `
        <span class="inline-block w-3 h-3 rounded-full bg-green-500"></span>
        <span class="text-sm font-medium text-green-600">Unit ${value}</span>
    `;
    
    // Enable all step buttons and checkboxes
    for (let i = 1; i <= 27; i++) {
        const btn = document.getElementById(`btnStep${i}`);
        if (btn) {
            btn.disabled = false;
            btn.classList.remove('opacity-50', 'cursor-not-allowed');
            btn.classList.add('hover:bg-blue-600');
        }
        
        const chk = document.getElementById(`chkStep${i}`);
        if (chk) {
            chk.disabled = false;
        }
    }
    
    // Enable Select All button
    const btnSelectAll = document.getElementById('btnSelectAll');
    if (btnSelectAll) {
        btnSelectAll.disabled = false;
        btnSelectAll.classList.remove('opacity-50', 'cursor-not-allowed');
        btnSelectAll.classList.add('hover:bg-blue-600');
    }
    
    // Enable Execute Selected button
    const btnExecuteSelected = document.getElementById('btnExecuteSelected');
    if (btnExecuteSelected) {
        btnExecuteSelected.disabled = false;
        btnExecuteSelected.classList.remove('opacity-50', 'cursor-not-allowed');
        btnExecuteSelected.classList.add('hover:bg-orange-600');
    }
    
    // Enable Execute All button
    const btnExecuteAll = document.getElementById('btnExecuteAll');
    if (btnExecuteAll) {
        btnExecuteAll.disabled = false;
        btnExecuteAll.classList.remove('opacity-50', 'cursor-not-allowed');
        btnExecuteAll.classList.add('hover:bg-green-600');
    }
    
    console.log(`Operation Unit set to: ${value}`);
}

// Select/Deselect all steps
function selectAllSteps() {
    const btnSelectAll = document.getElementById('btnSelectAll');
    if (!btnSelectAll) return;
    
    // Check if any checkbox is unchecked
    let anyUnchecked = false;
    for (let i = 1; i <= 27; i++) {
        const chk = document.getElementById(`chkStep${i}`);
        if (chk && !chk.checked) {
            anyUnchecked = true;
            break;
        }
    }
    
    // If any is unchecked, check all; otherwise uncheck all
    const newState = anyUnchecked;
    for (let i = 1; i <= 27; i++) {
        const chk = document.getElementById(`chkStep${i}`);
        if (chk) {
            chk.checked = newState;
        }
    }
    
    // Update button text
    btnSelectAll.innerHTML = newState ? '☐ Deselect All Steps' : '☑️ Select All Steps';
}

// Execute selected steps
async function executeSelectedSteps() {
    // Check if operation unit is set
    if (currentServerIndex === null) {
        showModal('Operation Unit Required', 'Please set Operation Unit (Server Index) first!', 'warning');
        return;
    }
    
    // Get list of selected steps
    const selectedSteps = [];
    for (let i = 1; i <= 27; i++) {
        const chk = document.getElementById(`chkStep${i}`);
        if (chk && chk.checked) {
            selectedSteps.push(i);
        }
    }
    
    if (selectedSteps.length === 0) {
        showModal('No Steps Selected', 'Please select at least one step to execute!', 'warning');
        return;
    }
    
    const btnExecuteSelected = document.getElementById('btnExecuteSelected');
    if (!btnExecuteSelected) return;
    
    // Disable the button
    btnExecuteSelected.disabled = true;
    const originalText = btnExecuteSelected.innerHTML;
    btnExecuteSelected.innerHTML = '⏳ Executing...';
    
    try {
        // Execute each selected step sequentially
        for (const stepNumber of selectedSteps) {
            console.log(`Executing selected step ${stepNumber}...`);
            await executeStep(stepNumber);
            
            // Check if the step failed
            const statusText = document.getElementById(`statusText${stepNumber}`);
            if (statusText && statusText.textContent === 'Failed') {
                console.error(`Step ${stepNumber} failed. Stopping execution.`);
                showModal('Execution Failed', `Execution stopped at Step ${stepNumber} due to failure.`, 'error');
                break;
            }
            
            // Small delay between steps
            await new Promise(resolve => setTimeout(resolve, 500));
        }
        console.log('All selected steps completed');
    } catch (error) {
        console.error('Error executing selected steps:', error);
        showModal('Execution Error', `Error executing selected steps: ${error.message}`, 'error');
    } finally {
        // Re-enable the button
        btnExecuteSelected.disabled = false;
        btnExecuteSelected.innerHTML = originalText;
    }
}

// Execute a single step
async function executeStep(stepNumber) {
    // Check if operation unit is set
    if (currentServerIndex === null) {
        showModal('Operation Unit Required', 'Please set Operation Unit (Server Index) first!', 'warning');
        return;
    }
    
    const btnId = `btnStep${stepNumber}`;
    const statusDotId = `statusDot${stepNumber}`;
    const statusTextId = `statusText${stepNumber}`;
    const btn = document.getElementById(btnId);
    const statusDot = document.getElementById(statusDotId);
    const statusText = document.getElementById(statusTextId);
    
    if (!btn || !statusDot || !statusText) {
        console.error(`Elements not found for step ${stepNumber}`);
        return;
    }
    
    // Disable button and show loading state
    btn.disabled = true;
    statusDot.className = 'inline-block w-3 h-3 rounded-full bg-yellow-500';
    statusText.textContent = 'Executing';
    statusText.className = 'text-xs font-medium text-yellow-600';
    
    try {
        console.log(`Executing Step ${stepNumber} with server_index=${currentServerIndex}...`);
        
        // Call backend API to execute the step
        const response = await fetch('/api/demo/execute_step', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                step: stepNumber,
                server_index: currentServerIndex
            })
        });
        
        const data = await response.json();
        
        if (data.success) {
            // Success - show green status
            statusDot.className = 'inline-block w-3 h-3 rounded-full bg-green-500';
            statusText.textContent = 'Success';
            statusText.className = 'text-xs font-medium text-green-600';
            console.log(`Step ${stepNumber} completed successfully`);
        } else {
            // Failure - show red status
            statusDot.className = 'inline-block w-3 h-3 rounded-full bg-red-500';
            statusText.textContent = 'Failed';
            statusText.className = 'text-xs font-medium text-red-600';
            console.error(`Step ${stepNumber} failed:`, data.message || 'Unknown error');
            showModal('Step Failed', `Step ${stepNumber} failed: ${data.message || 'Unknown error'}`, 'error');
        }
        
    } catch (error) {
        // Error - show red status
        statusDot.className = 'inline-block w-3 h-3 rounded-full bg-red-500';
        statusText.textContent = 'Failed';
        statusText.className = 'text-xs font-medium text-red-600';
        console.error(`Error executing Step ${stepNumber}:`, error);
        showModal('Execution Error', `Error executing Step ${stepNumber}: ${error.message}`, 'error');
    } finally {
        // Re-enable button after a short delay
        setTimeout(() => {
            btn.disabled = false;
        }, 1000);
    }
}

// Execute all steps sequentially
async function executeAllSteps() {
    // Check if operation unit is set
    if (currentServerIndex === null) {
        showModal('Operation Unit Required', 'Please set Operation Unit (Server Index) first!', 'warning');
        return;
    }
    
    const btnExecuteAll = document.getElementById('btnExecuteAll');
    if (!btnExecuteAll) return;
    
    // Disable execute all button
    btnExecuteAll.disabled = true;
    const originalText = btnExecuteAll.innerHTML;
    btnExecuteAll.innerHTML = '⏳ Executing...';
    
    try {
        console.log('Starting execution of all steps...');
        
        // Execute each step sequentially
        for (let i = 1; i <= 27; i++) {
            console.log(`Executing step ${i} of 27...`);
            await executeStep(i);
            
            // Check if the step failed
            const statusText = document.getElementById(`statusText${i}`);
            if (statusText && statusText.textContent === 'Failed') {
                console.error(`Step ${i} failed. Stopping execution.`);
                showModal('Execution Failed', `Execution stopped at Step ${i} due to failure.`, 'error');
                break;
            }
            
            // Small delay between steps
            await new Promise(resolve => setTimeout(resolve, 500));
        }
        
        console.log('All steps execution completed');
        
    } catch (error) {
        console.error('Error during all steps execution:', error);
        showModal('Execution Error', `Error during execution: ${error.message}`, 'error');
    } finally {
        // Re-enable the execute all button
        btnExecuteAll.disabled = false;
        btnExecuteAll.innerHTML = originalText;
    }
}


console.log('Task Manager JavaScript loaded');
