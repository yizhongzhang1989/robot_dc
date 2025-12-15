// Task Manager JavaScript

// Initialize when page loads
document.addEventListener('DOMContentLoaded', function() {
    console.log('Task Manager Interface loaded');
    
    // Initialize status bar
    initializeStatusBar();
    
    // Immediately fetch robot status multiple times for fast initial display
    fetchRobotStatus();
    setTimeout(() => fetchRobotStatus(), 100);
    setTimeout(() => fetchRobotStatus(), 300);
    setTimeout(() => fetchRobotStatus(), 500);
    
    // Check web services status
    checkWebServicesStatus();
    
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

// Start periodic status updates
function startPeriodicUpdates() {
    // Fetch robot status every 100ms for fast updates
    setInterval(() => {
        fetchRobotStatus();
    }, 100);
    
    // Check web services status every 5 seconds
    setInterval(() => {
        checkWebServicesStatus();
    }, 5000);
}

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

console.log('Task Manager JavaScript loaded');
