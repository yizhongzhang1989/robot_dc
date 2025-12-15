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
            robotModeElement.textContent = data.robot_mode_str;
            
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

console.log('Task Manager JavaScript loaded');
