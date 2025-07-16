// Robot Arm Web Control JavaScript

let commandCounter = 0;
let stateUpdateInterval;
let lastStateUpdate = null;
let updateCount = 0;
let updateRate = 0;

// Initialize the page
document.addEventListener('DOMContentLoaded', function() {
    fetchRobotArmInfo();
    updateConnectionStatus(true);
    startStateMonitoring();
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
    // Update TCP Position
    if (state.TCPActualPosition && state.TCPActualPosition.length >= 6) {
        const tcp = state.TCPActualPosition;
        document.getElementById('tcpPosX').textContent = tcp[0].toFixed(4);
        document.getElementById('tcpPosY').textContent = tcp[1].toFixed(4);
        document.getElementById('tcpPosZ').textContent = tcp[2].toFixed(4);
        document.getElementById('tcpPosRx').textContent = tcp[3].toFixed(4);
        document.getElementById('tcpPosRy').textContent = tcp[4].toFixed(4);
        document.getElementById('tcpPosRz').textContent = tcp[5].toFixed(4);
    }

    // Update Joint Temperatures
    if (state.jointActualTemperature) {
        const tempHtml = state.jointActualTemperature.map((temp, index) => {
            const tempClass = temp > 40 ? 'error-indicator' : 'normal-indicator';
            return `<div>J${index + 1}: <span class="${tempClass}">${temp.toFixed(1)}°C</span></div>`;
        }).join('');
        document.getElementById('jointTemperatures').innerHTML = tempHtml;
    }

    // Update Joint Currents
    if (state.jointActualCurrent) {
        const currentHtml = state.jointActualCurrent.map((current, index) => {
            const currentClass = current > 50 ? 'error-indicator' : 'normal-indicator';
            return `<div>J${index + 1}: <span class="${currentClass}">${current.toFixed(1)}‰</span></div>`;
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
        
        // Show first few driver states
        const stateText = state.driverState.slice(0, 3).map(s => s.toString()).join(', ');
        document.getElementById('driverStates').textContent = stateText;
    }

    // Update Joint Positions
    if (state.jointActualPosition) {
        const posHtml = state.jointActualPosition.map((pos, index) => {
            return `<div>J${index + 1}: ${pos.toFixed(4)} rad</div>`;
        }).join('');
        document.getElementById('jointPositions').innerHTML = posHtml;
    }

    // Update last update time
    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
    document.getElementById('dataStatus').textContent = 'Active';
    document.getElementById('dataStatus').className = 'normal-indicator';
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
