// UR15 Web Interface JavaScript

let freedriveActive = false;
let validationActive = false;
let currentOperationPath = ''; // Store the full operation path

// Web Log Functions
function logToWeb(message, type = 'info') {
    const logContainer = document.getElementById('webLogContainer');
    const timestamp = new Date().toLocaleTimeString('en-US', { hour12: false });
    const logEntry = document.createElement('div');
    logEntry.className = 'log-entry mb-1';
    
    // Set color based on type
    let colorClass = 'text-gray-300';
    let icon = 'ℹ️';
    switch(type) {
        case 'success':
            colorClass = 'text-green-400';
            icon = '✓';
            break;
        case 'error':
            colorClass = 'text-red-400';
            icon = '✗';
            break;
        case 'warning':
            colorClass = 'text-yellow-400';
            icon = '⚠';
            break;
        case 'info':
            colorClass = 'text-blue-400';
            icon = 'ℹ';
            break;
    }
    
    logEntry.innerHTML = `<span class="text-gray-500">[${timestamp}]</span> <span class="${colorClass}">${icon}</span> <span class="${colorClass}">${message}</span>`;
    
    logContainer.appendChild(logEntry);
    
    // Auto scroll to bottom
    logContainer.scrollTop = logContainer.scrollHeight;
    
    // Limit log entries to 100
    while (logContainer.children.length > 100) {
        logContainer.removeChild(logContainer.firstChild);
    }
}

function clearWebLog() {
    const logContainer = document.getElementById('webLogContainer');
    logContainer.innerHTML = '';
    logToWeb('Log cleared', 'info');
}

// Web Log polling
let lastLogIndex = -1;

function fetchWebLogs() {
    fetch(`/get_web_logs?last_index=${lastLogIndex}`)
    .then(response => response.json())
    .then(data => {
        if (data.success && data.messages && data.messages.length > 0) {
            data.messages.forEach(log => {
                logToWeb(log.message, log.type);
            });
            lastLogIndex = data.current_index;
        }
    })
    .catch(error => {
        // Silently fail to avoid spamming console
        // console.error('Failed to fetch web logs:', error);
    });
}

function showMessage(message, type = 'info', title = null) {
    const modal = document.getElementById('messageModal');
    const header = document.getElementById('messageModalHeader');
    const titleElement = document.getElementById('messageModalTitle');
    const text = document.getElementById('messageText');
    
    // Log the message
    logToWeb(message, type);
    
    // Set title based on type if not provided
    if (!title) {
        switch(type) {
            case 'success':
                title = '✓ Success';
                break;
            case 'error':
                title = '✗ Error';
                break;
            case 'warning':
                title = '⚠ Warning';
                break;
            default:
                title = 'ℹ Information';
        }
    }
    
    // Set header color based on type
    header.className = 'px-6 py-4 rounded-t-lg ';
    switch(type) {
        case 'success':
            header.className += 'bg-gradient-to-r from-green-500 to-green-600';
            break;
        case 'error':
            header.className += 'bg-gradient-to-r from-red-500 to-red-600';
            break;
        case 'warning':
            header.className += 'bg-gradient-to-r from-yellow-500 to-yellow-600';
            break;
        default:
            header.className += 'bg-gradient-to-r from-blue-500 to-blue-600';
    }
    
    titleElement.textContent = title;
    text.textContent = message;
    modal.classList.remove('hidden');
    modal.style.display = 'flex';
}

function closeMessageModal() {
    const modal = document.getElementById('messageModal');
    modal.classList.add('hidden');
    modal.style.display = 'none';
}

function changeDataDir() {
    const currentDir = document.getElementById('datasetDirPath').value;
    const modal = document.getElementById('dataDirModal');
    const input = document.getElementById('dataDirInput');
    
    // Set current directory as default value
    input.value = currentDir;
    
    // Show modal
    modal.classList.remove('hidden');
    modal.style.display = 'flex';
    
    // Focus on input
    setTimeout(() => input.focus(), 100);
    
    // Handle Enter key
    input.onkeypress = function(e) {
        if (e.key === 'Enter') {
            confirmDataDirChange();
        }
    };
}

function closeDataDirModal() {
    const modal = document.getElementById('dataDirModal');
    modal.classList.add('hidden');
    modal.style.display = 'none';
}

function confirmDataDirChange() {
    const currentDir = document.getElementById('datasetDirPath').value;
    const input = document.getElementById('dataDirInput');
    const newDir = input.value.trim();
    
    if (newDir === '' || newDir === currentDir) {
        closeDataDirModal();
        return;
    }
    
    logToWeb(`Changing dataset directory to: ${newDir}`, 'info');
    
    fetch('/change_data_dir', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ data_dir: newDir })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // Update the path panel
            document.getElementById('datasetDirPath').value = data.data_dir;
            // Update all task paths with new dataset directory
            updateAllTaskPaths(data.data_dir);
            logToWeb(`Dataset directory changed successfully to: ${data.data_dir}`, 'success');
            closeDataDirModal();
        } else {
            console.error('Failed to change directory:', data.message || 'Unknown error');
            logToWeb(`Failed to change directory: ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        console.error('Failed to change data directory:', error);
        logToWeb(`Failed to change data directory: ${error}`, 'error');
    });
}

// Operation Name Functions
function changeOperationName() {
    const currentName = document.getElementById('operationName').value;
    const modal = document.getElementById('operationPathModal');
    const input = document.getElementById('operationPathInput');
    
    // Set current name as default value (if not default text)
    if (currentName !== 'input operation name') {
        input.value = currentName;
    } else {
        input.value = '';
    }
    
    // Show modal
    modal.classList.remove('hidden');
    modal.style.display = 'flex';
    
    // Focus on input
    setTimeout(() => input.focus(), 100);
    
    // Handle Enter key
    input.onkeypress = function(e) {
        if (e.key === 'Enter') {
            confirmOperationNameChange();
        }
    };
}

function closeOperationPathModal() {
    const modal = document.getElementById('operationPathModal');
    modal.classList.add('hidden');
    modal.style.display = 'none';
}

// Alias for backward compatibility
function changeOperationPath() {
    changeOperationName();
}

function confirmOperationPathChange() {
    confirmOperationNameChange();
}

function confirmOperationNameChange() {
    const currentName = document.getElementById('operationName').value;
    const input = document.getElementById('operationPathInput');
    const newName = input.value.trim();
    
    if (newName === '' || newName === currentName) {
        closeOperationPathModal();
        return;
    }
    
    // Get dataset directory
    const datasetDir = document.getElementById('datasetDirPath').value;
    
    // Construct full operation path
    const fullOperationPath = `${datasetDir}/${newName}`;
    
    logToWeb(`Setting operation: ${newName}`, 'info');
    logToWeb(`Full operation path: ${fullOperationPath}`, 'info');
    
    // Update the display name and store the full path
    document.getElementById('operationName').value = newName;
    currentOperationPath = fullOperationPath;
    
    // Send to server (if needed for backend persistence)
    fetch('/change_operation_path', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ operation_path: fullOperationPath })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`Operation set successfully: ${newName}`, 'success');
            closeOperationPathModal();
        } else {
            console.error('Failed to set operation:', data.message || 'Unknown error');
            logToWeb(`Failed to set operation: ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        console.error('Failed to set operation:', error);
        logToWeb(`Failed to set operation: ${error}`, 'error');
    });
}

function updateAllTaskPaths(datasetDir) {
    // Reset operation name when dataset directory changes
    const operationNameInput = document.getElementById('operationName');
    if (operationNameInput) {
        operationNameInput.value = 'input operation name';
        currentOperationPath = '';
        logToWeb(`Operation name reset due to dataset directory change`, 'info');
    }
}

function captureTaskData(taskName) {
    const operationName = document.getElementById('operationName').value;
    const calibrationDataDir = document.getElementById('calibrationDirPath').value;
    
    if (!operationName || operationName === 'input operation name' || !currentOperationPath) {
        logToWeb(`Please set an operation name before capturing`, 'warning');
        showMessage(`Please set an operation name before capturing`, 'warning');
        return;
    }
    
    if (!calibrationDataDir || calibrationDataDir.trim() === '') {
        logToWeb(`Please set calibration data directory before capturing`, 'warning');
        showMessage(`Please set calibration data directory before capturing`, 'warning');
        return;
    }
    
    logToWeb(`Starting capture to: ${currentOperationPath}`, 'info');
    
    // Send capture request to server
    fetch('/capture_task_data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
            task_name: taskName,
            task_path: currentOperationPath.trim(),
            calibration_data_dir: calibrationDataDir.trim()
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`Capture completed`, 'success');
            logToWeb(`✓ Saved: ${data.image_file}`, 'success');
            logToWeb(`✓ Saved: ${data.pose_file}`, 'success');
            
            // Show calibration source information
            if (data.calibration_source) {
                logToWeb(`📷 Camera params from: ${data.calibration_source}`, 'info');
            }
            
            showMessage(
                `Successfully captured image and pose data:\n` +
                `• Image: ${data.image_file}\n` +
                `• Pose & Camera params: ${data.pose_file}` +
                (data.calibration_source ? `\n• Source: ${data.calibration_source}` : ''), 
                'success', 
                '📷 Capture Complete'
            );
        } else {
            console.error(`Failed to capture data:`, data.message || 'Unknown error');
            logToWeb(`Failed to capture data: ${data.message || 'Unknown error'}`, 'error');
            showMessage(
                `Failed to capture data: ${data.message || 'Unknown error'}`, 
                'error', 
                '❌ Capture Failed'
            );
        }
    })
    .catch(error => {
        console.error(`Failed to capture data:`, error);
        logToWeb(`Failed to capture data: ${error}`, 'error');
        showMessage(
            `Failed to capture data: ${error}`, 
            'error', 
            '❌ Capture Error'
        );
    });
}

function changeCalibrationDir() {
    const currentDir = document.getElementById('calibrationDirPath').value;
    const modal = document.getElementById('calibDirModal');
    const input = document.getElementById('calibDirInput');
    
    // Set current directory as default value
    input.value = currentDir;
    
    // Show modal
    modal.classList.remove('hidden');
    modal.style.display = 'flex';
    
    // Focus on input
    setTimeout(() => input.focus(), 100);
    
    // Handle Enter key
    input.onkeypress = function(e) {
        if (e.key === 'Enter') {
            confirmCalibDirChange();
        }
    };
}

function closeCalibDirModal() {
    const modal = document.getElementById('calibDirModal');
    modal.classList.add('hidden');
    modal.style.display = 'none';
}

function confirmCalibDirChange() {
    const currentDir = document.getElementById('calibrationDirPath').value;
    const input = document.getElementById('calibDirInput');
    const newDir = input.value.trim();
    
    if (newDir === '' || newDir === currentDir) {
        closeCalibDirModal();
        return;
    }
    
    logToWeb(`Changing calibration directory to: ${newDir}`, 'info');
    
    fetch('/change_calibration_dir', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ calibration_dir: newDir })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // Update the path panel
            document.getElementById('calibrationDirPath').value = data.calibration_data_dir;
            logToWeb(`Calibration directory changed successfully to: ${data.calibration_data_dir}`, 'success');
            closeCalibDirModal();
            // Update pose number after changing directory
            updatePoseNumber();
        } else {
            console.error('Failed to change calibration directory:', data.message || 'Unknown error');
            logToWeb(`Failed to change calibration directory: ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        console.error('Failed to change calibration directory:', error);
        logToWeb(`Failed to change calibration directory: ${error}`, 'error');
    });
}

function openChessboardConfigModal() {
    const modal = document.getElementById('chessboardConfigModal');
    const input = document.getElementById('chessboardConfigInput');
    const currentPath = document.getElementById('chessboardConfigPath').value;
    
    input.value = currentPath;
    modal.classList.remove('hidden');
    modal.style.display = 'flex';
    input.focus();
    
    // Handle Enter key
    input.onkeypress = function(e) {
        if (e.key === 'Enter') {
            confirmChessboardConfigChange();
        }
    };
}

function closeChessboardConfigModal() {
    const modal = document.getElementById('chessboardConfigModal');
    modal.classList.add('hidden');
    modal.style.display = 'none';
}

function confirmChessboardConfigChange() {
    const currentPath = document.getElementById('chessboardConfigPath').value;
    const input = document.getElementById('chessboardConfigInput');
    const newPath = input.value.trim();
    
    if (newPath === '' || newPath === currentPath) {
        closeChessboardConfigModal();
        return;
    }
    
    logToWeb(`Changing chessboard config to: ${newPath}`, 'info');
    
    fetch('/change_chessboard_config', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ chessboard_config: newPath })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // Update the path display
            document.getElementById('chessboardConfigPath').value = data.chessboard_config;
            logToWeb(`Chessboard config changed successfully to: ${data.chessboard_config}`, 'success');
            closeChessboardConfigModal();
        } else {
            console.error('Failed to change chessboard config:', data.message || 'Unknown error');
            logToWeb(`Failed to change chessboard config: ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        console.error('Failed to change chessboard config:', error);
        logToWeb(`Failed to change chessboard config: ${error}`, 'error');
    });
}

function updatePoseNumber() {
    fetch('/get_pose_count')
    .then(response => response.json())
    .then(data => {
        const display = document.getElementById('poseNumberDisplay');
        if (data.success) {
            display.textContent = data.count;
            display.style.color = '#374151'; // gray-700
        } else {
            display.textContent = 'Error: ' + (data.message || 'Unknown error');
            display.style.color = '#ef4444'; // red-500
        }
    })
    .catch(error => {
        console.error('Failed to get pose count:', error);
        const display = document.getElementById('poseNumberDisplay');
        display.textContent = 'Error loading';
        display.style.color = '#ef4444'; // red-500
    });
}

function toggleFreedrive() {
    const btn = document.getElementById('freedriveModeBtn');
    const statusValue = document.getElementById('statusBarFreedriveStatus');
    
    // Disable button temporarily
    btn.disabled = true;
    btn.style.opacity = '0.7';
    
    logToWeb('Toggling freedrive mode...', 'info');
    
    fetch('/toggle_freedrive', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            freedriveActive = data.freedrive_active;
            const mode = freedriveActive ? 'Freedrive' : 'Normal';
            
            // Update button text and appearance
            if (freedriveActive) {
                btn.querySelector('span:last-child').textContent = 'Disable Freedrive Mode';
                btn.classList.remove('bg-teal-500', 'hover:bg-teal-600');
                btn.classList.add('bg-red-500', 'hover:bg-red-600');
            } else {
                btn.querySelector('span:last-child').textContent = 'Enable Freedrive Mode';
                btn.classList.remove('bg-red-500', 'hover:bg-red-600');
                btn.classList.add('bg-teal-500', 'hover:bg-teal-600');
            }
            
            // Update status bar
            statusValue.textContent = mode;
            
            logToWeb(`Freedrive mode changed to: ${mode}`, 'success');
        }
        // Re-enable button
        btn.disabled = false;
        btn.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to toggle freedrive:', error);
        logToWeb(`Failed to toggle freedrive: ${error}`, 'error');
        // Re-enable button
        btn.disabled = false;
        btn.style.opacity = '1';
    });
}

function toggleValidation() {
    const btn = document.getElementById('validateCalibrationBtn');
    btn.disabled = true;
    btn.style.opacity = '0.7';
    
    fetch('/toggle_validation', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            validationActive = data.validation_active;
            
            // Update button appearance
            if (validationActive) {
                btn.classList.remove('bg-purple-500', 'hover:bg-purple-600');
                btn.classList.add('bg-red-500', 'hover:bg-red-600');
                btn.querySelector('span:last-child').textContent = 'Hide Validation Result';
            } else {
                btn.classList.remove('bg-red-500', 'hover:bg-red-600');
                btn.classList.add('bg-purple-500', 'hover:bg-purple-600');
                btn.querySelector('span:last-child').textContent = 'Validate Calibration Result';
            }
        }
        btn.disabled = false;
        btn.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to toggle validation:', error);
        btn.disabled = false;
        btn.style.opacity = '1';
    });
}

function uploadIntrinsic() {
    const fileInput = document.getElementById('intrinsicFile');
    const file = fileInput.files[0];
    
    if (!file) {
        return;
    }
    
    logToWeb(`Uploading intrinsic parameters: ${file.name}`, 'info');
    
    const formData = new FormData();
    formData.append('file', file);
    
    fetch('/upload_intrinsic', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb('Intrinsic parameters loaded successfully', 'success');
            updateStatus();
        } else {
            console.error('Failed to load intrinsic parameters:', data.message);
            logToWeb(`Failed to load intrinsic parameters: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        console.error('Upload failed:', error);
        logToWeb(`Intrinsic upload failed: ${error}`, 'error');
    });
}

function uploadExtrinsic() {
    const fileInput = document.getElementById('extrinsicFile');
    const file = fileInput.files[0];
    
    if (!file) {
        return;
    }
    
    logToWeb(`Uploading extrinsic parameters: ${file.name}`, 'info');
    
    const formData = new FormData();
    formData.append('file', file);
    
    fetch('/upload_extrinsic', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb('Extrinsic parameters loaded successfully', 'success');
            updateStatus();
        } else {
            console.error('Failed to load extrinsic parameters:', data.message);
            logToWeb(`Failed to load extrinsic parameters: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        console.error('Upload failed:', error);
        logToWeb(`Extrinsic upload failed: ${error}`, 'error');
    });
}

function takeScreenshot() {
    const btn = document.getElementById('screenshotBtn');
    const calibDataPath = document.getElementById('calibrationDirPath').value;
    
    btn.disabled = true;
    btn.style.opacity = '0.7';
    
    logToWeb('Taking screenshot...', 'info');
    
    fetch('/take_screenshot', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
            save_dir: calibDataPath 
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`Screenshot saved to: ${data.file_path || calibDataPath}`, 'success');
            // Update pose number after taking screenshot
            updatePoseNumber();
        } else {
            console.error('Failed to take screenshot:', data.message);
            logToWeb(`Failed to take screenshot: ${data.message}`, 'error');
        }
        btn.disabled = false;
        btn.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to take screenshot:', error);
        logToWeb(`Failed to take screenshot: ${error}`, 'error');
        btn.disabled = false;
        btn.style.opacity = '1';
    });
}

function autoCollectData() {
    const btn = document.getElementById('autoCollectBtn');
    btn.disabled = true;
    btn.style.opacity = '0.7';
    
    const calibDataDir = document.getElementById('calibrationDirPath').value;
    logToWeb('Starting auto collect data script...', 'info');
    logToWeb(`Data will be saved to: ${calibDataDir}`, 'info');
    
    fetch('/auto_collect_data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            console.log('Auto collect data script started successfully');
            showMessage('Collecting calibration data automatically!', 'success');
            if (data.log_file) {
                logToWeb(`Log file: ${data.log_file}`, 'info');
            }
        } else {
            console.error('Failed to start auto collect:', data.message);
            showMessage('Failed to start auto collect: ' + data.message, 'error');
        }
        btn.disabled = false;
        btn.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to start auto collect:', error);
        showMessage('Failed to start auto collect: ' + error, 'error');
        btn.disabled = false;
        btn.style.opacity = '1';
    });
}

function calibrateCam() {
    const btn = document.getElementById('calibrateCamBtn');
    btn.disabled = true;
    btn.style.opacity = '0.7';
    
    const calibDataDir = document.getElementById('calibrationDirPath').value;
    logToWeb('Starting camera calibration script...', 'info');
    logToWeb(`Data directory: ${calibDataDir}`, 'info');
    logToWeb(`Log file: ${calibDataDir}/calibrate_cam_log.txt`, 'info');
    
    fetch('/calibrate_cam', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            console.log('Camera calibration script started successfully');
            showMessage('Camera calibration script is running!', 'success');
            if (data.config_file) {
                logToWeb(`Config file: ${data.config_file}`, 'info');
            }
            if (data.output_dir) {
                logToWeb(`Results will be saved to: ${data.output_dir}`, 'info');
            }
        } else {
            console.error('Failed to start camera calibration:', data.message);
            showMessage('Failed to start camera calibration: ' + data.message, 'error');
        }
        btn.disabled = false;
        btn.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to start camera calibration:', error);
        showMessage('Failed to start camera calibration: ' + error, 'error');
        btn.disabled = false;
        btn.style.opacity = '1';
    });
}

function updateStatus() {
    fetch('/get_status')
    .then(response => response.json())
    .then(data => {
        console.log('Status update received:', data); // Debug log
        
        // Update robot status in status bar
        const statusBarRobotElement = document.getElementById('statusBarRobotStatus');
        if (data.robot_connected) {
            statusBarRobotElement.textContent = 'Connected';
            statusBarRobotElement.className = 'status-bar-value connected';
        } else {
            statusBarRobotElement.textContent = 'Disconnected';
            statusBarRobotElement.className = 'status-bar-value disconnected';
        }
        
        // Update camera status in status bar
        const statusBarCameraElement = document.getElementById('statusBarCameraStatus');
        if (data.has_image) {
            statusBarCameraElement.textContent = 'Connected';
            statusBarCameraElement.className = 'status-bar-value connected';
        } else {
            statusBarCameraElement.textContent = 'Disconnected';
            statusBarCameraElement.className = 'status-bar-value disconnected';
        }
        
        // Update intrinsic parameters status in status bar
        const statusBarIntrinsicElement = document.getElementById('statusBarIntrinsic');
        if (data.has_intrinsic) {
            statusBarIntrinsicElement.textContent = 'Loaded';
            statusBarIntrinsicElement.className = 'status-bar-value connected';
        } else {
            statusBarIntrinsicElement.textContent = 'Not Loaded';
            statusBarIntrinsicElement.className = 'status-bar-value disconnected';
        }
        
        // Update extrinsic parameters status in status bar
        const statusBarExtrinsicElement = document.getElementById('statusBarExtrinsic');
        if (data.has_extrinsic) {
            statusBarExtrinsicElement.textContent = 'Loaded';
            statusBarExtrinsicElement.className = 'status-bar-value connected';
        } else {
            statusBarExtrinsicElement.textContent = 'Not Loaded';
            statusBarExtrinsicElement.className = 'status-bar-value disconnected';
        }
        
        // Update image topic in status bar
        document.getElementById('statusBarImageTopic').textContent = data.camera_topic;
        
        // Update data dir in path panel
        if (data.data_dir) {
            document.getElementById('datasetDirPath').value = data.data_dir;
        }
        
        // Update calibration data dir in path panel
        if (data.calibration_data_dir) {
            document.getElementById('calibrationDirPath').value = data.calibration_data_dir;
        }
        
        // Update joint positions in Control Panel
        if (data.joint_positions && data.joint_positions.length > 0) {
            for (let i = 0; i < 6; i++) {
                const jointElement = document.getElementById(`joint${i + 1}Value`);
                if (jointElement && i < data.joint_positions.length) {
                    // Don't update if user is currently editing this input
                    if (document.activeElement !== jointElement) {
                        // Data from server is already in degrees, no conversion needed
                        const newValueDegrees = data.joint_positions[i].toFixed(1);
                        jointElement.value = newValueDegrees;
                    }
                }
            }
        } else {
            // Set all joint values to no data state (only if not being edited)
            for (let i = 0; i < 6; i++) {
                const jointElement = document.getElementById(`joint${i + 1}Value`);
                if (jointElement && document.activeElement !== jointElement) {
                    jointElement.value = '--';
                }
            }
        }
        
        // Update TCP pose in Control Panel
        if (data.tcp_pose) {
            const pose = data.tcp_pose;
            // Convert quaternion to axis-angle representation
            const axisAngle = quaternionToAxisAngle(pose.qx, pose.qy, pose.qz, pose.qw);
            
            // Update position values (only if not being edited)
            const tcpXElement = document.getElementById('tcpXValue');
            const tcpYElement = document.getElementById('tcpYValue');
            const tcpZElement = document.getElementById('tcpZValue');
            const tcpRXElement = document.getElementById('tcpRXValue');
            const tcpRYElement = document.getElementById('tcpRYValue');
            const tcpRZElement = document.getElementById('tcpRZValue');
            
            if (tcpXElement && document.activeElement !== tcpXElement) {
                tcpXElement.value = pose.x.toFixed(1);
            }
            if (tcpYElement && document.activeElement !== tcpYElement) {
                tcpYElement.value = pose.y.toFixed(1);
            }
            if (tcpZElement && document.activeElement !== tcpZElement) {
                tcpZElement.value = pose.z.toFixed(1);
            }
            if (tcpRXElement && document.activeElement !== tcpRXElement) {
                tcpRXElement.value = axisAngle.rx.toFixed(1);
            }
            if (tcpRYElement && document.activeElement !== tcpRYElement) {
                tcpRYElement.value = axisAngle.ry.toFixed(1);
            }
            if (tcpRZElement && document.activeElement !== tcpRZElement) {
                tcpRZElement.value = axisAngle.rz.toFixed(1);
            }
        } else {
            // Set all TCP values to no data state (only if not being edited)
            const tcpElements = ['tcpXValue', 'tcpYValue', 'tcpZValue', 'tcpRXValue', 'tcpRYValue', 'tcpRZValue'];
            tcpElements.forEach(id => {
                const element = document.getElementById(id);
                if (element && document.activeElement !== element) {
                    element.value = '--';
                }
            });
        }
        
        // Update freedrive mode status
        if (data.freedrive_active !== undefined) {
            freedriveActive = data.freedrive_active;
            
            // Update status bar freedrive status
            const statusBarFreedriveElement = document.getElementById('statusBarFreedriveStatus');
            if (data.freedrive_active) {
                statusBarFreedriveElement.textContent = 'Freedrive';
                statusBarFreedriveElement.className = 'status-bar-value connected';
            } else {
                statusBarFreedriveElement.textContent = 'Normal';
                statusBarFreedriveElement.className = 'status-bar-value';
            }
        }
    })
    .catch(error => {
        console.error('Status fetch failed:', error);
        // Update status bar on connection error
        const statusBarRobotElement = document.getElementById('statusBarRobotStatus');
        if (statusBarRobotElement) {
            statusBarRobotElement.textContent = 'Connection Error';
            statusBarRobotElement.className = 'status-bar-value disconnected';
        }
        
        const statusBarCameraElement = document.getElementById('statusBarCameraStatus');
        if (statusBarCameraElement) {
            statusBarCameraElement.textContent = 'Connection Error';
            statusBarCameraElement.className = 'status-bar-value disconnected';
        }
    });
}

// Delete Image Modal Functions
function openDeleteImageModal() {
    const modal = document.getElementById('deleteImageModal');
    const calibDataPath = document.getElementById('calibrationDirPath').value;
    
    // Display current directory
    document.getElementById('deleteImageDirDisplay').textContent = calibDataPath;
    
    // Fetch image count from backend
    fetch('/get_image_count', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ directory: calibDataPath })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            document.getElementById('deleteImageCountDisplay').textContent = data.count;
            logToWeb(`Found ${data.count} images in calibration directory`, 'info');
        } else {
            document.getElementById('deleteImageCountDisplay').textContent = '0';
            logToWeb(`Failed to get image count: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        document.getElementById('deleteImageCountDisplay').textContent = 'Error';
        logToWeb(`Failed to get image count: ${error}`, 'error');
    });
    
    // Clear input field
    document.getElementById('deleteImageNumberInput').value = '';
    
    // Show modal
    modal.style.display = 'flex';
}

function closeDeleteImageModal() {
    const modal = document.getElementById('deleteImageModal');
    modal.style.display = 'none';
}

function confirmDeleteImage() {
    const calibDataPath = document.getElementById('calibrationDirPath').value;
    const input = document.getElementById('deleteImageNumberInput').value.trim();
    
    if (input === '') {
        logToWeb('Please enter an image number or "all"', 'warning');
        return;
    }
    
    const deleteAll = input.toLowerCase() === 'all';
    const imageNumber = deleteAll ? null : parseInt(input);
    
    if (!deleteAll && (isNaN(imageNumber) || imageNumber < 0)) {
        logToWeb('Invalid image number. Please enter a valid number or "all"', 'error');
        return;
    }
    
    logToWeb(deleteAll ? 'Deleting all images...' : `Deleting image #${imageNumber}...`, 'info');
    
    fetch('/delete_calibration_image', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
            directory: calibDataPath,
            image_number: imageNumber,
            delete_all: deleteAll
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(data.message, 'success');
            closeDeleteImageModal();
            // Update pose number after deleting images
            updatePoseNumber();
        } else {
            logToWeb(`Failed to delete: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`Failed to delete: ${error}`, 'error');
    });
}

// Update status every 200ms
setInterval(() => {
    updateStatus();
}, 200);

// Fetch web logs every 1000ms
setInterval(() => {
    fetchWebLogs();
}, 1000);

// Convert quaternion to axis-angle representation
function quaternionToAxisAngle(qx, qy, qz, qw) {
    // Normalize quaternion
    const norm = Math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    const x = qx / norm;
    const y = qy / norm;
    const z = qz / norm;
    const w = qw / norm;
    
    // Calculate angle
    const angle = 2 * Math.acos(Math.abs(w));
    
    // Calculate axis
    const s = Math.sqrt(1 - w*w);
    let axisX, axisY, axisZ;
    
    if (s < 0.001) {
        // If s is close to zero, rotation is close to identity
        axisX = x;
        axisY = y;
        axisZ = z;
    } else {
        axisX = x / s;
        axisY = y / s;
        axisZ = z / s;
    }
    
    // Return axis-angle representation (rx, ry, rz are the rotation vector components)
    return {
        rx: axisX * angle * 180 / Math.PI, // Convert to degrees
        ry: axisY * angle * 180 / Math.PI,
        rz: axisZ * angle * 180 / Math.PI
    };
}

// Joint angle movement functions
function moveToJointPosition(jointValuesRad) {
    fetch('/movej', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            joint_positions: jointValuesRad
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // Convert radians back to degrees for display
            const jointValuesDeg = jointValuesRad.map(v => v * 180 / Math.PI);
            logToWeb(`Robot moving to: [${jointValuesDeg.map(v => v.toFixed(1)).join(', ')}]°`, 'success');
        } else {
            logToWeb(`Movement failed: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`Movement error: ${error}`, 'error');
    });
}

function setupJointInputListeners() {
    const jointInputs = [
        'joint1Value', 'joint2Value', 'joint3Value', 
        'joint4Value', 'joint5Value', 'joint6Value'
    ];

    jointInputs.forEach((inputId, index) => {
        const inputElement = document.getElementById(inputId);
        if (inputElement) {
            // Add event listener for Enter key and blur (when user finishes editing)
            inputElement.addEventListener('keypress', function(event) {
                if (event.key === 'Enter') {
                    this.blur(); // Trigger blur event to process the change
                }
            });

            // Add event listener for input changes (including +/- buttons)
            let inputTimeout;
            inputElement.addEventListener('input', function() {
                // Clear previous timeout to avoid multiple rapid calls
                clearTimeout(inputTimeout);
                
                // Set a short delay to avoid too frequent calls while user is typing
                inputTimeout = setTimeout(() => {
                    processJointChange.call(this, index);
                }, 100); // 100ms delay
            });

            inputElement.addEventListener('blur', function() {
                // Clear timeout since we're processing immediately
                clearTimeout(inputTimeout);
                processJointChange.call(this, index);
            });

            // Add visual feedback when input is focused
            inputElement.addEventListener('focus', function() {
                this.style.backgroundColor = '#e0f2fe';
                this.style.borderColor = '#0ea5e9';
            });

            inputElement.addEventListener('blur', function() {
                // Reset to default styling (will be handled by CSS)
                this.style.backgroundColor = '';
                this.style.borderColor = '';
            });
        }
    });
}

// Separate function to handle joint change processing
function processJointChange(jointIndex) {
    const value = parseFloat(this.value);
    
    // Skip if value is invalid or still showing placeholder
    if (isNaN(value) || this.value === '--') {
        return;
    }

    // Get all current joint values
    const jointValues = [];
    let allValid = true;

    for (let i = 0; i < 6; i++) {
        const joint = document.getElementById(`joint${i + 1}Value`);
        const jointValue = parseFloat(joint.value);
        
        if (isNaN(jointValue) || joint.value === '--') {
            allValid = false;
            break;
        }
        
        // Convert degrees to radians
        jointValues.push(jointValue * Math.PI / 180);
    }

    // Only move if all joint values are valid
    if (allValid) {
        logToWeb(`Joint ${jointIndex + 1} changed to ${value.toFixed(1)}°`, 'info');
        moveToJointPosition(jointValues);
    } else {
        logToWeb('Cannot move: Some joint values are invalid', 'warning');
    }
}

// TCP pose movement functions
function moveToTcpPose(tcpPoseValues) {
    fetch('/movel', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            tcp_pose: tcpPoseValues
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // Format display values
            const posDisplay = tcpPoseValues.slice(0, 3).map(v => v.toFixed(1) + 'mm');
            const rotDisplay = tcpPoseValues.slice(3, 6).map(v => v.toFixed(2) + '°');
            logToWeb(`Robot moving to TCP: [${posDisplay.concat(rotDisplay).join(', ')}]`, 'success');
        } else {
            logToWeb(`TCP movement failed: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`TCP movement error: ${error}`, 'error');
    });
}

function setupTcpInputListeners() {
    const tcpInputs = [
        'tcpXValue', 'tcpYValue', 'tcpZValue', 
        'tcpRXValue', 'tcpRYValue', 'tcpRZValue'
    ];

    tcpInputs.forEach((inputId, index) => {
        const inputElement = document.getElementById(inputId);
        if (inputElement) {
            // Add event listener for Enter key and blur (when user finishes editing)
            inputElement.addEventListener('keypress', function(event) {
                if (event.key === 'Enter') {
                    this.blur(); // Trigger blur event to process the change
                }
            });

            // Add event listener for input changes (including +/- buttons)
            let tcpInputTimeout;
            inputElement.addEventListener('input', function() {
                // Clear previous timeout to avoid multiple rapid calls
                clearTimeout(tcpInputTimeout);
                
                // Set a short delay to avoid too frequent calls while user is typing
                tcpInputTimeout = setTimeout(() => {
                    processTcpChange.call(this, index);
                }, 100); // 100ms delay
            });

            inputElement.addEventListener('blur', function() {
                // Clear timeout since we're processing immediately
                clearTimeout(tcpInputTimeout);
                processTcpChange.call(this, index);
            });

            // Add visual feedback when input is focused
            inputElement.addEventListener('focus', function() {
                this.style.backgroundColor = '#fef3c7';
                this.style.borderColor = '#f59e0b';
            });

            inputElement.addEventListener('blur', function() {
                // Reset to default styling (will be handled by CSS)
                this.style.backgroundColor = '';
                this.style.borderColor = '';
            });
        }
    });
}

// Separate function to handle TCP pose change processing
function processTcpChange(tcpIndex) {
    const value = parseFloat(this.value);
    
    // Skip if value is invalid or still showing placeholder
    if (isNaN(value) || this.value === '--') {
        return;
    }

    // Get all current TCP pose values
    const tcpPoseValues = [];
    let allValid = true;

    // Position values (X, Y, Z in mm)
    for (let i = 0; i < 3; i++) {
        const element = document.getElementById(['tcpXValue', 'tcpYValue', 'tcpZValue'][i]);
        const val = parseFloat(element.value);
        
        if (isNaN(val) || element.value === '--') {
            allValid = false;
            break;
        }
        
        // Values are already in mm, no conversion needed
        tcpPoseValues.push(val);
    }

    // Rotation values (RX, RY, RZ in degrees)
    if (allValid) {
        for (let i = 0; i < 3; i++) {
            const element = document.getElementById(['tcpRXValue', 'tcpRYValue', 'tcpRZValue'][i]);
            const val = parseFloat(element.value);
            
            if (isNaN(val) || element.value === '--') {
                allValid = false;
                break;
            }
            
            // Values are in degrees, no conversion needed
            tcpPoseValues.push(val);
        }
    }

    // Only move if all TCP pose values are valid
    if (allValid) {
        const coordName = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ'][tcpIndex];
        const unit = tcpIndex < 3 ? 'mm' : '°';
        logToWeb(`TCP ${coordName} changed to ${value.toFixed(tcpIndex < 3 ? 1 : 2)}${unit}`, 'info');
        moveToTcpPose(tcpPoseValues);
    } else {
        logToWeb('Cannot move: Some TCP pose values are invalid', 'warning');
    }
}

// Update status on page load
window.onload = function() {
    updateStatus();
    updatePoseNumber();
    setupJointInputListeners(); // Setup joint input event listeners
    setupTcpInputListeners(); // Setup TCP pose input event listeners
    
    // Initialize operation name display
    const operationNameElement = document.getElementById('operationName');
    if (operationNameElement && operationNameElement.value === '{{ data_dir }}') {
        operationNameElement.value = 'input operation name';
    }
    
    logToWeb('UR15 Web Interface loaded', 'success');
    logToWeb('System ready for operation', 'info');
    logToWeb('Joint angles and TCP pose are now editable - press Enter or click away to move robot', 'info');
};