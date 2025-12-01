// UR15 Web Interface JavaScript

let freedriveActive = false;
let validationActive = false;
let cornerDetectionEnabled = false;
let currentOperationPath = ''; // Store the full operation path

// Robot state update throttling
let lastRobotStateUpdate = 0;
let pendingRobotStateUpdate = null;
const ROBOT_STATE_UPDATE_INTERVAL = 1000; // Update every 1 second

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
    const inputElement = document.getElementById('operationName');
    const newName = inputElement.value.trim();
    
    // Check if the name is empty or still the default placeholder
    if (newName === '' || newName === 'input operation name') {
        logToWeb('Please enter a valid operation name', 'warning');
        inputElement.focus();
        return;
    }
    
    // Get dataset directory
    const datasetDir = document.getElementById('datasetDirPath').value;
    
    // Construct full operation path
    const fullOperationPath = `${datasetDir}/${newName}`;
    
    logToWeb(`Setting operation name: ${newName}`, 'info');
    logToWeb(`Full operation path: ${fullOperationPath}`, 'info');
    
    // Store the full path
    currentOperationPath = fullOperationPath;
    
    // Send to server (if needed for backend persistence)
    fetch('/change_operation_path', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
            operation_path: fullOperationPath,
            operation_name: newName
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`Operation set successfully: ${newName}`, 'success');
            // Enable the Locate Last Operation button
            const locateBtn = document.getElementById('locateLastOperationBtn');
            if (locateBtn) {
                locateBtn.disabled = false;
                locateBtn.classList.remove('opacity-50', 'cursor-not-allowed');
                locateBtn.classList.add('hover:bg-purple-600');
            }
        } else {
            logToWeb(`Failed to set operation: ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`Error setting operation: ${error.message}`, 'error');
        console.error('Error:', error);
    });
}

// Alias for backward compatibility
function changeOperationPath() {
    changeOperationName();
}

function setOperatingUnit() {
    const inputElement = document.getElementById('operatingUnit');
    const unitValue = parseInt(inputElement.value);
    
    // Validate the input
    if (isNaN(unitValue) || unitValue < 1) {
        logToWeb('Please enter a valid operating unit (must be >= 1)', 'warning');
        inputElement.focus();
        return;
    }
    
    logToWeb(`Setting operating unit to: ${unitValue}`, 'info');
    
    // Send to server to set in robot_status
    fetch('/set_operating_unit', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
            operating_unit: unitValue
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`Operating unit set successfully: ${unitValue}`, 'success');
        } else {
            logToWeb(`Failed to set operating unit: ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`Error setting operating unit: ${error.message}`, 'error');
        console.error('Error:', error);
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
        showMessage(`Please set an operation name before capturing`, 'warning');
        return;
    }
    
    if (!calibrationDataDir || calibrationDataDir.trim() === '') {
        logToWeb(`Please set correct calibration data directory before capturing`, 'warning');
        showMessage(`Please set calibration data directory before capturing`, 'warning');
        return;
    }
    
    logToWeb(`Starting capture...`, 'info');
    
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
            logToWeb(`Captured image and pose data successfully`, 'success');
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

function updateCaptureX3ButtonState() {
    const captureX3Btn = document.getElementById('captureX3Btn');
    if (!captureX3Btn) return;
    
    if (freedriveActive) {
        // Disable button when freedrive is active
        captureX3Btn.disabled = true;
        captureX3Btn.style.opacity = '0.5';
        captureX3Btn.style.cursor = 'not-allowed';
        captureX3Btn.title = 'Cannot use while freedrive mode is active. Please disable freedrive mode first.';
        captureX3Btn.classList.remove('hover:bg-orange-600');
    } else {
        // Enable button when freedrive is inactive
        captureX3Btn.disabled = false;
        captureX3Btn.style.opacity = '1';
        captureX3Btn.style.cursor = 'pointer';
        captureX3Btn.title = 'Capture 3 images at different positions (x, y offsets)';
        captureX3Btn.classList.add('hover:bg-orange-600');
    }
}

function loadWorkflowFiles() {
    fetch('/get_workflow_files', {
        method: 'GET',
    })
    .then(response => response.json())
    .then(data => {
        const select = document.getElementById('workflowFiles');
        select.innerHTML = ''; // Clear existing options
        
        if (data.status === 'success' && data.files.length > 0) {
            // Add default option
            const defaultOption = document.createElement('option');
            defaultOption.value = '';
            defaultOption.textContent = 'Select a workflow file';
            select.appendChild(defaultOption);
            
            // Add file options
            data.files.forEach(file => {
                const option = document.createElement('option');
                option.value = file;
                option.textContent = file;
                select.appendChild(option);
            });
            
            logToWeb(`Loaded ${data.files.length} workflow files`, 'info');
        } else {
            const option = document.createElement('option');
            option.value = '';
            option.textContent = 'No workflow files found';
            select.appendChild(option);
            logToWeb('No workflow files found', 'warning');
        }
    })
    .catch(error => {
        logToWeb(`Error loading workflow files: ${error.message}`, 'error');
        const select = document.getElementById('workflowFiles');
        select.innerHTML = '<option value="">Error loading files</option>';
    });
}

function runCurrentWorkflow() {
    const select = document.getElementById('workflowFiles');
    const selectedFile = select.value;
    
    if (!selectedFile || selectedFile === '') {
        logToWeb('Please select a workflow file first', 'warning');
        showMessage('Please select a workflow file', 'warning');
        return;
    }
    
    logToWeb(`Running workflow: ${selectedFile}...`, 'info');
    
    fetch('/run_workflow', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            workflow_file: selectedFile
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success') {
            logToWeb(`Workflow started successfully: ${selectedFile}`, 'success');
        } else {
            logToWeb(`Failed to start workflow: ${data.message}`, 'error');
            showMessage(`Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`Error running workflow: ${error.message}`, 'error');
        showMessage('Failed to run workflow', 'error');
    });
}

function goToWorkflowConfigCenter() {
    logToWeb('Opening workflow configuration center...', 'info');
    
    fetch('/workflow_config_center_url', {
        method: 'GET',
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success') {
            // Open workflow config center in new tab
            window.open(data.url, '_blank');
            logToWeb('Workflow configuration center opened in new tab', 'success');
        } else {
            logToWeb(`Error: ${data.message}`, 'error');
            showMessage(`Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`Error opening workflow config center: ${error.message}`, 'error');
        showMessage('Failed to open workflow config center', 'error');
    });
}

function labelLastCapturedImage() {
    logToWeb('Preparing last captured image for labeling...', 'info');
    
    fetch('/prepare_last_captured_image', {
        method: 'GET',
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success') {
            // Open image labeling web in new tab with image URL
            window.open(data.labeling_url, '_blank');
            logToWeb('Image labeling service opened in new tab', 'success');
        } else {
            logToWeb(`Error: ${data.message}`, 'error');
            showMessage(`Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`Failed to prepare image: ${error}`, 'error');
        showMessage(`Failed to prepare image: ${error}`, 'error');
    });
}

function captureRefImg1() {
    const operationName = document.getElementById('operationName').value;
    const calibrationDataDir = document.getElementById('calibrationDirPath').value;
    
    if (!operationName || operationName === 'input operation name' || !currentOperationPath) {
        showMessage(`Please set an operation name before capturing`, 'warning');
        return;
    }
    
    if (!calibrationDataDir || calibrationDataDir.trim() === '') {
        logToWeb(`Please set correct calibration data directory before capturing`, 'warning');
        showMessage(`Please set calibration data directory before capturing`, 'warning');
        return;
    }
    
    logToWeb(`Capturing ref_img_1...`, 'info');
    
    // Send capture request to server
    fetch('/capture_ref_img_1', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
            task_name: 'crack',
            task_path: currentOperationPath.trim(),
            calibration_data_dir: calibrationDataDir.trim()
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`Captured ref_img_1 successfully`, 'success');
        } else {
            console.error(`Failed to capture ref_img_1:`, data.message || 'Unknown error');
            logToWeb(`Failed to capture ref_img_1: ${data.message || 'Unknown error'}`, 'error');
            showMessage(
                `Failed to capture ref_img_1: ${data.message || 'Unknown error'}`, 
                'error', 
                '❌ Capture Failed'
            );
        }
    })
    .catch(error => {
        console.error(`Failed to capture ref_img_1:`, error);
        logToWeb(`Failed to capture ref_img_1: ${error}`, 'error');
        showMessage(
            `Failed to capture ref_img_1: ${error}`, 
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
            logToWeb(`Calibration data directory changed successfully to: ${data.calibration_data_dir}`, 'success');
            closeCalibDirModal();
            // Update pose number after changing directory
            updatePoseNumber();
        } else {
            console.error('Failed to change calibration data directory:', data.message || 'Unknown error');
            logToWeb(`Failed to change calibration data directory: ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        console.error('Failed to change calibration data directory:', error);
        logToWeb(`Failed to change calibration data directory: ${error}`, 'error');
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
    
    if (newPath === '') {
        closeChessboardConfigModal();
        return;
    }
    
    // If path is same, try to reload the config file (in case it was created after launch)
    if (newPath === currentPath) {
        logToWeb(`Reloading chessboard config from: ${newPath}`, 'info');
        fetch('/load_chessboard_config', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ config_path: newPath })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                logToWeb(`Chessboard config reloaded successfully`, 'success');
            } else {
                logToWeb(`Failed to reload config: ${data.message || 'Unknown error'}`, 'warning');
            }
            closeChessboardConfigModal();
        })
        .catch(error => {
            console.error('Failed to reload chessboard config:', error);
            logToWeb(`Failed to reload chessboard config: ${error}`, 'error');
            closeChessboardConfigModal();
        });
        return;
    }
    
    logToWeb(`Changing chessboard config path to: ${newPath}`, 'info');
    
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
            logToWeb(`Chessboard config path changed successfully to: ${data.chessboard_config}`, 'success');
            closeChessboardConfigModal();
        } else {
            console.error('Failed to change chessboard config:', data.message || 'Unknown error');
            logToWeb(`Failed to change chessboard config path : ${data.message || 'Unknown error'}`, 'error');
        }
    })
    .catch(error => {
        console.error('Failed to change chessboard config path:', error);
        logToWeb(`Failed to change chessboard config path: ${error}`, 'error');
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
            
            // Update Capture x3 button state
            updateCaptureX3ButtonState();
            
            logToWeb(`Drive mode changed to: ${mode}`, 'success');
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
    const checkbox = document.getElementById('validateCalibrationCheckbox');
    checkbox.disabled = true;
    
    logToWeb('Toggling calibration validation...', 'info');
    
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
            checkbox.checked = validationActive;
            
            if (validationActive) {
                logToWeb('Draw UR15 base activated', 'success');
            } else {
                logToWeb('Draw UR15 base deactivated', 'info');
            }
        } else {
            logToWeb(`Failed to toggle validation: ${data.message}`, 'error');
            // Revert checkbox state on failure
            checkbox.checked = !checkbox.checked;
        }
        checkbox.disabled = false;
    })
    .catch(error => {
        console.error('Failed to toggle validation:', error);
        logToWeb(`Error toggling validation: ${error.message}`, 'error');
        // Revert checkbox state on error
        checkbox.checked = !checkbox.checked;
        checkbox.disabled = false;
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
                logToWeb(`Log file will be saved as: ${data.log_file}`, 'info');
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
    logToWeb(`Log file will be saved as: ${calibDataDir}/calibrate_cam_log.txt`, 'info');
    
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
            if (data.config_file) {
                logToWeb(`Config file: ${data.config_file}`, 'info');
            }
            if (data.output_dir) {
                logToWeb(`Results will be saved to: ${data.output_dir}`, 'info');
            }
        } else {
            console.error('Failed to start camera calibration:', data.message);
            logToWeb(`Failed to start camera calibration: ${data.message}`, 'error');
        }
        btn.disabled = false;
        btn.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to start camera calibration:', error);
        logToWeb(`Failed to start camera calibration: ${error}`, 'error');
        btn.disabled = false;
        btn.style.opacity = '1';
    });
}

// Send robot state (joint positions and TCP pose) to robot_status
function sendRobotStateToStatus(jointPositions, tcpPose) {
    const now = Date.now();
    
    // Throttle updates to avoid overwhelming the server
    if (now - lastRobotStateUpdate < ROBOT_STATE_UPDATE_INTERVAL) {
        // Store pending update to send later
        if (pendingRobotStateUpdate) {
            clearTimeout(pendingRobotStateUpdate);
        }
        
        pendingRobotStateUpdate = setTimeout(() => {
            sendRobotStateToStatus(jointPositions, tcpPose);
        }, ROBOT_STATE_UPDATE_INTERVAL - (now - lastRobotStateUpdate));
        
        return;
    }
    
    lastRobotStateUpdate = now;
    pendingRobotStateUpdate = null;
    
    // Prepare data to send
    const dataToSend = {};
    
    if (jointPositions && jointPositions.length === 6) {
        // Joint positions are already in degrees (deg), send as array for numpy conversion
        dataToSend.joint_positions = jointPositions;
    }
    
    if (tcpPose) {
        // Convert TCP pose from mm and quaternion to m and axis-angle (rad)
        const axisAngle = quaternionToAxisAngle(tcpPose.qx, tcpPose.qy, tcpPose.qz, tcpPose.qw);
        
        // Send as array in order: [x, y, z, rx, ry, rz] for numpy conversion
        dataToSend.tcp_pose = [
            tcpPose.x / 1000.0,  // x in meters
            tcpPose.y / 1000.0,  // y in meters
            tcpPose.z / 1000.0,  // z in meters
            axisAngle.rx * Math.PI / 180.0,  // rx in radians
            axisAngle.ry * Math.PI / 180.0,  // ry in radians
            axisAngle.rz * Math.PI / 180.0   // rz in radians
        ];
    }
    
    // Only send if there's data to update
    if (Object.keys(dataToSend).length === 0) {
        return;
    }
    
    // Send to server
    fetch('/update_robot_state', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(dataToSend)
    })
    .then(response => response.json())
    .then(data => {
        if (!data.success) {
            console.warn('Failed to update robot state to robot_status:', data.message);
        }
    })
    .catch(error => {
        console.error('Error updating robot state:', error);
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
        
        // Update board type in status bar
        const statusBarBoardTypeElement = document.getElementById('statusBarBoardType');
        if (data.board_type_loaded) {
            statusBarBoardTypeElement.textContent = data.board_type_display;
            statusBarBoardTypeElement.className = 'status-bar-value connected';
        } else {
            statusBarBoardTypeElement.textContent = 'Unloaded';
            statusBarBoardTypeElement.className = 'status-bar-value disconnected';
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
            
            // Send joint positions to robot_status
            sendRobotStateToStatus(data.joint_positions, data.tcp_pose);
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
            
            // Update Capture x3 button state
            updateCaptureX3ButtonState();
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

// Corner Detection Functions
function toggleCornerDetection() {
    if (!cornerDetectionEnabled) {
        showCornerDetectionModal();
    } else {
        disableCornerDetection();
    }
}

async function showCornerDetectionModal() {
    const boardType = document.getElementById('boardTypeSelect').value;
    const modal = document.getElementById('cornerDetectionModal');
    
    // Define pattern info
    const patternInfo = {
        'ChessBoard': { icon: '🏁', name: 'ChessBoard', description: 'Standard chessboard pattern with black and white squares' },
        'CharucoBoard': { icon: '🎯', name: 'CharUco Board', description: 'CharUco boards combine chessboard and ArUco markers for robust detection' },
        'GridBoard': { icon: '📐', name: 'ArUco Grid Board', description: 'Grid boards use ArUco markers arranged in a grid pattern' }
    };
    
    const pattern = patternInfo[boardType] || patternInfo['ChessBoard'];
    
    // Update modal title
    const title = document.getElementById('patternModalTitle');
    if (title) {
        title.textContent = `${pattern.icon} ${pattern.name} Settings`;
    }
    
    // Update description
    const description = document.getElementById('patternDescription');
    if (description) {
        description.textContent = pattern.description;
    }
    
    // Try to load chessboard config and use it as default values
    try {
        const configPath = document.getElementById('chessboardConfigPath').value;
        const response = await fetch('/load_chessboard_config', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ config_path: configPath })
        });
        const data = await response.json();
        
        if (data.success && data.config) {
            // Generate parameters with config values
            generatePatternParametersWithConfig(boardType, data.config);
        } else {
            // Generate parameters with default values
            generatePatternParameters(boardType);
        }
    } catch (error) {
        console.log('Could not load chessboard config, using defaults:', error);
        // Generate parameters with default values
        generatePatternParameters(boardType);
    }
    
    modal.classList.remove('hidden');
}

function closeCornerDetectionModal() {
    const modal = document.getElementById('cornerDetectionModal');
    modal.classList.add('hidden');
}

async function confirmCornerDetection() {
    const width = parseInt(document.getElementById('detectionWidth').value);
    const height = parseInt(document.getElementById('detectionHeight').value);
    const boardTypeSelect = document.getElementById('boardTypeSelect');
    const boardType = boardTypeSelect ? boardTypeSelect.value : 'ChessBoard';
    
    if (isNaN(width) || isNaN(height) || width < 3 || height < 3) {
        showMessage('Invalid detection parameters. Width and height must be at least 3.', 'error');
        return;
    }
    
    try {
        const requestData = {
            enable: true,
            pattern_type: boardType,
            chessboard_width: width - 1,  // Convert to corners (internal corners = squares - 1)
            chessboard_height: height - 1,
            grid_width: width,
            grid_height: height
        };
        
        logToWeb(`Enabling corner detection: ${boardType} ${width}x${height}`, 'info');
        
        const response = await fetch('/api/calibration/corner-detection', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(requestData)
        });
        
        const data = await response.json();
        if (data.success) {
            cornerDetectionEnabled = true;
            document.getElementById('cornerDetectionBtn').innerHTML = '<span>🛑</span><span>Stop Detection</span>';
            document.getElementById('cornerDetectionBtn').className = 'responsive-btn bg-red-500 hover:bg-red-600 text-white font-medium py-2 px-4 rounded-lg transition-colors shadow-md hover:shadow-lg flex items-center justify-center gap-2';
            
            closeCornerDetectionModal();
            showMessage(`Corner detection enabled: ${boardType} ${width}x${height}`, 'success');
            logToWeb(`Corner detection started successfully`, 'success');
        } else {
            showMessage(`Failed to enable corner detection: ${data.message}`, 'error');
            logToWeb(`Corner detection failed: ${data.message}`, 'error');
        }
    } catch (error) {
        showMessage(`Network error: ${error.message}`, 'error');
        logToWeb(`Corner detection error: ${error.message}`, 'error');
    }
}

async function disableCornerDetection() {
    try {
        logToWeb('Disabling corner detection...', 'info');
        
        const response = await fetch('/api/calibration/corner-detection', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ enable: false })
        });
        
        const data = await response.json();
        if (data.success) {
            cornerDetectionEnabled = false;
            document.getElementById('cornerDetectionBtn').innerHTML = '<span>🎯</span><span>Corner Detect</span>';
            document.getElementById('cornerDetectionBtn').className = 'responsive-btn bg-yellow-500 hover:bg-yellow-600 text-white font-medium py-2 px-4 rounded-lg transition-colors shadow-md hover:shadow-lg flex items-center justify-center gap-2';
            
            showMessage('Corner detection disabled', 'success');
            logToWeb('Corner detection stopped', 'success');
        } else {
            showMessage(`Failed to disable corner detection: ${data.message}`, 'error');
            logToWeb(`Failed to stop corner detection: ${data.message}`, 'error');
        }
    } catch (error) {
        showMessage(`Network error: ${error.message}`, 'error');
        logToWeb(`Error stopping corner detection: ${error.message}`, 'error');
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
    
    // Initialize Capture x3 button state
    updateCaptureX3ButtonState();
    
    // Load workflow files
    loadWorkflowFiles();
    
    logToWeb('UR15 Web Interface loaded', 'success');
    logToWeb('System ready for operation', 'info');
    logToWeb('Joint angles and TCP pose are now editable - press Enter or click away to move robot', 'info');
};

// Corner Detection Functions
function toggleCornerDetection() {
    if (!cornerDetectionEnabled) {
        showCornerDetectionModal();
    } else {
        disableCornerDetection();
    }
}

function showCornerDetectionModal() {
    const boardType = document.getElementById('boardTypeSelect').value;
    const modal = document.getElementById('cornerDetectionModal');
    
    // Define pattern info
    const patternInfo = {
        'ChessBoard': { icon: '🏁', name: 'ChessBoard', description: 'Standard chessboard pattern with black and white squares' },
        'CharucoBoard': { icon: '🎯', name: 'CharUco Board', description: 'CharUco boards combine chessboard and ArUco markers for robust detection' },
        'GridBoard': { icon: '📐', name: 'ArUco Grid Board', description: 'Grid boards use ArUco markers arranged in a grid pattern' }
    };
    
    const pattern = patternInfo[boardType] || patternInfo['ChessBoard'];
    
    // Update modal title
    const title = document.getElementById('patternModalTitle');
    if (title) {
        title.textContent = `${pattern.icon} ${pattern.name} Settings`;
    }
    
    // Update description
    const description = document.getElementById('patternDescription');
    if (description) {
        description.textContent = pattern.description;
    }
    
    // Generate dynamic parameters
    generatePatternParameters(boardType);
    
    modal.classList.remove('hidden');
}

function generatePatternParameters(boardType) {
    const container = document.getElementById('patternParametersContainer');
    if (!container) return;
    
    container.innerHTML = ''; // Clear existing content
    
    // Define parameter configurations for different pattern types
    const parameterConfigs = {
        'ChessBoard': [
            { id: 'width', label: 'Squares Width', type: 'number', value: 12, min: 3, max: 20, 
              description: 'Number of squares across the width' },
            { id: 'height', label: 'Squares Height', type: 'number', value: 9, min: 3, max: 20, 
              description: 'Number of squares across the height' },
            { id: 'square_size', label: 'Square Size (m)', type: 'number', value: 0.02, min: 0.001, max: 1.0, step: 0.001,
              description: 'Physical size of each square in meters (e.g., 0.02 = 20mm)' }
        ],
        'CharucoBoard': [
            { id: 'width', label: 'Squares Width', type: 'number', value: 12, min: 3, max: 15, 
              description: 'Number of squares across the width' },
            { id: 'height', label: 'Squares Height', type: 'number', value: 9, min: 3, max: 15, 
              description: 'Number of squares across the height' },
            { id: 'square_size', label: 'Square Size (m)', type: 'number', value: 0.03, min: 0.001, max: 1.0, step: 0.001,
              description: 'Physical size of each square in meters (e.g., 0.03 = 30mm)' },
            { id: 'marker_size', label: 'Marker Size (m)', type: 'number', value: 0.0225, min: 0.001, max: 1.0, step: 0.0001,
              description: 'Physical size of ArUco markers in meters (e.g., 0.0225 = 22.5mm)' },
            { id: 'dictionary_id', label: 'ArUco Dictionary', type: 'select', value: 5, 
              options: [
                { value: 0, label: 'DICT_4X4_50' },
                { value: 1, label: 'DICT_4X4_100' },
                { value: 2, label: 'DICT_4X4_250' },
                { value: 3, label: 'DICT_4X4_1000' },
                { value: 4, label: 'DICT_5X5_50' },
                { value: 5, label: 'DICT_5X5_100' },
                { value: 6, label: 'DICT_5X5_250' },
                { value: 7, label: 'DICT_5X5_1000' },
                { value: 8, label: 'DICT_6X6_50' },
                { value: 9, label: 'DICT_6X6_100' },
                { value: 10, label: 'DICT_6X6_250' },
                { value: 11, label: 'DICT_6X6_1000' },
                { value: 12, label: 'DICT_7X7_50' },
                { value: 13, label: 'DICT_7X7_100' },
                { value: 14, label: 'DICT_7X7_250' },
                { value: 15, label: 'DICT_7X7_1000' },
                { value: 16, label: 'DICT_ARUCO_ORIGINAL' }
              ],
              description: 'ArUco dictionary type (DICT_5X5_100 works best for your board)' }
        ],
        'GridBoard': [
            { id: 'width', label: 'Markers Width', type: 'number', value: 6, min: 2, max: 20, 
              description: 'Number of ArUco markers across the width' },
            { id: 'height', label: 'Markers Height', type: 'number', value: 6, min: 2, max: 20, 
              description: 'Number of ArUco markers across the height' },
            { id: 'marker_size', label: 'Marker Size (m)', type: 'number', value: 0.022, min: 0.001, max: 1.0, step: 0.001,
              description: 'Physical size of each marker in meters (e.g., 0.022 = 2.2cm)' },
            { id: 'marker_separation', label: 'Marker Separation (m)', type: 'number', value: 0.0066, min: 0.001, max: 1.0, step: 0.0001,
              description: 'Physical separation between markers in meters (e.g., 0.0066 = 0.66cm)' },
            { id: 'dictionary_id', label: 'ArUco Dictionary', type: 'select', value: 20,
              options: [
                { value: 0, label: 'DICT_4X4_50' },
                { value: 1, label: 'DICT_4X4_100' },
                { value: 2, label: 'DICT_4X4_250' },
                { value: 3, label: 'DICT_4X4_1000' },
                { value: 4, label: 'DICT_5X5_50' },
                { value: 5, label: 'DICT_5X5_100' },
                { value: 6, label: 'DICT_5X5_250' },
                { value: 7, label: 'DICT_5X5_1000' },
                { value: 8, label: 'DICT_6X6_50' },
                { value: 9, label: 'DICT_6X6_100' },
                { value: 10, label: 'DICT_6X6_250' },
                { value: 11, label: 'DICT_6X6_1000' },
                { value: 12, label: 'DICT_7X7_50' },
                { value: 13, label: 'DICT_7X7_100' },
                { value: 14, label: 'DICT_7X7_250' },
                { value: 15, label: 'DICT_7X7_1000' },
                { value: 16, label: 'DICT_ARUCO_ORIGINAL' },
                { value: 17, label: 'DICT_APRILTAG_16H5' },
                { value: 18, label: 'DICT_APRILTAG_25H9' },
                { value: 19, label: 'DICT_APRILTAG_36H10' },
                { value: 20, label: 'DICT_APRILTAG_36H11' },
                { value: 21, label: 'DICT_ARUCO_MIP_36H12' }
              ],
              description: 'ArUco/AprilTag dictionary type (DICT_APRILTAG_36H11 is commonly used)' }
        ]
    };
    
    const params = parameterConfigs[boardType] || parameterConfigs['ChessBoard'];
    
    params.forEach(param => {
        // Create parameter group
        const paramGroup = document.createElement('div');
        paramGroup.className = 'mb-4';
        
        // Label
        const label = document.createElement('label');
        label.className = 'block text-sm font-medium text-gray-700 mb-2';
        label.textContent = param.label + ':';
        
        // Input or Select
        let inputElement;
        if (param.type === 'select') {
            inputElement = document.createElement('select');
            inputElement.className = 'w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-purple-500 bg-white';
            param.options.forEach(option => {
                const optionElement = document.createElement('option');
                optionElement.value = option.value;
                optionElement.textContent = option.label;
                if (option.value === param.value) {
                    optionElement.selected = true;
                }
                inputElement.appendChild(optionElement);
            });
        } else {
            inputElement = document.createElement('input');
            inputElement.type = param.type;
            inputElement.value = param.value;
            if (param.min !== undefined) inputElement.min = param.min;
            if (param.max !== undefined) inputElement.max = param.max;
            if (param.step) inputElement.step = param.step;
            inputElement.className = 'w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-purple-500 bg-white';
        }
        
        inputElement.id = getPatternInputId(param.id);
        
        // Description
        const desc = document.createElement('p');
        desc.className = 'text-xs text-gray-500 mt-1';
        desc.textContent = param.description;
        
        // Assemble parameter group
        paramGroup.appendChild(label);
        paramGroup.appendChild(inputElement);
        paramGroup.appendChild(desc);
        
        container.appendChild(paramGroup);
    });
}

function getPatternInputId(paramId) {
    const idMapping = {
        'width': 'patternWidth',
        'height': 'patternHeight',
        'square_size': 'squareSize',
        'marker_size': 'markerSize',
        'marker_separation': 'markerSeparation',
        'dictionary_id': 'dictionaryId'
    };
    return idMapping[paramId] || paramId;
}

function closeCornerDetectionModal() {
    const modal = document.getElementById('cornerDetectionModal');
    modal.classList.add('hidden');
}

function confirmCornerDetection() {
    const boardType = document.getElementById('boardTypeSelect').value;
    const width = parseInt(document.getElementById('patternWidth').value);
    const height = parseInt(document.getElementById('patternHeight').value);
    
    // Validate basic dimensions
    if (isNaN(width) || isNaN(height) || width <= 1 || height <= 1) {
        showMessage('Invalid dimensions. Width and height must be greater than 1.', 'error', 'Validation Error');
        return;
    }
    
    // Collect pattern-specific parameters
    const patternParams = collectPatternParameters(boardType);
    
    // Validate pattern-specific parameters
    if (!validatePatternParameters(patternParams, boardType)) {
        return; // Validation failed, error already shown
    }
    
    // Enable corner detection
    fetch('/toggle_corner_detection', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            enable: true,
            board_type: boardType,
            width: width,
            height: height,
            ...patternParams
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            cornerDetectionEnabled = true;
            const btn = document.getElementById('cornerDetectionBtn');
            btn.innerHTML = '<span>🛑</span><span>Stop Detection</span>';
            btn.className = 'responsive-btn bg-red-500 hover:bg-red-600 text-white font-medium py-2 px-4 rounded-lg transition-colors shadow-md hover:shadow-lg flex items-center justify-center gap-2';
            
            closeCornerDetectionModal();
            logToWeb(`Corner detection enabled: ${boardType} ${width}x${height}`, 'success');
        } else {
            showMessage('Failed to enable corner detection: ' + data.message, 'error');
            logToWeb(`Failed to enable corner detection: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        console.error('Corner detection error:', error);
        showMessage('Network error: ' + error.message, 'error');
        logToWeb(`Corner detection error: ${error.message}`, 'error');
    });
}

function collectPatternParameters(boardType) {
    const params = {};
    
    // Define which parameters to collect for each pattern type
    const parameterIds = {
        'ChessBoard': ['square_size'],
        'CharucoBoard': ['square_size', 'marker_size', 'dictionary_id'],
        'GridBoard': ['marker_size', 'marker_separation', 'dictionary_id']
    };
    
    const ids = parameterIds[boardType] || [];
    
    ids.forEach(id => {
        const element = document.getElementById(getPatternInputId(id));
        if (element) {
            if (element.tagName === 'SELECT') {
                params[id] = parseInt(element.value);
            } else if (element.type === 'number') {
                params[id] = parseFloat(element.value);
            }
        }
    });
    
    return params;
}

function validatePatternParameters(params, boardType) {
    // ChessBoard validation
    if (boardType === 'ChessBoard') {
        if (params.square_size && params.square_size <= 0) {
            showMessage('Square size must be greater than 0', 'error', 'Validation Error');
            return false;
        }
    }
    
    // CharucoBoard validation
    if (boardType === 'CharucoBoard') {
        if (params.square_size && params.marker_size) {
            if (params.marker_size >= params.square_size) {
                showMessage(
                    `Marker size (${params.marker_size}m) must be smaller than square size (${params.square_size}m). Typically, marker size should be 50-80% of square size.`,
                    'error',
                    'Invalid CharUco Parameters'
                );
                return false;
            }
        }
    }
    
    // GridBoard validation
    if (boardType === 'GridBoard') {
        if (params.marker_size && params.marker_size <= 0) {
            showMessage('Marker size must be greater than 0', 'error', 'Validation Error');
            return false;
        }
        if (params.marker_separation && params.marker_separation < 0) {
            showMessage('Marker separation must be non-negative', 'error', 'Validation Error');
            return false;
        }
    }
    
    return true;
}

function disableCornerDetection() {
    fetch('/toggle_corner_detection', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            enable: false
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            cornerDetectionEnabled = false;
            const btn = document.getElementById('cornerDetectionBtn');
            btn.innerHTML = '<span>🎯</span><span>Corner Detect</span>';
            btn.className = 'responsive-btn bg-purple-500 hover:bg-purple-600 text-white font-medium py-2 px-4 rounded-lg transition-colors shadow-md hover:shadow-lg flex items-center justify-center gap-2';
            
            showMessage('Corner detection disabled', 'info');
            logToWeb('Corner detection disabled', 'info');
        } else {
            showMessage('Failed to disable corner detection: ' + data.message, 'error');
            logToWeb(`Failed to disable corner detection: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        console.error('Corner detection error:', error);
        showMessage('Network error: ' + error.message, 'error');
        logToWeb(`Corner detection error: ${error.message}`, 'error');
    });
}

// Corner Detect Functions (Calibration Panel)
let cornerDetectEnabled = false;

async function toggleCornerDetect() {
    const checkbox = document.getElementById('cornerDetectCheckbox');
    
    if (checkbox.checked) {
        await enableCornerDetect();
    } else {
        await disableCornerDetect();
    }
}

async function enableCornerDetect() {
    const checkbox = document.getElementById('cornerDetectCheckbox');
    checkbox.disabled = true;
    
    try {
        // Read chessboard config path
        const configPath = document.getElementById('chessboardConfigPath').value;
        
        logToWeb('Loading chessboard configuration...', 'info');
        
        // Load chessboard config
        const configResponse = await fetch('/load_chessboard_config', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ config_path: configPath })
        });
        
        const configData = await configResponse.json();
        
        if (!configData.success || !configData.config) {
            showMessage('Failed to load chessboard config', 'error');
            logToWeb('Failed to load chessboard config', 'error');
            checkbox.checked = false;
            checkbox.disabled = false;
            return;
        }
        
        const config = configData.config;
        const patternName = config.name || 'Unknown Pattern';
        
        // Prepare detection parameters with complete JSON config
        let detectionParams = {
            enable: true,
            pattern_json: config
        };
        
        logToWeb(`Enabling corner detection: ${patternName}`, 'info');
        
        // Enable corner detection
        const response = await fetch('/toggle_corner_detection', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(detectionParams)
        });
        
        const data = await response.json();
        
        if (data.success) {
            cornerDetectEnabled = true;
            logToWeb('Draw chessboard corners enabled', 'success');
        } else {
            logToWeb(`Failed to enable corner detection: ${data.message}`, 'error');
            checkbox.checked = false;
        }
        
    } catch (error) {
        logToWeb(`Error enabling corner detection: ${error.message}`, 'error');
        checkbox.checked = false;
    } finally {
        checkbox.disabled = false;
    }
}

async function disableCornerDetect() {
    const checkbox = document.getElementById('cornerDetectCheckbox');
    checkbox.disabled = true;
    
    try {
        logToWeb('Disabling corner detection...', 'info');
        
        const response = await fetch('/toggle_corner_detection', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ enable: false })
        });
        
        const data = await response.json();
        
        if (data.success) {
            cornerDetectEnabled = false;
            logToWeb('Draw chessboard corners stopped', 'success');
        } else {
            logToWeb(`Failed to stop corner detection: ${data.message}`, 'error');
            checkbox.checked = true;
        }
        
    } catch (error) {
        logToWeb(`Error disabling corner detection: ${error.message}`, 'error');
        checkbox.checked = true;
    } finally {
        checkbox.disabled = false;
    }
}

// Draw GB200 Rack Functions
let drawRackEnabled = false;

// Draw Keypoints Functions
let drawKeypointsEnabled = false;

async function toggleDrawRack() {
    const checkbox = document.getElementById('drawRackCheckbox');
    checkbox.disabled = true;
    
    try {
        const response = await fetch('/toggle_draw_rack', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ enable: checkbox.checked })
        });
        
        const data = await response.json();
        
        if (data.success) {
            drawRackEnabled = data.enabled;
            checkbox.checked = data.enabled;
            
            if (data.enabled) {
                logToWeb('Draw GB200 rack enabled', 'success');
            } else {
                logToWeb('Draw GB200 rack disabled', 'info');
            }
        } else {
            logToWeb(`Failed to toggle rack drawing: ${data.message}`, 'error');
            // Revert checkbox on failure
            checkbox.checked = !checkbox.checked;
        }
    } catch (error) {
        logToWeb(`Error toggling rack drawing: ${error.message}`, 'error');
        // Revert checkbox on error
        checkbox.checked = !checkbox.checked;
    } finally {
        checkbox.disabled = false;
    }
}

async function toggleDrawKeypoints() {
    const checkbox = document.getElementById('drawKeypointsCheckbox');
    checkbox.disabled = true;
    
    try {
        const response = await fetch('/toggle_draw_keypoints', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ enable: checkbox.checked })
        });
        
        const data = await response.json();
        
        if (data.success) {
            drawKeypointsEnabled = data.enabled;
            checkbox.checked = data.enabled;
            
            if (data.enabled) {
                logToWeb('Draw keypoints enabled', 'success');
            } else {
                logToWeb('Draw keypoints disabled', 'info');
            }
        } else {
            logToWeb(`Failed to toggle keypoints drawing: ${data.message}`, 'error');
            // Revert checkbox on failure
            checkbox.checked = !checkbox.checked;
        }
    } catch (error) {
        logToWeb(`Error toggling keypoints drawing: ${error.message}`, 'error');
        // Revert checkbox on error
        checkbox.checked = !checkbox.checked;
    } finally {
        checkbox.disabled = false;
    }
}

// Open Eye-in-Hand Calibration Report
function openEyeInHandReport() {
    window.open('/calibration_report/eye_in_hand', '_blank');
}

// Open Intrinsic Calibration Report
function openIntrinsicReport() {
    window.open('/calibration_report/intrinsic', '_blank');
}

// Task Panel Functions
function locateLastOperation() {
    logToWeb('🎯 Locate Last Operation button clicked', 'info');
    
    // Disable button during execution
    const btn = document.getElementById('locateLastOperationBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/locate_last_operation', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        // Re-enable button after 2 seconds
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function locateUnlockKnob() {
    logToWeb('🔓 Locate Unlock Knob button clicked', 'info');
    
    const btn = document.getElementById('locateUnlockKnobBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/locate_unlock_knob', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function executeUnlockKnob() {
    logToWeb('🔧 Execute Unlock Knob button clicked', 'info');
    
    const btn = document.getElementById('executeUnlockKnobBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/execute_unlock_knob', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function locateOpenHandle() {
    logToWeb('🕹️ Locate Open Handle button clicked', 'info');
    
    const btn = document.getElementById('locateOpenHandleBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/locate_open_handle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function executeOpenHandle() {
    logToWeb('👜 Execute Open Handle button clicked', 'info');
    
    const btn = document.getElementById('executeOpenHandleBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/execute_open_handle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function locateCloseLeft() {
    logToWeb('⬅️ Locate Close Left button clicked', 'info');
    
    const btn = document.getElementById('locateCloseLeftBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/locate_close_left', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function executeCloseLeft() {
    logToWeb('◀️ Execute Close Left button clicked', 'info');
    
    const btn = document.getElementById('executeCloseLeftBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/execute_close_left', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function locateCloseRight() {
    logToWeb('➡️ Locate Close Right button clicked', 'info');
    
    const btn = document.getElementById('locateCloseRightBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/locate_close_right', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function executeCloseRight() {
    logToWeb('▶️ Execute Close Right button clicked', 'info');
    
    const btn = document.getElementById('executeCloseRightBtn');
    if (btn) {
        btn.disabled = true;
        btn.classList.add('opacity-50', 'cursor-not-allowed');
    }
    
    fetch('/execute_close_right', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ ${data.message}`, 'success');
        } else {
            logToWeb(`❌ Error: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        logToWeb(`❌ Network error: ${error.message}`, 'error');
    })
    .finally(() => {
        setTimeout(() => {
            if (btn) {
                btn.disabled = false;
                btn.classList.remove('opacity-50', 'cursor-not-allowed');
            }
        }, 2000);
    });
}

function emergencyStop() {
    // Confirm emergency stop action
    if (confirm('⚠️ WARNING: Are you sure you want to trigger EMERGENCY STOP?\n\nThis will immediately halt all robot operations!')) {
        logToWeb('🚨 EMERGENCY STOP TRIGGERED!', 'error');
        
        // Send emergency stop command to robot via Dashboard Server
        fetch('/emergency_stop', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                logToWeb('✅ Emergency stop executed successfully', 'success');
                if (data.response) {
                    logToWeb(`📡 Robot response: ${data.response}`, 'info');
                }
            } else {
                logToWeb(`❌ Emergency stop failed: ${data.message}`, 'error');
            }
        })
        .catch(error => {
            logToWeb(`❌ Network error: ${error.message}`, 'error');
        });
    }
}

function locateUnlockKnob() {
    if (!newLabel || newLabel.trim() === '') {
        logToWeb('⚠️ Label cannot be empty', 'warning');
        // Restore previous value
        document.getElementById(`rackLabel${rackNumber}`).value = rackDisplayNames[rackNumber];
        return;
    }
    
    const oldLabel = rackDisplayNames[rackNumber];
    const oldKey = rackJsonKeys[rackNumber];
    const newKey = newLabel.trim().toLowerCase().replace(/\s+/g, '_');
    
    // Update display name and JSON key
    rackDisplayNames[rackNumber] = newLabel.trim();
    rackJsonKeys[rackNumber] = newKey;
    
    // Notify backend to rename the key in JSON at specific position
    fetch('/rename_rack_key', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            rack_number: rackNumber,
            old_key: oldKey,
            new_key: newKey,
            position: rackNumber - 1  // 0-indexed position in JSON
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ Renamed rack ${rackNumber}: "${oldLabel}" → "${newLabel.trim()}"`, 'success');
        } else {
            logToWeb(`⚠️ Failed to rename rack key: ${data.message}`, 'warning');
            // Rollback on failure
            rackDisplayNames[rackNumber] = oldLabel;
            rackJsonKeys[rackNumber] = oldKey;
            document.getElementById(`rackLabel${rackNumber}`).value = oldLabel;
        }
    })
    .catch(error => {
        logToWeb(`❌ Error renaming rack key: ${error.message}`, 'error');
        // Rollback on error
        rackDisplayNames[rackNumber] = oldLabel;
        rackJsonKeys[rackNumber] = oldKey;
        document.getElementById(`rackLabel${rackNumber}`).value = oldLabel;
    });
}

function selectRack(rackNumber, event) {
    // Stop event propagation to prevent global click handler
    if (event) {
        event.stopPropagation();
    }
    
    // Remove selected class from all rack rows
    document.querySelectorAll('.rack-row').forEach(row => {
        row.classList.remove('selected');
    });
    
    // Add selected class to the clicked rack row
    const selectedRow = document.querySelector(`.rack-row[data-rack="${rackNumber}"]`);
    if (selectedRow) {
        selectedRow.classList.add('selected');
        selectedRack = rackNumber;
        logToWeb(`🎯 Selected ${rackDisplayNames[rackNumber]}`, 'info');
    }
}

// Add global click handler to deselect rack when clicking outside
document.addEventListener('click', function(event) {
    // Check if click is outside any rack row
    if (!event.target.closest('.rack-row') && selectedRack !== null) {
        // Store the name before clearing
        const deselectedName = rackDisplayNames[selectedRack];
        
        // Remove selected class from all rack rows
        document.querySelectorAll('.rack-row').forEach(row => {
            row.classList.remove('selected');
        });
        selectedRack = null;
        logToWeb(`🔵 Deselected ${deselectedName}`, 'info');
    }
});

function recordPosition() {
    if (selectedRack === null) {
        logToWeb('⚠️ Please select a rack first!', 'warning');
        return;
    }
    
    // Directly read current joint positions from the already-updated input fields
    const jointPositions = [];
    let allValid = true;
    
    for (let i = 1; i <= 6; i++) {
        const element = document.getElementById(`joint${i}Value`);
        if (element && element.value !== '--') {
            const value = parseFloat(element.value);
            if (!isNaN(value)) {
                jointPositions.push(value);
            } else {
                allValid = false;
                break;
            }
        } else {
            allValid = false;
            break;
        }
    }
    
    if (!allValid || jointPositions.length !== 6) {
        logToWeb('❌ Joint positions not available yet', 'error');
        return;
    }
    
    // Store the joint positions for the selected rack
    rackPositions[selectedRack] = [...jointPositions];
    
    // Update the display
    for (let i = 0; i < 6; i++) {
        const element = document.getElementById(`rack${selectedRack}_j${i}`);
        if (element) {
            element.textContent = jointPositions[i].toFixed(2);
        }
    }
    
    // Save to JSON file with dynamic key name
    const rackKeyMap = getRackKeyMap();
    fetch('/save_rack_positions', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            rack_number: selectedRack,
            rack_key: rackKeyMap[selectedRack],
            positions: jointPositions
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ Recorded position for ${rackDisplayNames[selectedRack]}: [${jointPositions.map(v => v.toFixed(1)).join(', ')}]° (saved to file)`, 'success');
        } else {
            logToWeb(`⚠️ Position recorded but failed to save: ${data.message}`, 'warning');
        }
    })
    .catch(error => {
        logToWeb(`⚠️ Position recorded but file save error: ${error.message}`, 'warning');
    });
}

function play() {
    logToWeb('▶️ Starting playback sequence for all rack positions...', 'info');
    
    // Load rack positions from JSON file
    fetch('/load_rack_positions')
    .then(response => response.json())
    .then(data => {
        if (!data.success || !data.data) {
            logToWeb('❌ Failed to load rack positions', 'error');
            return;
        }
        
        // Map rack keys dynamically based on current labels
        const rackKeyMap = getRackKeyMap();
        const rackSequence = [
            { key: rackKeyMap[1], name: rackDisplayNames[1] },
            { key: rackKeyMap[2], name: rackDisplayNames[2] },
            { key: rackKeyMap[3], name: rackDisplayNames[3] },
            { key: rackKeyMap[4], name: rackDisplayNames[4] }
        ];
        
        // Convert degrees to radians and prepare positions
        const positions = [];
        for (const rack of rackSequence) {
            if (data.data[rack.key] && Array.isArray(data.data[rack.key]) && data.data[rack.key].length === 6) {
                const degreePositions = data.data[rack.key];
                // Convert degrees to radians
                const radPositions = degreePositions.map(deg => deg * Math.PI / 180.0);
                positions.push({
                    name: rack.name,
                    positions: radPositions
                });
            } else {
                logToWeb(`⚠️ Skipping ${rack.name} - no valid position data`, 'warning');
            }
        }
        
        if (positions.length === 0) {
            logToWeb('❌ No valid positions to execute', 'error');
            return;
        }
        
        // Execute movements sequentially
        executeSequentialMovej(positions, 0);
    })
    .catch(error => {
        logToWeb(`❌ Error loading positions: ${error.message}`, 'error');
    });
}

// Helper function to execute movej commands sequentially
function executeSequentialMovej(positions, index) {
    if (index >= positions.length) {
        logToWeb('✅ Playback sequence completed!', 'success');
        return;
    }
    
    const current = positions[index];
    logToWeb(`📍 Moving to ${current.name}...`, 'info');
    
    // Send movej command with blocking=true for sequential execution
    fetch('/movej', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            joint_positions: current.positions,
            blocking: true  // Use blocking mode to wait for movement completion
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`✅ Reached ${current.name}`, 'success');
            // Wait 2 seconds before moving to next position
            setTimeout(() => {
                executeSequentialMovej(positions, index + 1);
            }, 2000);
        } else {
            logToWeb(`❌ Failed to move to ${current.name}: ${data.message}`, 'error');
            // Continue to next position even if one fails (no delay on failure)
            executeSequentialMovej(positions, index + 1);
        }
    })
    .catch(error => {
        logToWeb(`❌ Error moving to ${current.name}: ${error.message}`, 'error');
        // Continue to next position even if one fails (no delay on error)
        executeSequentialMovej(positions, index + 1);
    });
}

function deleteRackPosition(rackNumber) {
    // Clear the position from memory
    rackPositions[rackNumber] = null;
    
    // Clear the display
    for (let i = 0; i < 6; i++) {
        const element = document.getElementById(`rack${rackNumber}_j${i}`);
        if (element) {
            element.textContent = '0.00';
        }
    }
    
    // Save to JSON file (with all zeros)
    const zeroPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    fetch('/save_rack_positions', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            rack_number: rackNumber,
            positions: zeroPositions
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logToWeb(`🗑️ Cleared position for ${rackDisplayNames[rackNumber]}`, 'info');
        } else {
            logToWeb(`⚠️ Position cleared but failed to save: ${data.message}`, 'warning');
        }
    })
    .catch(error => {
        logToWeb(`⚠️ Position cleared but file save error: ${error.message}`, 'warning');
    });
}