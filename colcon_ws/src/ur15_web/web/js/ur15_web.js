// UR15 Web Interface JavaScript

let freedriveActive = false;
let validationActive = false;

function showMessage(message, type = 'info') {
    const modal = document.getElementById('messageModal');
    const text = document.getElementById('messageText');
    
    text.textContent = message;
    modal.classList.remove('hidden');
    modal.style.display = 'flex';
    
    setTimeout(() => {
        modal.classList.add('hidden');
        modal.style.display = 'none';
    }, 3000);
}

function changeDataDir() {
    const currentDir = document.getElementById('statusBarDataDir').textContent;
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
    const currentDir = document.getElementById('statusBarDataDir').textContent;
    const input = document.getElementById('dataDirInput');
    const newDir = input.value.trim();
    
    if (newDir === '' || newDir === currentDir) {
        closeDataDirModal();
        return;
    }
    
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
            document.getElementById('statusBarDataDir').textContent = data.data_dir;
            showMessage('Dataset directory changed successfully', 'success');
            closeDataDirModal();
        } else {
            showMessage('Failed to change directory: ' + (data.message || 'Unknown error'), 'error');
        }
    })
    .catch(error => {
        console.error('Failed to change data directory:', error);
        showMessage('Failed to change directory: ' + error.message, 'error');
    });
}

function toggleFreedrive() {
    // Disable the mode card temporarily
    const modeCard = document.querySelector('.status-bar-item.clickable');
    modeCard.style.pointerEvents = 'none';
    modeCard.style.opacity = '0.7';
    
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
        }
        // Re-enable the mode card
        modeCard.style.pointerEvents = 'auto';
        modeCard.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to toggle freedrive:', error);
        // Re-enable the mode card
        modeCard.style.pointerEvents = 'auto';
        modeCard.style.opacity = '1';
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
    
    const formData = new FormData();
    formData.append('file', file);
    
    fetch('/upload_intrinsic', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            updateStatus();
        } else {
            console.error('Failed to load intrinsic parameters:', data.message);
        }
    })
    .catch(error => {
        console.error('Upload failed:', error);
    });
}

function uploadExtrinsic() {
    const fileInput = document.getElementById('extrinsicFile');
    const file = fileInput.files[0];
    
    if (!file) {
        return;
    }
    
    const formData = new FormData();
    formData.append('file', file);
    
    fetch('/upload_extrinsic', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            updateStatus();
        } else {
            console.error('Failed to load extrinsic parameters:', data.message);
        }
    })
    .catch(error => {
        console.error('Upload failed:', error);
    });
}

function takeScreenshot() {
    const btn = document.getElementById('screenshotBtn');
    btn.disabled = true;
    btn.style.opacity = '0.7';
    
    fetch('/take_screenshot', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (!data.success) {
            console.error('Failed to take screenshot:', data.message);
        }
        btn.disabled = false;
        btn.style.opacity = '1';
    })
    .catch(error => {
        console.error('Failed to take screenshot:', error);
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
        
        // Update data dir in status bar
        if (data.data_dir) {
            document.getElementById('statusBarDataDir').textContent = data.data_dir;
        }
        
        // Update joint positions
        const jointPositionsElement = document.getElementById('jointPositions');
        if (data.joint_positions && data.joint_positions.length > 0) {
            const jointText = data.joint_positions.map((pos, idx) => 
                `J${idx + 1}: ${pos.toFixed(2)}°`
            ).join('\n');
            jointPositionsElement.textContent = jointText;
            jointPositionsElement.style.color = '#212529';
        } else {
            jointPositionsElement.textContent = 'Waiting for data...';
            jointPositionsElement.style.color = '#6c757d';
        }
        
        // Update TCP pose
        const tcpPoseElement = document.getElementById('tcpPoseDisplay');
        if (data.tcp_pose) {
            const pose = data.tcp_pose;
            const poseText = `Position (mm):\n  X: ${pose.x.toFixed(2)}\n  Y: ${pose.y.toFixed(2)}\n  Z: ${pose.z.toFixed(2)}\n\nOrientation (quaternion):\n  X: ${pose.qx.toFixed(4)}\n  Y: ${pose.qy.toFixed(4)}\n  Z: ${pose.qz.toFixed(4)}\n  W: ${pose.qw.toFixed(4)}`;
            tcpPoseElement.textContent = poseText;
            tcpPoseElement.style.color = '#212529';
        } else {
            tcpPoseElement.textContent = 'Waiting for data...';
            tcpPoseElement.style.color = '#6c757d';
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

// Update status every 500ms
setInterval(() => {
    updateStatus();
}, 500);

// Update status on page load
window.onload = function() {
    updateStatus();
};