// Workflow Config Center JavaScript Functions

// Load workflow templates
let workflowTemplates = null;

// Check if workflow is loaded and update UI state
function checkWorkflowLoaded() {
    const editor = document.getElementById('jsonEditor');
    const hasWorkflow = editor && editor.value.trim() !== '';
    
    // Update handler items draggable state
    const handlerItems = document.querySelectorAll('.handler-item');
    handlerItems.forEach(item => {
        if (hasWorkflow) {
            item.setAttribute('draggable', 'true');
            item.style.opacity = '1';
            item.style.cursor = 'move';
            item.title = '';
        } else {
            item.setAttribute('draggable', 'false');
            item.style.opacity = '0.5';
            item.style.cursor = 'not-allowed';
            item.title = 'Please create or load a workflow first';
        }
    });
    
    // Update editor and modular view drop zones
    const jsonEditor = document.getElementById('jsonEditor');
    const modularView = document.getElementById('workflowModularView');
    
    if (hasWorkflow) {
        if (jsonEditor) {
            jsonEditor.style.pointerEvents = 'auto';
        }
        if (modularView) {
            modularView.style.pointerEvents = 'auto';
        }
    } else {
        if (jsonEditor) {
            jsonEditor.style.pointerEvents = 'auto'; // Allow typing
        }
        if (modularView) {
            modularView.style.pointerEvents = 'none';
        }
    }
    
    return hasWorkflow;
}

async function loadWorkflowTemplates() {
    try {
        console.log('Loading workflow templates from: template/workflow_template.json');
        const response = await fetch('template/workflow_template.json');
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        workflowTemplates = data;
        console.log('Workflow templates loaded successfully:', workflowTemplates);
        console.log('Available template types:', Object.keys(workflowTemplates));
    } catch (error) {
        console.error('Failed to load workflow templates:', error);
        console.error('Error details:', error.message);
        // Fallback to empty templates
        workflowTemplates = {
            robot_move: [],
            capture_image: [],
            record_data: [],
            positioning: []
        };
        console.warn('Using empty fallback templates');
    }
}

// Get template for a specific handler
function getHandlerTemplate(handlerType, handlerId) {
    console.log('Getting template for:', handlerType, handlerId);
    console.log('Current workflowTemplates:', workflowTemplates);
    
    if (!workflowTemplates) {
        console.error('workflowTemplates is null or undefined!');
        return null;
    }
    
    if (!workflowTemplates[handlerType]) {
        console.error('Handler type not found in templates:', handlerType);
        console.log('Available types:', Object.keys(workflowTemplates));
        return null;
    }
    
    const templates = workflowTemplates[handlerType];
    console.log('Templates for type', handlerType, ':', templates);
    
    const template = templates.find(t => t.id === handlerId);
    
    if (template) {
        console.log('Found template:', template);
        // Return a deep copy of the template
        return JSON.parse(JSON.stringify(template));
    }
    
    console.error('Handler template not found:', handlerType, handlerId);
    console.log('Available handler IDs:', templates.map(t => t.id));
    return null;
}

function showLoadWorkflowModal() {
    // 获取工作流文件列表
    fetch('http://localhost:8008/api/workflows')
        .then(response => response.json())
        .then(data => {
            const select = document.getElementById('workflowFileList');
            if (data.success && data.files.length > 0) {
                select.innerHTML = '';
                data.files.forEach(file => {
                    const option = document.createElement('option');
                    option.value = file;
                    option.textContent = file;
                    select.appendChild(option);
                });
            } else {
                select.innerHTML = '<option value="">No workflow files found</option>';
            }
            document.getElementById('loadWorkflowModal').style.display = 'block';
        })
        .catch(error => {
            console.error('Error loading workflow list:', error);
            showErrorModal('Failed to load workflow list: ' + error.message);
        });
}

function closeLoadWorkflowModal() {
    document.getElementById('loadWorkflowModal').style.display = 'none';
}

function loadSelectedWorkflow() {
    const select = document.getElementById('workflowFileList');
    const filename = select.value;
    
    if (!filename) {
        return;
    }
    
    // 从服务器加载工作流文件
    fetch(`http://localhost:8008/api/workflow/${filename}`)
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                // 更新编辑器内容
                const editor = document.getElementById('jsonEditor');
                if (editor) {
                    editor.value = JSON.stringify(data.content, null, 2);
                }
                
                // 更新文件名显示
                const filenameDisplay = document.getElementById('workflowFilename');
                if (filenameDisplay) {
                    filenameDisplay.textContent = data.filename;
                }
                
                closeLoadWorkflowModal();
                console.log('Loaded workflow:', data.filename);
                checkWorkflowLoaded(); // Update drag state after loading
            } else {
                showErrorModal('Failed to load workflow: ' + data.error);
            }
        })
        .catch(error => {
            console.error('Error loading workflow:', error);
            showErrorModal('Failed to load workflow: ' + error.message);
        });
}

function showNewWorkflowModal() {
    document.getElementById('newWorkflowModal').style.display = 'block';
}

function closeNewWorkflowModal() {
    document.getElementById('newWorkflowModal').style.display = 'none';
}

function createNewWorkflow() {
    const fileName = document.getElementById('workflowFileName').value;
    if (!fileName) {
        return;
    }
    
    // 确保文件名以.json结尾
    const finalFileName = fileName.endsWith('.json') ? fileName : fileName + '.json';
    
    // 创建空的workflow配置
    const emptyWorkflow = {
        "context": {
            "robot_ip": "192.168.1.15",
            "robot_port": 30002,
            "camera_topic": "/ur15_camera/image_raw",
            "positioning_service_url": "http://localhost:8004"
        },
        "workflow": []
    };
    
    // 更新编辑器内容
    const editor = document.getElementById('jsonEditor');
    if (editor) {
        editor.value = JSON.stringify(emptyWorkflow, null, 2);
    }
    
    // 更新文件名显示
    const filenameDisplay = document.getElementById('workflowFilename');
    if (filenameDisplay) {
        filenameDisplay.textContent = finalFileName;
    }
    
    closeNewWorkflowModal();
    checkWorkflowLoaded(); // Update drag state after creating new workflow
    
    // 调用API创建文件
    fetch('http://localhost:8008/api/workflow', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ fileName: finalFileName, content: emptyWorkflow })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            console.log('Workflow created:', data.message);
        } else {
            console.error('Failed to create workflow:', data.error);
        }
    })
    .catch(error => {
        console.error('Error creating workflow:', error);
    });
}

function deleteWorkflow() {
    const filenameDisplay = document.getElementById('workflowFilename');
    const currentFilename = filenameDisplay ? filenameDisplay.textContent : '';
    
    if (!currentFilename || currentFilename === 'No file loaded') {
        return;
    }
    
    // 显示删除确认模态框
    const deleteFileNameSpan = document.getElementById('deleteFileName');
    if (deleteFileNameSpan) {
        deleteFileNameSpan.textContent = currentFilename;
    }
    
    document.getElementById('deleteWorkflowModal').style.display = 'block';
}

function closeDeleteWorkflowModal() {
    document.getElementById('deleteWorkflowModal').style.display = 'none';
}

function confirmDeleteWorkflow() {
    const filenameDisplay = document.getElementById('workflowFilename');
    const currentFilename = filenameDisplay ? filenameDisplay.textContent : '';
    
    closeDeleteWorkflowModal();
    
    // 调用API删除文件
    fetch(`http://localhost:8008/api/workflow/${currentFilename}`, {
        method: 'DELETE'
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            console.log('Workflow deleted:', data.message);
            
            // 清空编辑器
            const editor = document.getElementById('jsonEditor');
            if (editor) {
                editor.value = '';
            }
            
            // 重置文件名显示
            if (filenameDisplay) {
                filenameDisplay.textContent = 'No file loaded';
            }
            
            checkWorkflowLoaded(); // Update drag state after deleting
        } else {
            showErrorModal('Delete failed: ' + data.error);
        }
    })
    .catch(error => {
        console.error('Error deleting workflow:', error);
        showErrorModal('Delete failed: ' + error.message);
    });
}

function saveWorkflowToFile() {
    const editor = document.getElementById('jsonEditor');
    const filenameDisplay = document.getElementById('workflowFilename');
    
    if (!editor || !editor.value) {
        return;
    }
    
    try {
        // 验证 JSON 格式
        const jsonData = JSON.parse(editor.value);
        
        // 获取文件名
        let filename = filenameDisplay ? filenameDisplay.textContent : 'workflow.json';
        if (filename === 'No file loaded') {
            filename = 'workflow.json';
        }
        
        // 调用API保存文件
        fetch('http://localhost:8008/api/workflow', {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ fileName: filename, content: jsonData })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                console.log('Workflow saved:', data.message);
                showSuccessModal('Workflow saved: ' + filename);
            } else {
                showErrorModal('Save failed: ' + data.error);
            }
        })
        .catch(error => {
            console.error('Error saving workflow:', error);
            showErrorModal('Save failed: ' + error.message);
        });
        
    } catch (error) {
        showErrorModal('Invalid JSON format: ' + error.message);
    }
}

function clearWorkflow() {
    const filenameDisplay = document.getElementById('workflowFilename');
    const currentFilename = filenameDisplay ? filenameDisplay.textContent : '';
    
    if (!currentFilename || currentFilename === 'No file loaded') {
        return;
    }
    
    // 显示清空确认模态框
    document.getElementById('clearWorkflowModal').style.display = 'block';
}

function closeClearWorkflowModal() {
    document.getElementById('clearWorkflowModal').style.display = 'none';
}

function confirmClearWorkflow() {
    const editor = document.getElementById('jsonEditor');
    
    if (editor && editor.value) {
        try {
            const jsonData = JSON.parse(editor.value);
            
            // 清空 workflow 数组，保留 context
            if (jsonData.workflow) {
                jsonData.workflow = [];
            }
            
            // 更新编辑器内容
            editor.value = JSON.stringify(jsonData, null, 2);
            checkWorkflowLoaded(); // Update drag state after clearing
            
            // 更新 Modular View
            renderModularView();
            
            console.log('Workflow cleared, context preserved');
        } catch (error) {
            console.error('Failed to parse JSON:', error);
        }
    }
    
    closeClearWorkflowModal();
}

function closeWorkflow() {
    const filenameDisplay = document.getElementById('workflowFilename');
    const currentFilename = filenameDisplay ? filenameDisplay.textContent : '';
    
    if (!currentFilename || currentFilename === 'No file loaded') {
        showInfoModal('No workflow is currently open.');
        return;
    }
    
    // 显示关闭确认模态框
    const closeFileNameSpan = document.getElementById('closeFileName');
    if (closeFileNameSpan) {
        closeFileNameSpan.textContent = currentFilename;
    }
    
    document.getElementById('closeWorkflowModal').style.display = 'block';
}

function closeCloseWorkflowModal() {
    document.getElementById('closeWorkflowModal').style.display = 'none';
}

function confirmCloseWorkflow() {
    const filenameDisplay = document.getElementById('workflowFilename');
    const currentFilename = filenameDisplay ? filenameDisplay.textContent : '';
    
    closeCloseWorkflowModal();
    
    // 清空编辑器
    const editor = document.getElementById('jsonEditor');
    if (editor) {
        editor.value = '';
    }
    
    // 重置文件名显示
    if (filenameDisplay) {
        filenameDisplay.textContent = 'No file loaded';
    }
    
    checkWorkflowLoaded(); // Update drag state after closing
    
    console.log('Workflow closed:', currentFilename);
}

function showSuccessModal(message) {
    const messageElement = document.getElementById('successMessage');
    if (messageElement) {
        messageElement.textContent = message;
    }
    document.getElementById('successModal').style.display = 'block';
}

function closeSuccessModal() {
    document.getElementById('successModal').style.display = 'none';
}

function showErrorModal(message) {
    const messageElement = document.getElementById('errorMessage');
    if (messageElement) {
        messageElement.textContent = message;
    }
    document.getElementById('errorModal').style.display = 'block';
}

function closeErrorModal() {
    document.getElementById('errorModal').style.display = 'none';
}

function showInfoModal(message) {
    const messageElement = document.getElementById('infoMessage');
    if (messageElement) {
        messageElement.textContent = message;
    }
    document.getElementById('infoModal').style.display = 'block';
}

function closeInfoModal() {
    document.getElementById('infoModal').style.display = 'none';
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', async function() {
    // Load workflow templates first
    await loadWorkflowTemplates();
    
    // Get all handler items
    const handlerItems = document.querySelectorAll('.handler-item');
    const jsonEditor = document.getElementById('jsonEditor');
    const modularView = document.getElementById('workflowModularView');
    
    // Add drag event listeners to handler items
    handlerItems.forEach(item => {
        item.addEventListener('dragstart', handleDragStart);
        item.addEventListener('dragend', handleDragEnd);
    });
    
    // Add drop event listeners to JSON editor
    if (jsonEditor) {
        jsonEditor.addEventListener('dragover', handleDragOver);
        jsonEditor.addEventListener('dragleave', handleDragLeave);
        jsonEditor.addEventListener('drop', handleDrop);
        
        // Monitor editor changes to update drag state
        jsonEditor.addEventListener('input', checkWorkflowLoaded);
    }
    
    // Add drop event listeners to modular view
    if (modularView) {
        modularView.addEventListener('dragover', handleModularDragOver);
        modularView.addEventListener('dragleave', handleModularDragLeave);
        modularView.addEventListener('drop', handleModularDrop);
    }
    
    // Initialize workflow state
    checkWorkflowLoaded();
});

function handleDragStart(e) {
    // Check if workflow is loaded
    if (!checkWorkflowLoaded()) {
        e.preventDefault();
        showInfoModal('Please create or load a workflow first before dragging handlers.');
        return false;
    }
    
    e.target.classList.add('dragging');
    const handlerId = e.target.dataset.handlerId;
    const handlerType = e.target.dataset.handlerType;
    
    // Store handler info in dataTransfer
    e.dataTransfer.effectAllowed = 'copy';
    e.dataTransfer.setData('text/plain', JSON.stringify({
        handler_id: handlerId,
        handler_type: handlerType
    }));
}

function handleDragEnd(e) {
    e.target.classList.remove('dragging');
    
    // Clear throttle timeout
    if (dragOverThrottle) {
        clearTimeout(dragOverThrottle);
        dragOverThrottle = null;
    }
    
    // Remove drop indicator when drag ends
    const existingIndicator = document.querySelector('.drop-indicator');
    if (existingIndicator) {
        existingIndicator.remove();
    }
}

function handleDragOver(e) {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'copy';
    e.target.classList.add('drag-over');
}

function handleDragLeave(e) {
    e.target.classList.remove('drag-over');
}

function handleDrop(e) {
    e.preventDefault();
    e.target.classList.remove('drag-over');
    
    try {
        const data = JSON.parse(e.dataTransfer.getData('text/plain'));
        
        // Get template for this handler
        const template = getHandlerTemplate(data.handler_type, data.handler_id);
        
        if (!template) {
            showErrorModal('Template not found for handler: ' + data.handler_id + '. Please make sure workflow_template.json is loaded.');
            return;
        }
        
        // Use the complete template as the new step
        const newStep = template;
        
        // Get current JSON content
        const editor = document.getElementById('jsonEditor');
        let workflowData;
        
        if (editor.value.trim()) {
            try {
                workflowData = JSON.parse(editor.value);
            } catch (e) {
                showErrorModal('Invalid JSON format in editor. Please fix it first.');
                return;
            }
        } else {
            // Create default workflow structure
            workflowData = {
                context: {
                    robot_ip: "192.168.1.15",
                    robot_port: 30002,
                    use_real_robot: true
                },
                workflow: []
            };
        }
        
        // Ensure workflow array exists
        if (!workflowData.workflow) {
            workflowData.workflow = [];
        }
        
        // Add new step to workflow
        workflowData.workflow.push(newStep);
        
        // Update editor with formatted JSON
        editor.value = JSON.stringify(workflowData, null, 2);
        
        // Update modular view if active
        if (isModularViewActive()) {
            renderModularView();
        }
        
        console.log('Handler added to workflow:', data.handler_id);
        
    } catch (error) {
        console.error('Error adding handler to workflow:', error);
        showErrorModal('Failed to add handler: ' + error.message);
    }
}

// Modular view drag and drop handlers
let dragOverThrottle = null;

function handleModularDragOver(e) {
    e.preventDefault();
    
    // Don't process if dragging from within modular view (reordering)
    if (e.dataTransfer.effectAllowed === 'move') {
        return;
    }
    
    e.dataTransfer.dropEffect = 'copy';
    
    // Throttle the indicator update to improve performance
    if (dragOverThrottle) {
        clearTimeout(dragOverThrottle);
    }
    
    dragOverThrottle = setTimeout(() => {
        updateDropIndicator(e.clientY);
    }, 10);
}

function updateDropIndicator(clientY) {
    console.log('updateDropIndicator called with clientY:', clientY);
    
    const modularView = document.getElementById('workflowModularView');
    const steps = modularView.querySelectorAll('.workflow-step:not(.dragging)');
    
    console.log('Number of steps:', steps.length);
    
    // Remove existing indicator
    const existingIndicator = document.querySelector('.drop-indicator');
    if (existingIndicator) {
        existingIndicator.remove();
    }
    
    // Create the indicator
    const indicator = document.createElement('div');
    indicator.className = 'drop-indicator';
    indicator.style.cssText = `
        height: 4px;
        background: linear-gradient(90deg, #3b82f6, #8b5cf6);
        margin: 8px 0;
        border-radius: 2px;
        box-shadow: 0 0 12px rgba(59, 130, 246, 0.8);
        animation: pulse 1s ease-in-out infinite;
        pointer-events: none;
        width: 100%;
        flex-shrink: 0;
    `;
    
    // Find where to show the indicator
    if (steps.length > 0) {
        let insertBeforeStep = null;
        
        for (let i = 0; i < steps.length; i++) {
            const step = steps[i];
            const rect = step.getBoundingClientRect();
            const midpoint = rect.top + rect.height / 2;
            
            if (clientY < midpoint) {
                insertBeforeStep = step;
                break;
            }
        }
        
        if (insertBeforeStep) {
            console.log('Inserting indicator before step:', insertBeforeStep.dataset.index);
            modularView.insertBefore(indicator, insertBeforeStep);
        } else {
            console.log('Inserting indicator at the end');
            modularView.appendChild(indicator);
        }
    } else {
        console.log('No steps, inserting indicator in empty space');
        modularView.appendChild(indicator);
    }
    
    console.log('Indicator inserted:', document.querySelector('.drop-indicator'));
}

function handleModularDragLeave(e) {
    // Only clear when actually leaving the modular view, not when entering child elements
    const modularView = document.getElementById('workflowModularView');
    const relatedTarget = e.relatedTarget;
    
    // Check if we're leaving to outside the modular view
    if (!modularView.contains(relatedTarget)) {
        e.currentTarget.style.background = '';
        
        // Clear throttle timeout
        if (dragOverThrottle) {
            clearTimeout(dragOverThrottle);
            dragOverThrottle = null;
        }
        
        // Remove drop indicator when leaving the container
        const existingIndicator = document.querySelector('.drop-indicator');
        if (existingIndicator) {
            existingIndicator.remove();
        }
    }
}

function handleModularDrop(e) {
    e.preventDefault();
    e.currentTarget.style.background = '';
    
    // Clear throttle timeout
    if (dragOverThrottle) {
        clearTimeout(dragOverThrottle);
        dragOverThrottle = null;
    }
    
    // Remove drop indicator if exists
    const existingIndicator = document.querySelector('.drop-indicator');
    if (existingIndicator) {
        existingIndicator.remove();
    }
    
    try {
        const data = JSON.parse(e.dataTransfer.getData('text/plain'));
        
        // Get template for this handler
        const template = getHandlerTemplate(data.handler_type, data.handler_id);
        
        if (!template) {
            showErrorModal('Template not found for handler: ' + data.handler_id + '. Please make sure workflow_template.json is loaded.');
            return;
        }
        
        // Use the complete template as the new step
        const newStep = template;
        
        // Get current JSON content
        const editor = document.getElementById('jsonEditor');
        let workflowData;
        
        if (editor.value.trim()) {
            try {
                workflowData = JSON.parse(editor.value);
            } catch (error) {
                showErrorModal('Invalid JSON format in editor. Please fix it first.');
                return;
            }
        } else {
            // Create default workflow structure
            workflowData = {
                context: {
                    robot_ip: "192.168.1.15",
                    robot_port: 30002,
                    use_real_robot: true
                },
                workflow: []
            };
        }
        
        // Ensure workflow array exists
        if (!workflowData.workflow) {
            workflowData.workflow = [];
        }
        
        // Find the drop position
        let insertIndex = workflowData.workflow.length; // Default to end
        const steps = document.querySelectorAll('.workflow-step');
        
        if (steps.length > 0) {
            for (let i = 0; i < steps.length; i++) {
                const step = steps[i];
                const rect = step.getBoundingClientRect();
                const midpoint = rect.top + rect.height / 2;
                
                // If mouse is above the midpoint of this step, insert before it
                if (e.clientY < midpoint) {
                    insertIndex = i;
                    break;
                }
            }
        }
        
        // Insert new step at the determined position
        workflowData.workflow.splice(insertIndex, 0, newStep);
        
        // Update editor with formatted JSON
        editor.value = JSON.stringify(workflowData, null, 2);
        
        // Update modular view
        renderModularView();
        
        console.log('Handler added to workflow at position', insertIndex, ':', data.handler_id);
        
    } catch (error) {
        console.error('Error adding handler to modular view:', error);
        showErrorModal('Failed to add handler: ' + error.message);
    }
}

// View toggle functionality
let currentView = 'json'; // 'json' or 'modular'

function toggleWorkflowView() {
    const jsonEditor = document.getElementById('jsonEditor');
    const modularView = document.getElementById('workflowModularView');
    const toggleBtn = document.getElementById('viewToggleBtn');
    const toggleIcon = document.getElementById('viewToggleIcon');
    const toggleText = document.getElementById('viewToggleText');
    
    if (currentView === 'json') {
        // Switch to modular view
        currentView = 'modular';
        jsonEditor.style.display = 'none';
        modularView.classList.add('active');
        modularView.style.pointerEvents = 'auto'; // Enable drag and drop
        toggleIcon.textContent = '{}';
        toggleText.textContent = 'JSON View';
        renderModularView();
    } else {
        // Switch to JSON view
        currentView = 'json';
        jsonEditor.style.display = 'block';
        modularView.classList.remove('active');
        toggleIcon.textContent = '📋';
        toggleText.textContent = 'Modular View';
        syncFromModularToJson();
    }
}

function isModularViewActive() {
    return currentView === 'modular';
}

function renderModularView() {
    const modularView = document.getElementById('workflowModularView');
    const editor = document.getElementById('jsonEditor');
    
    try {
        let workflowData;
        if (editor.value.trim()) {
            workflowData = JSON.parse(editor.value);
        } else {
            modularView.innerHTML = `
                <div class="workflow-empty">
                    <div class="workflow-empty-icon">📦</div>
                    <div style="font-size: 1.1rem; font-weight: 600; margin-bottom: 0.5rem;">No Workflow Steps</div>
                    <div style="font-size: 0.9rem;">Drag handlers from the left panel to start building your workflow</div>
                </div>
            `;
            return;
        }
        
        const workflow = workflowData.workflow || [];
        
        if (workflow.length === 0) {
            modularView.innerHTML = `
                <div class="workflow-empty">
                    <div class="workflow-empty-icon">📦</div>
                    <div style="font-size: 1.1rem; font-weight: 600; margin-bottom: 0.5rem;">No Workflow Steps</div>
                    <div style="font-size: 0.9rem;">Drag handlers from the left panel to start building your workflow</div>
                </div>
            `;
            return;
        }
        
        // Render workflow steps
        modularView.innerHTML = workflow.map((step, index) => {
            console.log('Rendering step', index, ':', step);
            
            // Support both old format (handler_id, handler_type) and new format (id, type)
            const handlerId = step.id || step.handler_id || 'Unknown';
            const handlerType = step.type || step.handler_type || 'Unknown';
            const description = step.description || '';
            
            console.log('Extracted values - id:', handlerId, 'type:', handlerType, 'description:', description);
            
            // For display, exclude id, type, and description from params
            const displayParams = {...step};
            delete displayParams.id;
            delete displayParams.type;
            delete displayParams.description;
            
            return `
            <div class="workflow-step" data-index="${index}" draggable="true">
                <div class="workflow-step-header">
                    <div class="workflow-step-number">${index + 1}</div>
                    <div class="workflow-step-info">
                        <div class="workflow-step-handler">${handlerId}</div>
                        <span class="workflow-step-type">${handlerType}</span>
                        ${description ? `<div style="font-size: 0.75rem; color: #6b7280; margin-top: 0.25rem;">${description}</div>` : ''}
                    </div>
                    <div class="workflow-step-actions">
                        <button class="workflow-step-btn" onclick="moveStepUp(${index})" title="Move up" ${index === 0 ? 'disabled' : ''}>⬆️</button>
                        <button class="workflow-step-btn" onclick="moveStepDown(${index})" title="Move down" ${index === workflow.length - 1 ? 'disabled' : ''}>⬇️</button>
                        <button class="workflow-step-btn" onclick="editStepParams(${index})" title="Edit parameters">✏️</button>
                        <button class="workflow-step-btn" onclick="deleteStep(${index})" title="Delete">🗑️</button>
                    </div>
                </div>
                <div class="workflow-step-params">${JSON.stringify(displayParams, null, 2)}</div>
            </div>
        `}).join('');
        
        // Add drag and drop for reordering
        addStepDragListeners();
        
    } catch (error) {
        console.error('Error rendering modular view:', error);
        modularView.innerHTML = `
            <div class="workflow-empty">
                <div class="workflow-empty-icon">⚠️</div>
                <div style="font-size: 1.1rem; font-weight: 600; margin-bottom: 0.5rem; color: #ef4444;">Error Parsing Workflow</div>
                <div style="font-size: 0.9rem;">${error.message}</div>
            </div>
        `;
    }
}

function syncFromModularToJson() {
    // Already synced via operations
}

function moveStepUp(index) {
    const editor = document.getElementById('jsonEditor');
    try {
        const workflowData = JSON.parse(editor.value);
        if (index > 0 && workflowData.workflow) {
            const temp = workflowData.workflow[index];
            workflowData.workflow[index] = workflowData.workflow[index - 1];
            workflowData.workflow[index - 1] = temp;
            editor.value = JSON.stringify(workflowData, null, 2);
            renderModularView();
        }
    } catch (error) {
        console.error('Error moving step up:', error);
    }
}

function moveStepDown(index) {
    const editor = document.getElementById('jsonEditor');
    try {
        const workflowData = JSON.parse(editor.value);
        if (index < workflowData.workflow.length - 1 && workflowData.workflow) {
            const temp = workflowData.workflow[index];
            workflowData.workflow[index] = workflowData.workflow[index + 1];
            workflowData.workflow[index + 1] = temp;
            editor.value = JSON.stringify(workflowData, null, 2);
            renderModularView();
        }
    } catch (error) {
        console.error('Error moving step down:', error);
    }
}

let currentEditingStepIndex = null;

function editStepParams(index) {
    const editor = document.getElementById('jsonEditor');
    try {
        const workflowData = JSON.parse(editor.value);
        const step = workflowData.workflow[index];
        
        // Store the index being edited
        currentEditingStepIndex = index;
        
        // Set handler name
        const handlerName = step.id || step.handler_id || 'Unknown';
        document.getElementById('editParamsHandlerName').textContent = handlerName;
        
        // Extract parameters for editing
        // Exclude id, type, and description which are metadata
        const paramsToEdit = {...step};
        delete paramsToEdit.id;
        delete paramsToEdit.type;
        delete paramsToEdit.description;
        delete paramsToEdit.handler_id;
        delete paramsToEdit.handler_type;
        
        // Set parameters in textarea
        const paramsEditor = document.getElementById('paramsEditor');
        paramsEditor.value = JSON.stringify(paramsToEdit, null, 2);
        
        // Clear error message
        const errorMsg = document.getElementById('paramsErrorMessage');
        errorMsg.style.display = 'none';
        errorMsg.textContent = '';
        
        // Show modal
        document.getElementById('editParamsModal').style.display = 'block';
    } catch (error) {
        console.error('Error opening edit params modal:', error);
        showErrorModal('Failed to open editor: ' + error.message);
    }
}

function closeEditParamsModal() {
    document.getElementById('editParamsModal').style.display = 'none';
    currentEditingStepIndex = null;
}

function confirmEditParams() {
    const paramsEditor = document.getElementById('paramsEditor');
    const errorMsg = document.getElementById('paramsErrorMessage');
    const editor = document.getElementById('jsonEditor');
    
    try {
        // Validate JSON
        const newParams = JSON.parse(paramsEditor.value);
        
        // Get workflow data
        const workflowData = JSON.parse(editor.value);
        
        // Update the step parameters
        if (currentEditingStepIndex !== null && workflowData.workflow[currentEditingStepIndex]) {
            const step = workflowData.workflow[currentEditingStepIndex];
            
            // Keep metadata fields (id, type, description)
            const updatedStep = {
                id: step.id || step.handler_id,
                type: step.type || step.handler_type,
                description: step.description,
                ...newParams
            };
            
            // Remove undefined fields
            if (updatedStep.description === undefined) {
                delete updatedStep.description;
            }
            
            workflowData.workflow[currentEditingStepIndex] = updatedStep;
            
            // Update editor
            editor.value = JSON.stringify(workflowData, null, 2);
            
            // Update modular view if active
            if (isModularViewActive()) {
                renderModularView();
            }
            
            // Close modal
            closeEditParamsModal();
            
            console.log('Parameters updated for step:', currentEditingStepIndex);
        }
    } catch (e) {
        // Show error in modal
        errorMsg.textContent = 'Invalid JSON format: ' + e.message;
        errorMsg.style.display = 'block';
    }
}

function deleteStep(index) {
    const editor = document.getElementById('jsonEditor');
    try {
        const workflowData = JSON.parse(editor.value);
        if (workflowData.workflow && confirm('Delete this step?')) {
            workflowData.workflow.splice(index, 1);
            editor.value = JSON.stringify(workflowData, null, 2);
            renderModularView();
        }
    } catch (error) {
        console.error('Error deleting step:', error);
    }
}

function addStepDragListeners() {
    const steps = document.querySelectorAll('.workflow-step');
    steps.forEach(step => {
        step.addEventListener('dragstart', handleStepDragStart);
        step.addEventListener('dragend', handleStepDragEnd);
        step.addEventListener('dragover', handleStepDragOver);
        step.addEventListener('drop', handleStepDrop);
    });
}

let draggedStepIndex = null;

function handleStepDragStart(e) {
    draggedStepIndex = parseInt(e.currentTarget.dataset.index);
    e.currentTarget.classList.add('dragging');
    e.dataTransfer.effectAllowed = 'move';
}

function handleStepDragEnd(e) {
    e.currentTarget.classList.remove('dragging');
    
    // Clear throttle timeout
    if (dragOverThrottle) {
        clearTimeout(dragOverThrottle);
        dragOverThrottle = null;
    }
    
    // Remove drop indicator when drag ends
    const existingIndicator = document.querySelector('.drop-indicator');
    if (existingIndicator) {
        existingIndicator.remove();
    }
}

function handleStepDragOver(e) {
    e.preventDefault();
    
    console.log('handleStepDragOver triggered, effectAllowed:', e.dataTransfer.effectAllowed);
    
    // Check if this is a reorder (move) or new handler (copy)
    if (e.dataTransfer.effectAllowed === 'move') {
        e.dataTransfer.dropEffect = 'move';
        console.log('Move mode - skipping indicator');
    } else {
        e.dataTransfer.dropEffect = 'copy';
        console.log('Copy mode - showing indicator');
        
        // Use the shared update function
        if (dragOverThrottle) {
            clearTimeout(dragOverThrottle);
        }
        
        dragOverThrottle = setTimeout(() => {
            updateDropIndicator(e.clientY);
        }, 10);
    }
}

function handleStepDrop(e) {
    e.preventDefault();
    
    // Clear throttle timeout
    if (dragOverThrottle) {
        clearTimeout(dragOverThrottle);
        dragOverThrottle = null;
    }
    
    // Remove drop indicator
    const existingIndicator = document.querySelector('.drop-indicator');
    if (existingIndicator) {
        existingIndicator.remove();
    }
    
    const dropIndex = parseInt(e.currentTarget.dataset.index);
    
    // Check if this is a reorder (move) or new handler (copy)
    if (e.dataTransfer.effectAllowed === 'move') {
        // Handle reordering
        if (draggedStepIndex !== null && draggedStepIndex !== dropIndex) {
            const editor = document.getElementById('jsonEditor');
            try {
                const workflowData = JSON.parse(editor.value);
                const workflow = workflowData.workflow;
                
                // Reorder steps
                const draggedStep = workflow[draggedStepIndex];
                workflow.splice(draggedStepIndex, 1);
                workflow.splice(dropIndex, 0, draggedStep);
                
                editor.value = JSON.stringify(workflowData, null, 2);
                renderModularView();
            } catch (error) {
                console.error('Error reordering steps:', error);
            }
        }
        draggedStepIndex = null;
    } else {
        // Handle new handler drop - delegate to handleModularDrop
        handleModularDrop(e);
    }
}

// 点击模态框外部关闭
window.onclick = function(event) {
    const newWorkflowModal = document.getElementById('newWorkflowModal');
    const loadWorkflowModal = document.getElementById('loadWorkflowModal');
    const deleteWorkflowModal = document.getElementById('deleteWorkflowModal');
    const clearWorkflowModal = document.getElementById('clearWorkflowModal');
    const successModal = document.getElementById('successModal');
    
    if (event.target === newWorkflowModal) {
        closeNewWorkflowModal();
    }
    
    if (event.target === loadWorkflowModal) {
        closeLoadWorkflowModal();
    }
    
    if (event.target === deleteWorkflowModal) {
        closeDeleteWorkflowModal();
    }
    
    if (event.target === clearWorkflowModal) {
        closeClearWorkflowModal();
    }
    
    if (event.target === successModal) {
        closeSuccessModal();
    }
}
