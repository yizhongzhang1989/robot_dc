// Workflow Config Center JavaScript Functions

// Get API base URL - dynamically adapt to current hostname and port
function getApiBaseUrl() {
    // Use the same origin (protocol + hostname + port) as the current page
    // This way the API calls go to the same server that served the page
    return window.location.origin;
}

// Resolve which robot this editor session is operating on.
//
// The workflow config center is a single Flask service shared by every
// dashboard, so the editor itself cannot know which robot opened it.
// Each dashboard passes its own robot namespace via the query string
// (e.g. http://localhost:8008/?robot=ur10e). If the parameter is missing
// we fall back to 'ur15' for backward compatibility with older bookmarks
// and standalone launches.
function getActiveRobotName() {
    try {
        const params = new URLSearchParams(window.location.search);
        const robot = params.get('robot');
        if (robot && robot.length > 0) {
            return robot;
        }
    } catch (e) {
        // URLSearchParams not available — extremely unlikely in supported
        // browsers, just fall through.
    }
    return 'ur15';
}

// Per-robot defaults for the workflow JSON's top-level ``context`` block.
//
// Populated once at editor load time by loadContextDefaults() which calls
// GET /api/<robot>/context_defaults. The backend resolves each key from
// robot_status_redis (populated at launch by each web node) and falls back
// to hardcoded ur15 values if the namespace is unknown or Redis is down.
//
// Consumers (createNewWorkflow, drop-onto-empty paths) call
// buildDefaultContext() instead of inlining ur15-specific literals, so the
// editor always pre-fills the right robot's IP/camera/etc.
let contextDefaultsCache = null;

const HARDCODED_CONTEXT_FALLBACK = Object.freeze({
    status_namespace: 'ur15',
    robot_ip: '192.168.1.15',
    robot_port: 30002,
    robot_type: 'ur15',
    camera_topic: '/ur15_camera/image_raw',
});

async function loadContextDefaults() {
    const robotName = getActiveRobotName();
    try {
        const response = await fetch(`${getApiBaseUrl()}/api/${encodeURIComponent(robotName)}/context_defaults`);
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        const data = await response.json();
        if (!data.success || !data.defaults) {
            throw new Error(data.error || 'Malformed response');
        }
        contextDefaultsCache = {
            defaults: data.defaults,
            sources: data.sources || {},
            robot: data.robot || robotName,
        };
        console.log('Context defaults loaded for', robotName, contextDefaultsCache);
        return contextDefaultsCache;
    } catch (e) {
        console.warn(`Failed to fetch context defaults for '${robotName}', using hardcoded fallback:`, e);
        contextDefaultsCache = {
            defaults: { ...HARDCODED_CONTEXT_FALLBACK, status_namespace: robotName },
            sources: {},
            robot: robotName,
        };
        return contextDefaultsCache;
    }
}

// Return a fresh copy of the context block to seed new/empty workflows
// with. Uses cached defaults if available, otherwise the hardcoded
// ur15-shaped fallback (and tags status_namespace with the active robot
// so even a cold-start editor doesn't silently target the wrong robot).
function buildDefaultContext() {
    if (contextDefaultsCache && contextDefaultsCache.defaults) {
        return { ...contextDefaultsCache.defaults };
    }
    return { ...HARDCODED_CONTEXT_FALLBACK, status_namespace: getActiveRobotName() };
}

// Rewrite an existing context block (typically loaded from a template
// that was authored against a different robot) so that robot-identity
// keys point at the active robot.
//
// Semantics, per key:
//   - source 'redis' or 'request' (server has a real value)  → ALWAYS override
//   - source 'fallback' (server has no real value, just a guess) → only fill
//     when the template didn't provide one, so any user-customized values
//     survive a template load.
//
// If contextDefaultsCache is null (load failed at startup) we add only
// status_namespace so the loaded template never silently targets the
// wrong robot.
function applyRobotDefaultsToContext(templateContext) {
    const ctx = { ...(templateContext || {}) };
    if (!contextDefaultsCache || !contextDefaultsCache.defaults) {
        ctx.status_namespace = getActiveRobotName();
        return ctx;
    }
    const { defaults, sources } = contextDefaultsCache;
    for (const [key, value] of Object.entries(defaults)) {
        const source = sources ? sources[key] : undefined;
        if (source === 'redis' || source === 'request') {
            ctx[key] = value;
        } else if (!(key in ctx) || ctx[key] === undefined || ctx[key] === null) {
            ctx[key] = value;
        }
    }
    return ctx;
}

// Align a freshly-instantiated template step (e.g. just dropped onto the
// editor) with the workflow's current context block. The shipped
// workflow_template.json is authored against ur15 and contains literal
// values like camera_topic="/ur15_camera/image_raw" and
// status_namespace="ur15" — without this rewrite, dropping a capture or
// positioning node into a ur10e workflow would silently use the ur15
// camera/namespace.
//
// Rule: for every top-level key the template step declares, if the
// current context block defines the same key, copy the context value
// onto the step. Keys that don't appear in context are left untouched,
// so handler-specific parameters (move_type, robot_pose, timeout, ...)
// stay as the template's defaults.
function applyContextToTemplateStep(step, context) {
    if (!step || !context) {
        return step;
    }
    for (const key of Object.keys(step)) {
        if (Object.prototype.hasOwnProperty.call(context, key)) {
            const ctxValue = context[key];
            if (ctxValue !== undefined && ctxValue !== null && ctxValue !== '') {
                step[key] = ctxValue;
            }
        }
    }
    return step;
}

// Pull the current context block out of the editor, returning {} if the
// editor is empty or unparseable so callers can still operate on a fresh
// workflow.
function getCurrentEditorContext() {
    try {
        const editor = document.getElementById('jsonEditor');
        if (!editor || !editor.value.trim()) {
            return {};
        }
        const data = JSON.parse(editor.value);
        return (data && data.context) || {};
    } catch (e) {
        return {};
    }
}

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
        
        // Add timeout to prevent hanging
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 5000); // 5 second timeout
        
        const response = await fetch('template/workflow_template.json', {
            signal: controller.signal
        });
        clearTimeout(timeoutId);
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        workflowTemplates = data;
        console.log('✓ Workflow templates loaded successfully');
        console.log('Available template types:', Object.keys(workflowTemplates));
    } catch (error) {
        console.error('✗ Failed to load workflow templates:', error.message);
        // Fallback to empty templates
        workflowTemplates = {
            robot_move: [],
            capture_image: [],
            record_data: [],
            positioning: []
        };
        console.warn('⚠ Using empty fallback templates');
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
    fetch(`${getApiBaseUrl()}/api/workflows`)
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
    fetch(`${getApiBaseUrl()}/api/workflow/${filename}`)
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
                
                // 更新 Modular View
                renderModularView();
                
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
    const originalFileName = fileName.endsWith('.json') ? fileName : fileName + '.json';
    
    // 创建空的workflow配置 — context is seeded with the active robot's
    // defaults (fetched from /api/<robot>/context_defaults at page load).
    const emptyWorkflow = {
        "context": buildDefaultContext(),
        "workflow": []
    };
    
    // 更新编辑器内容
    const editor = document.getElementById('jsonEditor');
    if (editor) {
        editor.value = JSON.stringify(emptyWorkflow, null, 2);
    }
    
    // 更新 Modular View
    renderModularView();
    
    closeNewWorkflowModal();
    checkWorkflowLoaded(); // Update drag state after creating new workflow
    
    // 尝试创建文件，如果已存在则自动生成新文件名
    tryCreateWorkflowFile(originalFileName, emptyWorkflow, originalFileName)
        .then(result => {
            if (result.success) {
                // 更新文件名显示为实际保存的文件名
                const filenameDisplay = document.getElementById('workflowFilename');
                if (filenameDisplay) {
                    filenameDisplay.textContent = result.fileName;
                }
            }
        })
        .catch(error => {
            console.error('Error creating workflow:', error);
            showErrorModal('Failed to create workflow: ' + error.message);
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
    fetch(`${getApiBaseUrl()}/api/workflow/${currentFilename}`, {
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
            
            // 清空 Modular View
            renderModularView();
            
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
        fetch(`${getApiBaseUrl()}/api/workflow`, {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ fileName: filename, content: jsonData })
        })
        .then(response => response.json())
        .then(data => {
            console.log('Save workflow response:', data);
            if (data.success) {
                console.log('Workflow saved:', data.message);
                console.log('Filepath:', data.filepath);
                showSuccessModal(`Workflow saved successfully!\n\nFile path: ${data.filepath}`);
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

function showLoadTemplateModal() {
    // 从模板目录获取文件列表
    fetch(`${getApiBaseUrl()}/api/templates`)
        .then(response => response.json())
        .then(data => {
            const select = document.getElementById('templateFileList');
            if (data.success && data.files.length > 0) {
                select.innerHTML = '';
                data.files.forEach(file => {
                    const option = document.createElement('option');
                    option.value = file;
                    option.textContent = file;
                    select.appendChild(option);
                });
            } else {
                select.innerHTML = '<option value="">No template files found</option>';
            }
            document.getElementById('loadTemplateModal').style.display = 'block';
        })
        .catch(error => {
            console.error('Error loading template list:', error);
            showErrorModal('Failed to load template list: ' + error.message);
        });
}

function closeLoadTemplateModal() {
    document.getElementById('loadTemplateModal').style.display = 'none';
    // 重置文件名输入框为默认值
    document.getElementById('newWorkflowFileName').value = 'default_name.json';
}

// 辅助函数：生成带编号的文件名
function generateUniqueFilename(baseFilename, counter = 1) {
    // 移除 .json 后缀
    const nameWithoutExt = baseFilename.replace(/\.json$/, '');
    // 生成新的文件名，如 filename(1).json
    return `${nameWithoutExt}(${counter}).json`;
}

// 辅助函数：尝试创建文件，如果已存在则自动重试
function tryCreateWorkflowFile(fileName, content, originalName, counter = 1) {
    return fetch(`${getApiBaseUrl()}/api/workflow`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ fileName: fileName, content: content })
    })
    .then(response => response.json())
    .then(saveData => {
        if (saveData.success) {
            console.log('Workflow file created from template:', saveData.message);
            
            // 如果使用了带编号的文件名，显示提示
            if (counter > 1) {
                showInfoModal(`File "${originalName}" already exists. Created as "${fileName}" instead.`);
            }
            
            return { success: true, fileName: fileName };
        } else if (saveData.error === 'File already exists') {
            // 文件已存在，生成新的文件名并重试
            const newFileName = generateUniqueFilename(originalName, counter);
            console.log(`File ${fileName} exists, trying ${newFileName}`);
            return tryCreateWorkflowFile(newFileName, content, originalName, counter + 1);
        } else {
            throw new Error(saveData.error);
        }
    });
}

function loadSelectedTemplate() {
    const select = document.getElementById('templateFileList');
    const templateFilename = select.value;
    
    if (!templateFilename) {
        return;
    }
    
    // 获取用户输入的新文件名
    const newFilenameInput = document.getElementById('newWorkflowFileName');
    let newFilename = newFilenameInput.value.trim();
    
    // 如果用户输入了文件名，确保以 .json 结尾
    if (newFilename) {
        if (!newFilename.endsWith('.json')) {
            newFilename += '.json';
        }
    }
    
    // 从服务器加载模板文件
    fetch(`${getApiBaseUrl()}/api/template/${templateFilename}`)
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                // Rewrite the template's context to match this dashboard's
                // active robot. Templates are shipped with hardcoded ur15
                // identity values; the editor opened from the ur10e dashboard
                // would otherwise persist those into a brand-new workflow.
                // See applyRobotDefaultsToContext for the override policy.
                const rewritten = {
                    ...data.content,
                    context: applyRobotDefaultsToContext(data.content && data.content.context),
                };

                // 更新编辑器内容
                const editor = document.getElementById('jsonEditor');
                if (editor) {
                    editor.value = JSON.stringify(rewritten, null, 2);
                }
                
                // 确定最终的文件名
                const originalFileName = newFilename || 'default_name.json';
                
                closeLoadTemplateModal();
                console.log('Loaded template:', data.filename, `as ${originalFileName}`, '(context retargeted to', getActiveRobotName(), ')');
                checkWorkflowLoaded(); // Update drag state after loading
                
                // 更新 Modular View
                renderModularView();
                
                // 尝试创建文件，如果已存在则自动生成新文件名
                tryCreateWorkflowFile(originalFileName, rewritten, originalFileName)
                    .then(result => {
                        if (result.success) {
                            // 更新文件名显示为实际保存的文件名
                            const filenameDisplay = document.getElementById('workflowFilename');
                            if (filenameDisplay) {
                                filenameDisplay.textContent = result.fileName;
                            }
                        }
                    })
                    .catch(error => {
                        console.error('Error creating workflow file:', error);
                        showErrorModal('Template loaded but failed to save file: ' + error.message);
                    });
            } else {
                showErrorModal('Failed to load template: ' + data.error);
            }
        })
        .catch(error => {
            console.error('Error loading template:', error);
            showErrorModal('Failed to load template: ' + error.message);
        });
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
    
    // 清空 Modular View
    renderModularView();
    
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
    console.log('🚀 Initializing Workflow Config Center...');

    // Surface which robot this editor session targets so users editing
    // a workflow from the ur10e dashboard don't accidentally load ur15
    // joint angles.
    try {
        const robotName = getActiveRobotName();
        const titleEl = document.querySelector('h1.title');
        if (titleEl && !titleEl.dataset.robotBadgeApplied) {
            titleEl.textContent = `Workflow Config Center [${robotName}]`;
            titleEl.dataset.robotBadgeApplied = '1';
        }
        document.title = `Workflow Config Center [${robotName}]`;
    } catch (e) {
        console.warn('Failed to apply robot badge:', e);
    }

    // Pre-fetch per-robot context defaults so "New Workflow" / template
    // / drop-onto-empty paths can seed the JSON with the active robot's
    // IP/camera/etc. Non-blocking-on-failure: buildDefaultContext()
    // falls back to a hardcoded ur15-shaped object on any error.
    try {
        await loadContextDefaults();
    } catch (error) {
        console.error('Context defaults loading error:', error);
    }

    // Load workflow templates first (non-blocking if it fails)
    try {
        await loadWorkflowTemplates();
    } catch (error) {
        console.error('Template loading error:', error);
    }
    
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
    
    console.log('✓ Workflow Config Center initialized successfully');
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
            // Create default workflow structure seeded with the active
            // robot's per-namespace context (see buildDefaultContext).
            workflowData = {
                context: buildDefaultContext(),
                workflow: []
            };
        }
        
        // Ensure workflow array exists
        if (!workflowData.workflow) {
            workflowData.workflow = [];
        }
        
        // Align the template's context-bound fields (camera_topic,
        // status_namespace, ...) with the workflow's current context
        // before inserting.
        const newStep = applyContextToTemplateStep(template, workflowData.context || {});
        
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
let lastClientY = null;

function handleModularDragOver(e) {
    e.preventDefault();
    
    // Don't process if dragging from within modular view (reordering)
    if (e.dataTransfer.effectAllowed === 'move') {
        return;
    }
    
    e.dataTransfer.dropEffect = 'copy';
    
    // Only update if mouse moved significantly (reduce updates)
    if (lastClientY !== null && Math.abs(e.clientY - lastClientY) < 5) {
        return;
    }
    
    lastClientY = e.clientY;
    
    // Throttle the indicator update to improve performance
    if (dragOverThrottle) {
        return; // Skip if already scheduled
    }
    
    dragOverThrottle = setTimeout(() => {
        updateDropIndicator(lastClientY);
        dragOverThrottle = null;
    }, 50);
}

function updateDropIndicator(clientY) {
    const modularView = document.getElementById('workflowModularView');
    const steps = modularView.querySelectorAll('.workflow-step:not(.dragging)');
    
    // Find where to show the indicator
    let insertBeforeStep = null;
    
    if (steps.length > 0) {
        for (let i = 0; i < steps.length; i++) {
            const step = steps[i];
            const rect = step.getBoundingClientRect();
            const midpoint = rect.top + rect.height / 2;
            
            if (clientY < midpoint) {
                insertBeforeStep = step;
                break;
            }
        }
    }
    
    // Check if indicator already exists
    let indicator = document.querySelector('.drop-indicator');
    
    if (!indicator) {
        // Create the indicator if it doesn't exist
        indicator = document.createElement('div');
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
    }
    
    // Check if we need to move the indicator
    const currentNextSibling = indicator.nextSibling;
    const needsMove = currentNextSibling !== insertBeforeStep;
    
    if (needsMove) {
        // Remove from current position (if in DOM)
        if (indicator.parentNode) {
            indicator.parentNode.removeChild(indicator);
        }
        
        // Insert at new position
        if (insertBeforeStep) {
            modularView.insertBefore(indicator, insertBeforeStep);
        } else {
            modularView.appendChild(indicator);
        }
    }
}

function handleModularDragLeave(e) {
    // Only clear when actually leaving the modular view, not when entering child elements
    const modularView = document.getElementById('workflowModularView');
    const relatedTarget = e.relatedTarget;
    
    // Check if we're leaving to outside the modular view
    if (!modularView.contains(relatedTarget)) {
        e.currentTarget.style.background = '';
        lastClientY = null;
        
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
    
    // Reset tracking variables
    lastClientY = null;
    
    // Clear throttle timeout
    if (dragOverThrottle) {
        clearTimeout(dragOverThrottle);
        dragOverThrottle = null;
    }
    
    // Get the indicator's position before removing it
    const existingIndicator = document.querySelector('.drop-indicator');
    let insertBeforeElement = null;
    if (existingIndicator) {
        insertBeforeElement = existingIndicator.nextSibling;
        existingIndicator.remove();
    }
    
    // Check if this is a step reorder operation
    if (e.dataTransfer.effectAllowed === 'move' && draggedStepIndex !== null) {
        // This is a step reorder, handle it here for container drops
        const editor = document.getElementById('jsonEditor');
        try {
            const workflowData = JSON.parse(editor.value);
            const workflow = workflowData.workflow;
            
            // Calculate insert index based on where the indicator was
            let insertIndex;
            if (insertBeforeElement && insertBeforeElement.classList && insertBeforeElement.classList.contains('workflow-step')) {
                // Insert before this step
                insertIndex = parseInt(insertBeforeElement.dataset.index);
                
                // Adjust for removal of dragged item
                if (draggedStepIndex < insertIndex) {
                    insertIndex--;
                }
            } else {
                // Insert at the end
                insertIndex = workflow.length - 1;
            }
            
            // Skip if not actually moving
            if (insertIndex === draggedStepIndex) {
                draggedStepIndex = null;
                return;
            }
            
            // Reorder steps
            const draggedStep = workflow[draggedStepIndex];
            workflow.splice(draggedStepIndex, 1);
            workflow.splice(insertIndex, 0, draggedStep);
            
            editor.value = JSON.stringify(workflowData, null, 2);
            renderModularView();
            
            console.log('Moved step', draggedStepIndex, 'to', insertIndex);
            draggedStepIndex = null;
            return;
        } catch (error) {
            console.error('Error moving step:', error);
            draggedStepIndex = null;
            return;
        }
    }
    
    try {
        const data = JSON.parse(e.dataTransfer.getData('text/plain'));
        
        // Check if this is actually a reorder marker (not a handler)
        if (data.reorder) {
            console.log('Ignoring reorder marker in handleModularDrop');
            return;
        }
        
        // Get template for this handler
        const template = getHandlerTemplate(data.handler_type, data.handler_id);
        
        if (!template) {
            showErrorModal('Template not found for handler: ' + data.handler_id + '. Please make sure workflow_template.json is loaded.');
            return;
        }
        
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
            // Create default workflow structure seeded with the active
            // robot's per-namespace context (see buildDefaultContext).
            workflowData = {
                context: buildDefaultContext(),
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
        
        // Align the template's context-bound fields (camera_topic,
        // status_namespace, ...) with the workflow's current context
        // before inserting.
        const newStep = applyContextToTemplateStep(template, workflowData.context || {});
        
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
                        <span class="workflow-step-type" data-type="${handlerType}">${handlerType}</span>
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
        
        // Show/hide Load Joint Angles button for movej_to_pose
        const loadJointAnglesBtn = document.getElementById('loadJointAnglesBtn');
        if (handlerName === 'movej_to_pose') {
            loadJointAnglesBtn.style.display = 'flex';
        } else {
            loadJointAnglesBtn.style.display = 'none';
        }
        
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
        const errorText = document.getElementById('paramsErrorText');
        errorMsg.style.display = 'none';
        if (errorText) errorText.textContent = '';
        
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

async function loadJointAngles() {
    try {
        // Get actual joint positions directly from the active robot.
        // The active robot is determined per-dashboard via the ?robot=
        // URL query parameter — see getActiveRobotName().
        const robotName = getActiveRobotName();
        const response = await fetch(`${getApiBaseUrl()}/api/${encodeURIComponent(robotName)}/actual_joint_positions`, {
            method: 'GET'
        });
        
        if (!response.ok) {
            throw new Error(`Failed to get joint positions from ${robotName}`);
        }
        
        const data = await response.json();
        
        if (!data.success || !data.value || data.value.length !== 6) {
            throw new Error('Invalid joint positions data');
        }
        
        // Joint positions are already in radians from the robot, round to 4 decimal places
        const jointAnglesRad = data.value.map(rad => parseFloat(rad.toFixed(4)));
        
        // Get current parameters from editor
        const paramsEditor = document.getElementById('paramsEditor');
        let params;
        try {
            params = JSON.parse(paramsEditor.value);
        } catch (e) {
            params = {};
        }
        
        // Update robot_pose with joint angles
        params.robot_pose = jointAnglesRad;
        
        // Update editor with new parameters
        paramsEditor.value = JSON.stringify(params, null, 2);
        
        console.log('Loaded joint angles from robot:', jointAnglesRad);
        
    } catch (error) {
        console.error('Error loading joint angles:', error);
        showErrorModal('Failed to load joint angles: ' + error.message);
    }
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
        const errorText = document.getElementById('paramsErrorText');
        if (errorText) {
            errorText.textContent = 'Invalid JSON format: ' + e.message;
        } else {
            errorMsg.textContent = 'Invalid JSON format: ' + e.message;
        }
        errorMsg.style.display = 'block';
    }
}

let currentDeletingStepIndex = null;

function deleteStep(index) {
    const editor = document.getElementById('jsonEditor');
    try {
        const workflowData = JSON.parse(editor.value);
        if (workflowData.workflow && workflowData.workflow[index]) {
            const step = workflowData.workflow[index];
            currentDeletingStepIndex = index;
            
            // Set step info in modal
            document.getElementById('deleteStepNumber').textContent = index + 1;
            document.getElementById('deleteStepName').textContent = step.id || step.handler_id || 'Unknown';
            
            // Show modal
            document.getElementById('deleteStepModal').style.display = 'block';
        }
    } catch (error) {
        console.error('Error opening delete modal:', error);
    }
}

function closeDeleteStepModal() {
    document.getElementById('deleteStepModal').style.display = 'none';
    currentDeletingStepIndex = null;
}

function confirmDeleteStep() {
    if (currentDeletingStepIndex === null) return;
    
    const editor = document.getElementById('jsonEditor');
    try {
        const workflowData = JSON.parse(editor.value);
        if (workflowData.workflow) {
            workflowData.workflow.splice(currentDeletingStepIndex, 1);
            editor.value = JSON.stringify(workflowData, null, 2);
            renderModularView();
            closeDeleteStepModal();
            console.log('Deleted step at index:', currentDeletingStepIndex);
        }
    } catch (error) {
        console.error('Error deleting step:', error);
        showErrorModal('Failed to delete step: ' + error.message);
    }
}

function addStepDragListeners() {
    const steps = document.querySelectorAll('.workflow-step');
    steps.forEach(step => {
        step.addEventListener('dragstart', handleStepDragStart);
        step.addEventListener('dragend', handleStepDragEnd);
        step.addEventListener('dragover', handleStepDragOver);
        step.addEventListener('dragleave', handleStepDragLeave);
        step.addEventListener('drop', handleStepDrop);
    });
}

function handleStepDragLeave(e) {
    // Don't remove indicator on step leave, only clear throttle
    // The indicator will be repositioned by dragover on the next step
}

let draggedStepIndex = null;

function handleStepDragStart(e) {
    draggedStepIndex = parseInt(e.currentTarget.dataset.index);
    e.currentTarget.classList.add('dragging');
    e.dataTransfer.effectAllowed = 'move';
    // Set some data to prevent errors (but won't be used for reordering)
    e.dataTransfer.setData('text/plain', JSON.stringify({ reorder: true }));
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
    
    // Check if this is a reorder (move) or new handler (copy)
    if (e.dataTransfer.effectAllowed === 'move') {
        e.dataTransfer.dropEffect = 'move';
    } else {
        e.dataTransfer.dropEffect = 'copy';
    }
    
    // Only update if mouse moved significantly (reduce updates)
    if (lastClientY !== null && Math.abs(e.clientY - lastClientY) < 5) {
        return;
    }
    
    lastClientY = e.clientY;
    
    // Throttle the indicator update
    if (dragOverThrottle) {
        return; // Skip if already scheduled
    }
    
    dragOverThrottle = setTimeout(() => {
        updateDropIndicator(lastClientY);
        dragOverThrottle = null;
    }, 50);
}

function handleStepDrop(e) {
    e.preventDefault();
    e.stopPropagation(); // Prevent event from bubbling to container
    
    // Reset tracking variables
    lastClientY = null;
    
    // Clear throttle timeout
    if (dragOverThrottle) {
        clearTimeout(dragOverThrottle);
        dragOverThrottle = null;
    }
    
    // Get the indicator's position before removing it
    const existingIndicator = document.querySelector('.drop-indicator');
    let insertBeforeElement = null;
    if (existingIndicator) {
        insertBeforeElement = existingIndicator.nextSibling;
        existingIndicator.remove();
    }
    
    // Check if this is a reorder (move) or new handler (copy)
    if (e.dataTransfer.effectAllowed === 'move') {
        // Handle reordering
        if (draggedStepIndex !== null) {
            const editor = document.getElementById('jsonEditor');
            try {
                const workflowData = JSON.parse(editor.value);
                const workflow = workflowData.workflow;
                
                // Calculate insert index based on where the indicator was
                let insertIndex;
                if (insertBeforeElement && insertBeforeElement.classList && insertBeforeElement.classList.contains('workflow-step')) {
                    // Insert before this step
                    insertIndex = parseInt(insertBeforeElement.dataset.index);
                } else {
                    // Insert at the end
                    insertIndex = workflow.length;
                }
                
                // Skip if dropping on itself
                if (insertIndex === draggedStepIndex || insertIndex === draggedStepIndex + 1) {
                    draggedStepIndex = null;
                    return;
                }
                
                // Adjust for removal of dragged item
                if (draggedStepIndex < insertIndex) {
                    insertIndex--;
                }
                
                // Reorder steps
                const draggedStep = workflow[draggedStepIndex];
                workflow.splice(draggedStepIndex, 1);
                workflow.splice(insertIndex, 0, draggedStep);
                
                editor.value = JSON.stringify(workflowData, null, 2);
                renderModularView();
                
                console.log('Reordered step from', draggedStepIndex, 'to', insertIndex);
            } catch (error) {
                console.error('Error reordering steps:', error);
                showErrorModal('Failed to reorder steps: ' + error.message);
            }
        }
        draggedStepIndex = null;
    } else {
        // Handle new handler drop on step
        try {
            const data = JSON.parse(e.dataTransfer.getData('text/plain'));
            
            // Make sure this is a valid handler drop, not a reorder
            if (data.handler_id && data.handler_type) {
                // Get template for this handler
                const template = getHandlerTemplate(data.handler_type, data.handler_id);
                
                if (!template) {
                    showErrorModal('Template not found for handler: ' + data.handler_id);
                    return;
                }
                
                const editor = document.getElementById('jsonEditor');
                let workflowData;
                
                try {
                    workflowData = JSON.parse(editor.value);
                } catch (error) {
                    showErrorModal('Invalid JSON format in editor.');
                    return;
                }
                
                if (!workflowData.workflow) {
                    workflowData.workflow = [];
                }
                
                // Find insert position based on mouse position
                const dropIndex = parseInt(e.currentTarget.dataset.index);
                const rect = e.currentTarget.getBoundingClientRect();
                const midpoint = rect.top + rect.height / 2;
                
                let insertIndex = dropIndex;
                if (e.clientY < midpoint) {
                    // Insert before this step
                    insertIndex = dropIndex;
                } else {
                    // Insert after this step
                    insertIndex = dropIndex + 1;
                }
                
                // Align the template's context-bound fields (camera_topic,
                // status_namespace, ...) with the workflow's current
                // context before inserting.
                const alignedStep = applyContextToTemplateStep(template, workflowData.context || {});
                
                // Insert new step
                workflowData.workflow.splice(insertIndex, 0, alignedStep);
                
                // Update editor
                editor.value = JSON.stringify(workflowData, null, 2);
                renderModularView();
                
                console.log('Handler added at position', insertIndex);
            }
        } catch (error) {
            console.error('Error handling drop:', error);
        }
    }
}

// 点击模态框外部关闭
window.onclick = function(event) {
    const newWorkflowModal = document.getElementById('newWorkflowModal');
    const loadWorkflowModal = document.getElementById('loadWorkflowModal');
    const loadTemplateModal = document.getElementById('loadTemplateModal');
    const deleteWorkflowModal = document.getElementById('deleteWorkflowModal');
    const deleteStepModal = document.getElementById('deleteStepModal');
    const successModal = document.getElementById('successModal');
    
    if (event.target === newWorkflowModal) {
        closeNewWorkflowModal();
    }
    
    if (event.target === loadWorkflowModal) {
        closeLoadWorkflowModal();
    }
    
    if (event.target === loadTemplateModal) {
        closeLoadTemplateModal();
    }
    
    if (event.target === deleteWorkflowModal) {
        closeDeleteWorkflowModal();
    }
    
    if (event.target === successModal) {
        closeSuccessModal();
    }
    
    if (event.target === deleteStepModal) {
        closeDeleteStepModal();
    }
}
