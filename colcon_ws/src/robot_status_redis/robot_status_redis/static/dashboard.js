// Global state
let currentNamespace = null;
let statusData = {};

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function formatAge(timestamp) {
    if (!timestamp) {
        return '';
    }
    
    const now = Date.now() / 1000;  // Convert to seconds
    const ageSeconds = Math.floor(now - timestamp);
    
    let ageClass = 'very-fresh';
    let ageText = '';
    
    if (ageSeconds < 1) {
        ageText = '0s ago';
        ageClass = 'very-fresh';
    } else if (ageSeconds < 60) {  // < 1 minute - very clear
        ageText = ageSeconds + 's ago';
        ageClass = 'very-fresh';
    } else if (ageSeconds < 1800) {  // < 30 minutes - less clear
        const minutes = Math.floor(ageSeconds / 60);
        const seconds = ageSeconds % 60;
        ageText = `${minutes}m ${seconds}s ago`;
        ageClass = 'fresh';
    } else if (ageSeconds < 86400) {  // < 1 day - noticeable color
        const hours = Math.floor(ageSeconds / 3600);
        const remainingMinutes = Math.floor((ageSeconds % 3600) / 60);
        const seconds = ageSeconds % 60;
        if (hours > 0) {
            if (remainingMinutes > 0) {
                ageText = `${hours}h ${remainingMinutes}m ${seconds}s ago`;
            } else {
                ageText = `${hours}h ${seconds}s ago`;
            }
        } else {
            ageText = `${remainingMinutes}m ${seconds}s ago`;
        }
        ageClass = 'stale';
    } else {  // > 1 day - dim color
        const days = Math.floor(ageSeconds / 86400);
        const remainingHours = Math.floor((ageSeconds % 86400) / 3600);
        const remainingMinutes = Math.floor((ageSeconds % 3600) / 60);
        const seconds = ageSeconds % 60;
        if (remainingHours > 0) {
            ageText = `${days}d ${remainingHours}h ${remainingMinutes}m ${seconds}s ago`;
        } else if (remainingMinutes > 0) {
            ageText = `${days}d ${remainingMinutes}m ${seconds}s ago`;
        } else {
            ageText = `${days}d ${seconds}s ago`;
        }
        ageClass = 'very-stale';
    }
    
    return `<span class="status-timestamp ${ageClass}">${ageText}</span>`;
}

function updateConnectionStatus(isConnected) {
    const statusElem = document.getElementById('connection-status');
    if (isConnected) {
        statusElem.className = 'connection-status connected';
        statusElem.textContent = '🟢 Connected';
    } else {
        statusElem.className = 'connection-status disconnected';
        statusElem.textContent = '🔴 Disconnected';
    }
}

async function deleteStatus(namespace, key) {
    if (!confirm(`Delete ${namespace}.${key}?`)) {
        return;
    }

    try {
        const response = await fetch(`/api/status/${namespace}/${key}`, {
            method: 'DELETE'
        });
        const result = await response.json();
        if (result.success) {
            refreshStatus();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        alert('Error: ' + error);
    }
}

async function deleteNamespace(namespace) {
    if (!confirm(`Delete entire namespace '${namespace}' and all its keys?`)) {
        return;
    }

    try {
        const response = await fetch(`/api/namespace/${namespace}`, {
            method: 'DELETE'
        });
        const result = await response.json();
        if (result.success) {
            currentNamespace = null;
            refreshStatus();
        } else {
            alert('Error: ' + result.message);
        }
    } catch (error) {
        alert('Error: ' + error);
    }
}

async function refreshStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        if (data.success) {
            statusData = data.status;
            renderDashboard();
            document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
            updateConnectionStatus(true);
        } else {
            updateConnectionStatus(false);
        }
    } catch (error) {
        console.error('Error fetching status:', error);
        updateConnectionStatus(false);
    }
}

function renderDashboard() {
    const namespaces = Object.keys(statusData).sort();
    
    if (namespaces.length === 0) {
        document.getElementById('tabs').innerHTML = '';
        document.getElementById('content').innerHTML = 
            '<div class="empty-state"><h2>No robot status available</h2></div>';
        return;
    }

    // Set first namespace as active if none selected
    if (!currentNamespace || !namespaces.includes(currentNamespace)) {
        currentNamespace = namespaces[0];
    }

    // Render tabs
    document.getElementById('tabs').innerHTML = namespaces.map(ns => 
        `<div class="tab ${ns === currentNamespace ? 'active' : ''}" onclick="switchNamespace('${ns}')">${ns}</div>`
    ).join('');

    // Render content
    const content = namespaces.map(ns => {
        const items = statusData[ns];
        const keys = Object.keys(items).sort();
        
        const itemsHtml = keys.map(key => {
            const item = items[key];
            let displayValue = '';
            let typeInfo = '';
            let timestampInfo = '';
            
            // Handle new format with type and value
            if (typeof item === 'object' && item.type && item.value !== undefined) {
                const escapedType = escapeHtml(item.type);
                typeInfo = `<span class="status-type">${escapedType}<span class="type-tooltip">${escapedType}</span></span>`;
                
                // Format timestamp if available
                if (item.timestamp) {
                    timestampInfo = formatAge(item.timestamp);
                }
                
                // Format the display value
                if (typeof item.value === 'object') {
                    displayValue = JSON.stringify(item.value, null, 2);
                } else {
                    displayValue = String(item.value);
                }
            } else {
                // Fallback for old format
                try {
                    const parsed = JSON.parse(item);
                    displayValue = JSON.stringify(parsed, null, 2);
                } catch (e) {
                    displayValue = String(item);
                }
            }
            
            // Escape for HTML attribute (base64 encode to avoid escaping issues)
            const encodedValue = btoa(unescape(encodeURIComponent(displayValue)));
            
            return `
                <div class="status-item">
                    <div class="status-key">
                        <div class="status-key-info">
                            <div class="status-key-row">
                                <span class="status-key-name" data-copy-value="${encodedValue}" onclick="copyToClipboard(event)">
                                    <span>${key}</span>
                                    <span class="copy-icon">📋</span>
                                    <span class="copy-tooltip">Click to copy value</span>
                                </span>
                            </div>
                            <div class="status-key-row">
                                ${typeInfo}
                                ${timestampInfo}
                            </div>
                        </div>
                        <button class="delete-btn" onclick="deleteStatus('${ns}', '${key}')">🗑️ Delete</button>
                    </div>
                    <div class="status-value">${displayValue}</div>
                    <div class="copy-feedback">✓ Copied!</div>
                </div>
            `;
        }).join('');

        return `
            <div class="namespace-content ${ns === currentNamespace ? 'active' : ''}" data-namespace="${ns}">
                <div class="namespace-header">
                    <h2>${ns}</h2>
                    <button class="delete-namespace-btn" onclick="deleteNamespace('${ns}')">🗑️ Delete Namespace</button>
                </div>
                <div class="status-grid">${itemsHtml || '<div class="empty-state">No status items</div>'}</div>
            </div>
        `;
    }).join('');

    document.getElementById('content').innerHTML = content;
}

function switchNamespace(namespace) {
    currentNamespace = namespace;
    renderDashboard();
}

async function copyToClipboard(event) {
    event.stopPropagation();
    
    try {
        // Decode the base64 encoded value
        const encodedValue = event.currentTarget.getAttribute('data-copy-value');
        const actualValue = decodeURIComponent(escape(atob(encodedValue)));
        
        await navigator.clipboard.writeText(actualValue);
        
        // Show feedback
        const feedbackElem = event.currentTarget.closest('.status-item').querySelector('.copy-feedback');
        feedbackElem.classList.add('show');
        
        setTimeout(() => {
            feedbackElem.classList.remove('show');
        }, 1500);
    } catch (err) {
        // Fallback for older browsers
        try {
            const encodedValue = event.currentTarget.getAttribute('data-copy-value');
            const actualValue = decodeURIComponent(escape(atob(encodedValue)));
            
            const textArea = document.createElement('textarea');
            textArea.value = actualValue;
            textArea.style.position = 'fixed';
            textArea.style.left = '-999999px';
            document.body.appendChild(textArea);
            textArea.select();
            
            document.execCommand('copy');
            document.body.removeChild(textArea);
            
            // Show feedback
            const feedbackElem = event.currentTarget.closest('.status-item').querySelector('.copy-feedback');
            feedbackElem.classList.add('show');
            
            setTimeout(() => {
                feedbackElem.classList.remove('show');
            }, 1500);
        } catch (err2) {
            console.error('Failed to copy:', err, err2);
            alert('Failed to copy to clipboard');
        }
    }
}

// Auto-refresh every 2 seconds
setInterval(refreshStatus, 2000);

// Initial load
refreshStatus();
