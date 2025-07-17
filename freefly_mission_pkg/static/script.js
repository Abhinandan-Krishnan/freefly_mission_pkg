// WebSocket connection
const socket = io();

// Connection status
socket.on('connect', function() {
    console.log('Connected to server');
    document.getElementById('connection-status').textContent = 'Connected';
    document.getElementById('connection-status').className = 'badge bg-success';
});

socket.on('disconnect', function() {
    console.log('Disconnected from server');
    document.getElementById('connection-status').textContent = 'Disconnected';
    document.getElementById('connection-status').className = 'badge bg-danger';
});

// Status updates
socket.on('status_update', function(data) {
    console.log('Status update:', data);
    const statusElement = document.getElementById('drone-status');
    if (statusElement) {
        const status = data.status || data.raw || 'Unknown';
        statusElement.textContent = status;
        
        // Update badge color based on status
        if (status.includes('WAYPOINT_NAV')) {
            statusElement.className = 'badge bg-primary fs-6 mt-1';
        } else if (status.includes('IDLE')) {
            statusElement.className = 'badge bg-secondary fs-6 mt-1';
        } else if (status.includes('LANDING')) {
            statusElement.className = 'badge bg-warning fs-6 mt-1';
        } else if (status.includes('TAKEOFF')) {
            statusElement.className = 'badge bg-info fs-6 mt-1';
        } else if (status.includes('ARMING')) {
            statusElement.className = 'badge bg-warning fs-6 mt-1';
        } else {
            statusElement.className = 'badge bg-secondary fs-6 mt-1';
        }
    }
});

// Position updates
socket.on('position_update', function(data) {
    console.log('Position update:', data);
    document.getElementById('position-x').textContent = data.x.toFixed(3);
    document.getElementById('position-y').textContent = data.y.toFixed(3);
    document.getElementById('position-z').textContent = data.z.toFixed(3);
});

// Mission result updates via WebSocket
socket.on('mission_result', function(data) {
    console.log('Mission result:', data);
    const statusDiv = document.getElementById('mission-status');
    const button = document.getElementById('start-mission');
    
    if (data.success) {
        statusDiv.innerHTML = '<div class="alert alert-success">Mission: ' + data.message + '</div>';
    } else {
        statusDiv.innerHTML = '<div class="alert alert-danger">Mission Failed: ' + data.message + '</div>';
    }
    
    // Re-enable button
    button.disabled = false;
    button.innerHTML = '<i class="fas fa-play"></i> Start Mission';
});

// Land result updates via WebSocket
socket.on('land_result', function(data) {
    console.log('Land result:', data);
    const statusDiv = document.getElementById('mission-status');
    const button = document.getElementById('emergency-land');
    
    if (data.success) {
        statusDiv.innerHTML = '<div class="alert alert-success">Landing: ' + data.message + '</div>';
    } else {
        statusDiv.innerHTML = '<div class="alert alert-danger">Landing Failed: ' + data.message + '</div>';
    }
    
    // Re-enable button
    button.disabled = false;
    button.innerHTML = '<i class="fas fa-exclamation-triangle"></i> Emergency Land';
});

// Button handlers
document.getElementById('start-mission').addEventListener('click', function() {
    const button = this;
    const statusDiv = document.getElementById('mission-status');
    
    // Disable button and show loading
    button.disabled = true;
    button.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Starting Mission...';
    statusDiv.innerHTML = '<div class="alert alert-info">Initiating mission...</div>';
    
    fetch('/api/start_mission', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        console.log('Mission initiation response:', data);
        if (data.success) {
            statusDiv.innerHTML = '<div class="alert alert-info">Mission initiated. Waiting for result...</div>';
            // Result will come via WebSocket callback
        } else {
            statusDiv.innerHTML = '<div class="alert alert-danger">Failed to initiate mission: ' + data.message + '</div>';
            // Re-enable button if failed to initiate
            button.disabled = false;
            button.innerHTML = '<i class="fas fa-play"></i> Start Mission';
        }
    })
    .catch(error => {
        console.error('Error:', error);
        statusDiv.innerHTML = '<div class="alert alert-danger">Error: ' + error.message + '</div>';
        
        // Re-enable button
        button.disabled = false;
        button.innerHTML = '<i class="fas fa-play"></i> Start Mission';
    });
});

document.getElementById('emergency-land').addEventListener('click', function() {
    const button = this;
    const statusDiv = document.getElementById('mission-status');
    
    // Disable button and show loading
    button.disabled = true;
    button.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Landing...';
    statusDiv.innerHTML = '<div class="alert alert-warning">Initiating emergency landing...</div>';
    
    fetch('/api/emergency_land', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        console.log('Land initiation response:', data);
        if (data.success) {
            statusDiv.innerHTML = '<div class="alert alert-warning">Emergency landing initiated. Waiting for result...</div>';
            // Result will come via WebSocket callback
        } else {
            statusDiv.innerHTML = '<div class="alert alert-danger">Failed to initiate landing: ' + data.message + '</div>';
            // Re-enable button if failed to initiate
            button.disabled = false;
            button.innerHTML = '<i class="fas fa-exclamation-triangle"></i> Emergency Land';
        }
    })
    .catch(error => {
        console.error('Error:', error);
        statusDiv.innerHTML = '<div class="alert alert-danger">Error: ' + error.message + '</div>';
        
        // Re-enable button
        button.disabled = false;
        button.innerHTML = '<i class="fas fa-exclamation-triangle"></i> Emergency Land';
    });
});

// Waypoint upload functionality
document.getElementById('upload-waypoints').addEventListener('click', function() {
    const button = this;
    const fileInput = document.getElementById('waypoint-file');
    const statusDiv = document.getElementById('upload-status');
    
    if (!fileInput.files || fileInput.files.length === 0) {
        statusDiv.innerHTML = '<div class="alert alert-warning">Please select a file first</div>';
        return;
    }
    
    const file = fileInput.files[0];
    
    // Validate file type
    if (!file.name.toLowerCase().endsWith('.txt') && !file.name.toLowerCase().endsWith('.csv')) {
        statusDiv.innerHTML = '<div class="alert alert-danger">Please select a .txt or .csv file</div>';
        return;
    }
    
    // Disable button and show loading
    button.disabled = true;
    button.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Uploading...';
    statusDiv.innerHTML = '<div class="alert alert-info">Uploading waypoint file...</div>';
    
    // Create FormData for file upload
    const formData = new FormData();
    formData.append('file', file);
    
    fetch('/api/upload_waypoints', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        console.log('Upload response:', data);
        
        if (data.success) {
            statusDiv.innerHTML = '<div class="alert alert-success">' + data.message + '</div>';
            
            // Update filename display
            document.getElementById('waypoint-filename').textContent = data.filename;
            
            // Update waypoint preview
            const previewDiv = document.getElementById('waypoint-preview');
            if (data.waypoints && data.waypoints.length > 0) {
                let previewHtml = '<div class="mt-2"><strong>Preview (first 5 waypoints):</strong><br>';
                data.waypoints.forEach((wp, index) => {
                    previewHtml += `<small>${index + 1}. X: ${wp.x.toFixed(2)}, Y: ${wp.y.toFixed(2)}, Z: ${wp.z.toFixed(2)}</small><br>`;
                });
                if (data.waypoint_count > 5) {
                    previewHtml += `<small class="text-muted">... and ${data.waypoint_count - 5} more waypoints</small>`;
                }
                previewHtml += '</div>';
                previewDiv.innerHTML = previewHtml;
            }
        } else {
            statusDiv.innerHTML = '<div class="alert alert-danger">Upload failed: ' + data.message + '</div>';
        }
        
        // Re-enable button
        button.disabled = false;
        button.innerHTML = '<i class="fas fa-upload"></i> Upload Waypoints';
    })
    .catch(error => {
        console.error('Upload error:', error);
        statusDiv.innerHTML = '<div class="alert alert-danger">Upload error: ' + error.message + '</div>';
        
        // Re-enable button
        button.disabled = false;
        button.innerHTML = '<i class="fas fa-upload"></i> Upload Waypoints';
    });
});

// File input change handler
document.getElementById('waypoint-file').addEventListener('change', function() {
    const file = this.files[0];
    const statusDiv = document.getElementById('upload-status');
    
    if (file) {
        // Clear previous status
        statusDiv.innerHTML = '';
        
        // Validate file type
        if (!file.name.toLowerCase().endsWith('.txt') && !file.name.toLowerCase().endsWith('.csv')) {
            statusDiv.innerHTML = '<div class="alert alert-warning">Please select a .txt or .csv file</div>';
            this.value = ''; // Clear the file input
        } else {
            statusDiv.innerHTML = '<div class="alert alert-info">File selected: ' + file.name + '</div>';
        }
    }
});

// Load current waypoint file info on page load
function loadCurrentWaypointFile() {
    fetch('/api/get_current_waypoint_file')
        .then(response => response.json())
        .then(data => {
            console.log('Current waypoint file:', data);
            
            if (data.success) {
                document.getElementById('waypoint-filename').textContent = data.filename;
                
                // Update waypoint preview
                const previewDiv = document.getElementById('waypoint-preview');
                if (data.waypoints && data.waypoints.length > 0) {
                    let previewHtml = '<div class="mt-2"><strong>Preview (first 5 waypoints):</strong><br>';
                    data.waypoints.forEach((wp, index) => {
                        previewHtml += `<small>${index + 1}. X: ${wp.x.toFixed(2)}, Y: ${wp.y.toFixed(2)}, Z: ${wp.z.toFixed(2)}</small><br>`;
                    });
                    if (data.waypoint_count > 5) {
                        previewHtml += `<small class="text-muted">... and ${data.waypoint_count - 5} more waypoints</small>`;
                    }
                    previewHtml += '</div>';
                    previewDiv.innerHTML = previewHtml;
                }
            }
        })
        .catch(error => {
            console.error('Error loading current waypoint file:', error);
        });
}

// Initial data load
fetch('/api/current_data')
    .then(response => response.json())
    .then(data => {
        console.log('Initial data:', data);
        
        // Update status
        const statusElement = document.getElementById('drone-status');
        if (statusElement && data.status) {
            statusElement.textContent = data.status;
        }
        
        // Update position
        if (data.position) {
            document.getElementById('position-x').textContent = data.position.x.toFixed(3);
            document.getElementById('position-y').textContent = data.position.y.toFixed(3);
            document.getElementById('position-z').textContent = data.position.z.toFixed(3);
        }
    })
    .catch(error => {
        console.error('Error loading initial data:', error);
    });

// Load current waypoint file info
loadCurrentWaypointFile();

// Request status updates periodically
setInterval(function() {
    socket.emit('request_status');
}, 2000); // Every 2 seconds 