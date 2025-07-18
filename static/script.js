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

// Flight data plotting functionality
document.getElementById('plot-flight-data').addEventListener('click', function() {
    const button = this;
    const statusDiv = document.getElementById('plot-status');
    const plotContainer = document.getElementById('flight-plot-container');
    
    // Disable button and show loading
    button.disabled = true;
    button.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Loading Data...';
    statusDiv.style.display = 'block';
    statusDiv.innerHTML = '<div class="alert alert-info">Loading flight data...</div>';
    
    fetch('/api/get_flight_data')
        .then(response => response.json())
        .then(data => {
            console.log('Flight data response:', data);
            
            if (data.success) {
                statusDiv.innerHTML = '<div class="alert alert-success">Flight data loaded successfully!</div>';
                
                // Update statistics
                document.getElementById('data-points').textContent = data.stats.data_points;
                document.getElementById('flight-duration').textContent = `${data.stats.flight_duration_minutes} min (${data.stats.flight_duration_seconds}s)`;
                
                // Create 3D plot
                createFlightPlot(data.positions, data.stats);
                
                // Show plot container
                plotContainer.style.display = 'block';
                
            } else {
                statusDiv.innerHTML = '<div class="alert alert-warning">' + data.message + '</div>';
            }
            
            // Re-enable button
            button.disabled = false;
            button.innerHTML = '<i class="fas fa-chart-3d"></i> Plot Flight Path';
        })
        .catch(error => {
            console.error('Plot error:', error);
            statusDiv.innerHTML = '<div class="alert alert-danger">Error loading flight data: ' + error.message + '</div>';
            
            // Re-enable button
            button.disabled = false;
            button.innerHTML = '<i class="fas fa-chart-3d"></i> Plot Flight Path';
        });
});

function createFlightPlot(positions, stats) {
    // Extract coordinates
    const x_coords = positions.map(pos => pos[0]);
    const y_coords = positions.map(pos => pos[1]);
    const z_coords = positions.map(pos => pos[2]);
    
    // Create color gradient based on time
    const colors = [];
    for (let i = 0; i < positions.length; i++) {
        const intensity = i / positions.length;
        colors.push(`rgb(${Math.floor(255 * intensity)}, ${Math.floor(100 + 155 * intensity)}, ${Math.floor(255 * (1 - intensity))})`);
    }
    
    // Create the 3D scatter plot
    const trace = {
        x: x_coords,
        y: y_coords,
        z: z_coords,
        mode: 'lines+markers',
        type: 'scatter3d',
        line: {
            color: colors,
            width: 3
        },
        marker: {
            size: 2,
            color: colors,
            opacity: 0.8
        },
        name: 'Flight Path'
    };
    
    // Add start and end markers
    const startTrace = {
        x: [x_coords[0]],
        y: [y_coords[0]],
        z: [z_coords[0]],
        mode: 'markers',
        type: 'scatter3d',
        marker: {
            size: 8,
            color: 'green',
            symbol: 'diamond'
        },
        name: 'Start'
    };
    
    const endTrace = {
        x: [x_coords[x_coords.length - 1]],
        y: [y_coords[y_coords.length - 1]],
        z: [z_coords[z_coords.length - 1]],
        mode: 'markers',
        type: 'scatter3d',
        marker: {
            size: 8,
            color: 'red',
            symbol: 'diamond'
        },
        name: 'End'
    };
    
    const layout = {
        title: {
            text: 'Drone Flight Path 3D Visualization',
            font: { color: '#ffffff' }
        },
        scene: {
            xaxis: {
                title: 'X (meters)',
                gridcolor: '#444444',
                zerolinecolor: '#666666',
                titlefont: { color: '#ffffff' },
                tickfont: { color: '#ffffff' }
            },
            yaxis: {
                title: 'Y (meters)',
                gridcolor: '#444444',
                zerolinecolor: '#666666',
                titlefont: { color: '#ffffff' },
                tickfont: { color: '#ffffff' }
            },
            zaxis: {
                title: 'Z (meters)',
                gridcolor: '#444444',
                zerolinecolor: '#666666',
                titlefont: { color: '#ffffff' },
                tickfont: { color: '#ffffff' }
            },
            bgcolor: '#1a1a1a',
            camera: {
                eye: { x: 1.5, y: 1.5, z: 1.5 }
            }
        },
        paper_bgcolor: '#000000',
        plot_bgcolor: '#000000',
        font: { color: '#ffffff' },
        margin: { l: 0, r: 0, t: 50, b: 0 },
        height: 600,
        width: null, // Use full container width
        autosize: true,
        showlegend: true,
        legend: {
            font: { color: '#ffffff' },
            bgcolor: 'rgba(0,0,0,0.8)'
        }
    };
    
    const config = {
        responsive: true,
        displayModeBar: true,
        modeBarButtonsToRemove: ['pan2d', 'lasso2d', 'select2d'],
        displaylogo: false,
        autosize: true
    };
    
    // Create the plot
    Plotly.newPlot('flight-plot', [trace, startTrace, endTrace], layout, config);
    
    // Handle window resize to make plot responsive
    window.addEventListener('resize', function() {
        Plotly.Plots.resize('flight-plot');
    });
}

// Load current waypoint file info
loadCurrentWaypointFile();

// Request status updates periodically
setInterval(function() {
    socket.emit('request_status');
}, 2000); // Every 2 seconds 