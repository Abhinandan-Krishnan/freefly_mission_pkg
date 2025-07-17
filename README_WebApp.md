# Drone Control Web Application

A fancy web-based dashboard for real-time drone monitoring and control with live 3D trajectory visualization.

## Features

- **Live 3D Trajectory Plotting**: Real-time visualization of drone position using Plotly
- **Mission Control**: Start waypoint missions directly from the web interface
- **Emergency Landing**: Emergency landing button for safety
- **Real-time Status**: Live updates of drone state, armed status, and altitude
- **Interactive 3D Plot**: Zoom, pan, and rotate the 3D trajectory plot
- **Responsive Design**: Works on desktop and mobile devices

## Installation

1. Install required Python packages:
```bash
pip3 install -r requirements.txt
```

2. Build the ROS2 package:
```bash
cd /path/to/freefly_ws
colcon build --packages-select freefly_mission_pkg
source install/setup.bash
```

## Usage

### 1. Start the Drone Navigation Node
```bash
ros2 run freefly_mission_pkg drone_nav_node
```

### 2. Start the Web Application
```bash
cd /path/to/freefly_ws/src/freefly_mission_pkg/freefly_mission_pkg
python3 web_app.py
```

### 3. Open Web Browser
Navigate to: `http://localhost:5000`

## Web Interface Components

### Control Panel
- **Start Mission**: Initiates the complete waypoint mission (ARM → TAKEOFF → NAVIGATE → LAND)
- **Emergency Land**: Emergency landing from any flight state

### Status Panel
- **State**: Current flight state (IDLE, ARMING, TAKEOFF, WAYPOINT_NAV, etc.)
- **Armed**: Whether the drone is armed
- **Altitude**: Current altitude in meters

### Position Panel
- **X, Y, Z**: Current position coordinates in meters

### 3D Trajectory Plot
- **Live Plotting**: Real-time trajectory visualization
- **Interactive**: Zoom, pan, rotate the 3D view
- **Clear Plot**: Reset the trajectory history
- **Reset View**: Return to default camera angle

## API Endpoints

- `GET /`: Main dashboard page
- `GET /api/current_data`: Current drone position and status
- `POST /api/start_mission`: Start waypoint mission
- `POST /api/emergency_land`: Emergency landing

## WebSocket Events

- `position_update`: Real-time position updates
- `status_update`: Real-time status updates
- `connect/disconnect`: Connection status

## File Structure

```
freefly_mission_pkg/
├── web_app.py              # Flask backend with ROS2 integration
├── templates/
│   └── index.html          # Main HTML template
├── static/
│   ├── style.css           # CSS styling
│   └── script.js           # JavaScript for live plotting
├── requirements.txt        # Python dependencies
└── README_WebApp.md       # This file
```

## Features in Detail

### Live 3D Visualization
- Uses Plotly.js for smooth, interactive 3D plotting
- Real-time position updates via WebSocket
- Color-coded trajectory showing flight progression
- Configurable axis ranges and camera angles

### Mission Control
- Direct integration with ROS2 services
- Real-time feedback on mission status
- Loading indicators and error handling
- Bootstrap modals for user notifications

### Responsive Design
- Clean, modern UI with Bootstrap 5
- Gradient backgrounds and smooth animations
- Mobile-friendly responsive layout
- Custom CSS for drone-specific styling

## Troubleshooting

1. **Connection Issues**: Ensure the drone navigation node is running before starting the web app
2. **Port 5000 in Use**: Change the port in `web_app.py` if needed
3. **Missing Dependencies**: Install all packages from `requirements.txt`
4. **ROS2 Services Not Found**: Verify the drone navigation node is running and services are available

## Security Note

This web application is designed for local development and testing. For production use, implement proper authentication and security measures. 