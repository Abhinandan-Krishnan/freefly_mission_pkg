# Running Drone Web App as ROS Node

The drone web application has been properly integrated as a ROS node with the following benefits:

## 🚀 **Benefits of Running as ROS Node:**

1. **Proper ROS Integration**: Full lifecycle management
2. **Launch File Support**: Can be included in complex launch configurations
3. **Parameter Support**: Configurable via ROS parameters
4. **Service Discovery**: Automatic discovery of drone services
5. **Better Error Handling**: Proper ROS shutdown handling

## 📋 **Available Commands:**

### **Individual Node Commands:**

```bash
# Terminal 1: Start drone navigation node
cd /path/to/freefly_ws
source install/setup.bash
ros2 run freefly_mission_pkg drone_nav_node

# Terminal 2: Start web app node
cd /path/to/freefly_ws
source install/setup.bash
ros2 run freefly_mission_pkg drone_web_app
```

### **Launch File Commands:**

```bash
# Launch complete system (drone + web app)
ros2 launch freefly_mission_pkg drone_control_system.launch.py

# Launch just the web app
ros2 launch freefly_mission_pkg drone_web_app.launch.py
```

## ⚙️ **Configuration Parameters:**

The web app accepts the following ROS parameters:

- `web_port`: Port to run web server on (default: 5000)
- `web_host`: Host address to bind to (default: '0.0.0.0')

### **Example with Custom Parameters:**

```bash
ros2 run freefly_mission_pkg drone_web_app --ros-args -p web_port:=8080 -p web_host:=localhost
```

## 🔧 **Launch File Configuration:**

You can customize the launch files for your specific needs:

```python
# In your launch file
Node(
    package='freefly_mission_pkg',
    executable='drone_web_app',
    name='drone_web_app',
    output='screen',
    parameters=[
        {'web_port': 8080},
        {'web_host': 'localhost'}
    ]
)
```

## 🎯 **Usage Workflow:**

1. **Build the package:**
   ```bash
   cd /path/to/freefly_ws
   colcon build --packages-select freefly_mission_pkg
   source install/setup.bash
   ```

2. **Start the system:**
   ```bash
   # Option 1: Launch everything together
   ros2 launch freefly_mission_pkg drone_control_system.launch.py
   
   # Option 2: Start nodes individually
   ros2 run freefly_mission_pkg drone_nav_node
   ros2 run freefly_mission_pkg drone_web_app
   ```

3. **Access the web interface:**
   - Open browser to `http://localhost:5000`
   - Use the web interface to control the drone

## 🔍 **Debugging:**

### **Check if nodes are running:**
```bash
ros2 node list
```

### **Check available services:**
```bash
ros2 service list
```

### **Monitor topics:**
```bash
# Monitor position updates
ros2 topic echo /fmu/out/vehicle_local_position

# Monitor status updates
ros2 topic echo /drone/status
```

## 🎨 **Web Interface Features:**

- **Real-time Position Updates**: Live 3D trajectory plotting
- **Mission Control**: Start/stop missions via web interface
- **Status Monitoring**: Real-time flight state and telemetry
- **Emergency Controls**: Emergency landing button
- **WebSocket Integration**: Live data streaming

## 🛠️ **Development Notes:**

- **Template Location**: Templates are now properly packaged with the ROS node
- **Static Files**: CSS and JavaScript are included in the package
- **ROS Parameters**: Web server configuration via ROS parameters
- **Service Integration**: Direct integration with drone navigation services
- **Error Handling**: Proper ROS shutdown and cleanup

## 📂 **File Structure:**

```
freefly_mission_pkg/
├── launch/
│   ├── drone_control_system.launch.py  # Complete system launch
│   └── drone_web_app.launch.py         # Web app only launch
├── freefly_mission_pkg/
│   ├── web_app.py                      # Main web app node
│   ├── drone_nav_node.py               # Drone navigation node
│   ├── templates/
│   │   └── index.html                  # Web interface template
│   └── static/
│       ├── style.css                   # Web interface styling
│       └── script.js                   # Web interface JavaScript
└── setup.py                           # Package configuration
```

This integration provides a professional, production-ready drone control system with web interface! 