#!/usr/bin/env python3

import os
import time
import threading
import socket
import getpass
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

# Get username for consistent path handling
username = getpass.getuser()

# Initialize Flask app with proper paths
app = Flask(__name__, 
            static_folder=f'/home/{username}/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/static',
            template_folder=f'/home/{username}/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/freefly_mission_pkg/templates')
app.config['SECRET_KEY'] = 'drone_control_secret_key'
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024  # 16MB max file size
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Create waypoints directory if it doesn't exist
waypoints_dir = f'/home/{username}/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/data/waypoints'
os.makedirs(waypoints_dir, exist_ok=True)

# Global variables to hold drone data
drone_status_data = {"status": "Initializing...", "raw": ""}
drone_position_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
mission_result = {"success": False, "message": "No operation"}
land_result = {"success": False, "message": "No operation"}
current_waypoint_file = None

class DroneWebApp(Node):
    def __init__(self):
        super().__init__('drone_web_app')
        
        # Declare parameters
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('web_host', '0.0.0.0')
        
        # Get parameters
        self.web_port = self.get_parameter('web_port').value
        self.web_host = self.get_parameter('web_host').value
        
        # Setup subscribers
        self.status_subscriber = self.create_subscription(
            String,
            '/drone/status',
            self.status_callback,
            10
        )
        
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        # Setup service clients
        self.mission_client = self.create_client(Trigger, '/drone/start_waypoint_mission')
        self.land_client = self.create_client(Trigger, '/drone/land')
        
        # Wait for services to be available
        while not self.mission_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Mission service not available, waiting...')
        
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Land service not available, waiting...')
        
        # Service futures for async calls
        self.mission_future = None
        self.land_future = None
        
        self.get_logger().info('Drone Web App Node initialized')
    
    def status_callback(self, msg):
        """Handle status updates"""
        global drone_status_data
        drone_status_data = {
            "status": msg.data,
            "raw": msg.data
        }
        
        # Emit status update via WebSocket
        try:
            socketio.emit('status_update', drone_status_data)
        except Exception as e:
            self.get_logger().warn(f"Failed to emit status update: {e}")
    
    def pose_callback(self, msg):
        """Handle position updates"""
        global drone_position_data
        drone_position_data = {
            'x': round(msg.pose.position.x, 3),
            'y': round(msg.pose.position.y, 3),
            'z': round(msg.pose.position.z, 3)
        }
        
        # Emit position update via WebSocket
        try:
            socketio.emit('position_update', drone_position_data)
        except Exception as e:
            self.get_logger().warn(f"Failed to emit position update: {e}")
    
    def call_mission_service(self):
        """Call the mission service asynchronously"""
        if not self.mission_client.service_is_ready():
            self.get_logger().error("Mission service is not available")
            global mission_result
            mission_result = {'success': False, 'message': 'Mission service not available'}
            return
        
        request = Trigger.Request()
        self.mission_future = self.mission_client.call_async(request)
        self.mission_future.add_done_callback(self.mission_response_callback)
        self.get_logger().info("Mission service call triggered")
    
    def mission_response_callback(self, future):
        """Handle mission service response"""
        global mission_result
        try:
            response = future.result()
            mission_result = {
                'success': response.success,
                'message': response.message
            }
            self.get_logger().info(f"Mission response: {mission_result}")
            # Emit result via WebSocket
            socketio.emit('mission_result', mission_result)
        except Exception as e:
            mission_result = {
                'success': False,
                'message': f'Service call failed: {str(e)}'
            }
            self.get_logger().error(f"Mission service call failed: {e}")
            socketio.emit('mission_result', mission_result)
    
    def call_land_service(self):
        """Call the land service asynchronously"""
        if not self.land_client.service_is_ready():
            self.get_logger().error("Land service is not available")
            global land_result
            land_result = {'success': False, 'message': 'Land service not available'}
            return
        
        request = Trigger.Request()
        self.land_future = self.land_client.call_async(request)
        self.land_future.add_done_callback(self.land_response_callback)
        self.get_logger().info("Land service call triggered")
    
    def land_response_callback(self, future):
        """Handle land service response"""
        global land_result
        try:
            response = future.result()
            land_result = {
                'success': response.success,
                'message': response.message
            }
            self.get_logger().info(f"Land response: {land_result}")
            # Emit result via WebSocket
            socketio.emit('land_result', land_result)
        except Exception as e:
            land_result = {
                'success': False,
                'message': f'Service call failed: {str(e)}'
            }
            self.get_logger().error(f"Land service call failed: {e}")
            socketio.emit('land_result', land_result)

# Flask routes
@app.route('/')
def index():
    """Serve the main dashboard"""
    return render_template('index.html')

@app.route('/api/current_data')
def current_data():
    """Get current drone data"""
    global drone_status_data, drone_position_data
    return jsonify({
        'status': drone_status_data["status"],
        'position': drone_position_data
    })

@app.route('/api/start_mission', methods=['POST'])
def start_mission():
    """Start waypoint mission"""
    global mission_result
    mission_result = {"success": False, "message": "Processing..."}
    
    if node:
        node.call_mission_service()
        # Return immediately - actual result will come via callback
        return jsonify({"success": True, "message": "Mission service call initiated"})
    return jsonify({'success': False, 'message': 'No drone connection'})

@app.route('/api/emergency_land', methods=['POST'])
def emergency_land():
    """Emergency land"""
    global land_result
    land_result = {"success": False, "message": "Processing..."}
    
    if node:
        node.call_land_service()
        # Return immediately - actual result will come via callback
        return jsonify({"success": True, "message": "Land service call initiated"})
    return jsonify({'success': False, 'message': 'No drone connection'})

@app.route('/api/get_mission_result', methods=['GET'])
def get_mission_result():
    """Get the latest mission result"""
    global mission_result
    return jsonify(mission_result)

@app.route('/api/get_land_result', methods=['GET'])
def get_land_result():
    """Get the latest land result"""
    global land_result
    return jsonify(land_result)

@app.route('/api/upload_waypoints', methods=['POST'])
def upload_waypoints():
    """Upload waypoint file - overwrites previous file"""
    global current_waypoint_file
    
    try:
        if 'file' not in request.files:
            return jsonify({'success': False, 'message': 'No file provided'})
        
        file = request.files['file']
        if file.filename == '':
            return jsonify({'success': False, 'message': 'No file selected'})
        
        if file and file.filename:
            # Ensure it's a text file
            if not file.filename.lower().endswith(('.txt', '.csv')):
                return jsonify({'success': False, 'message': 'Only .txt and .csv files are allowed'})
            
            # Delete any existing waypoint files
            for existing_file in os.listdir(waypoints_dir):
                if existing_file.lower().endswith(('.txt', '.csv')):
                    os.remove(os.path.join(waypoints_dir, existing_file))
            
            # Save the new file with fixed name
            filename = 'waypoints.txt'
            filepath = os.path.join(waypoints_dir, filename)
            file.save(filepath)
            
            # Read and validate the file content
            waypoints = []
            try:
                with open(filepath, 'r') as f:
                    for line_num, line in enumerate(f, 1):
                        line = line.strip()
                        if line and not line.startswith('#'):  # Skip empty lines and comments
                            try:
                                # Parse X,Y,Z coordinates
                                coords = line.split(',')
                                if len(coords) >= 3:
                                    x = float(coords[0].strip())
                                    y = float(coords[1].strip())
                                    z = float(coords[2].strip())
                                    waypoints.append({'x': x, 'y': y, 'z': z})
                                else:
                                    return jsonify({
                                        'success': False, 
                                        'message': f'Invalid format at line {line_num}. Expected X,Y,Z format.'
                                    })
                            except ValueError:
                                return jsonify({
                                    'success': False, 
                                    'message': f'Invalid coordinates at line {line_num}. Expected numeric values.'
                                })
                
                if not waypoints:
                    return jsonify({'success': False, 'message': 'No valid waypoints found in file'})
                
                # Store the current waypoint file info
                current_waypoint_file = {
                    'filename': filename,
                    'filepath': filepath,
                    'waypoints': waypoints,
                    'count': len(waypoints)
                }
                
                return jsonify({
                    'success': True, 
                    'message': f'Successfully uploaded {len(waypoints)} waypoints (replaced previous file)',
                    'filename': filename,
                    'waypoint_count': len(waypoints),
                    'waypoints': waypoints[:5]  # Return first 5 waypoints for preview
                })
                
            except Exception as e:
                return jsonify({'success': False, 'message': f'Error reading file: {str(e)}'})
        
        return jsonify({'success': False, 'message': 'Invalid file'})
    
    except Exception as e:
        return jsonify({'success': False, 'message': f'Upload failed: {str(e)}'})

@app.route('/api/get_current_waypoint_file', methods=['GET'])
def get_current_waypoint_file():
    """Get information about the current waypoint file"""
    global current_waypoint_file
    
    # Check if waypoint file exists
    waypoint_file_path = os.path.join(waypoints_dir, 'waypoints.txt')
    
    if os.path.exists(waypoint_file_path) and current_waypoint_file:
        return jsonify({
            'success': True,
            'filename': current_waypoint_file['filename'],
            'waypoint_count': current_waypoint_file['count'],
            'waypoints': current_waypoint_file['waypoints'][:5]  # Return first 5 waypoints for preview
        })
    else:
        return jsonify({
            'success': False,
            'message': 'No waypoint file uploaded'
        })

# SocketIO events
@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print("Client connected")
    global drone_status_data, drone_position_data
    emit('status_update', drone_status_data)
    emit('position_update', drone_position_data)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print("Client disconnected")

@socketio.on('request_status')
def handle_status_request():
    """Handle status request"""
    global drone_status_data, drone_position_data
    emit('status_update', drone_status_data)
    emit('position_update', drone_position_data)

def find_free_port(start_port=5000):
    """Find an available port starting from start_port."""
    port = start_port
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(('0.0.0.0', port))
                s.listen(1)
                return port
            except OSError:
                port += 1  # Increment port and try again

def main(args=None):
    global node
    rclpy.init(args=args)
    node = DroneWebApp()
    
    # Find a free port for the web app
    free_port = find_free_port(node.web_port if node.web_port else 5000)
    
    # Use MultiThreadedExecutor to allow Flask and ROS2 to operate concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run the Flask app in a separate daemon thread
    flask_thread = threading.Thread(
        target=lambda: socketio.run(app, host=node.web_host, port=free_port, debug=False, allow_unsafe_werkzeug=True),
        daemon=True
    )
    flask_thread.start()
    
    node.get_logger().info(f"Drone Web App running on http://{node.web_host}:{free_port}")
    print(f"Drone Web App initialized")
    print(f"Web app will run on {node.web_host}:{free_port}")
    print(f"Starting Drone Web App on http://{node.web_host}:{free_port}")
    
    # Spin the executor in the main thread
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down web app...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 