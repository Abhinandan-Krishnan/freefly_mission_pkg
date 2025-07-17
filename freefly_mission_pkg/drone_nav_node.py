#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import os
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint
)

# Import our utility functions
from .drone_utilities import DroneUtilities, NavigationHelper


class SimpleDroneController(Node):
    def __init__(self):
        super().__init__('simple_drone_controller')
        
        # Parameters
        self.declare_parameter('config_file', 'takeoff_config.yaml')
        self.declare_parameter('waypoints_file', 'waypoints.txt')
        
        # QoS profile for PX4 communication (required for PX4)
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # State variables
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.is_armed = False
        self.waypoints_started = False
        
        # Initialize with default configuration (will be reloaded on mission start)
        self.config = {
            'takeoff_height': 5.0,
            'max_horizontal_speed': 3.0,
            'max_vertical_speed': 1.5,
            'takeoff_speed': 2.0,
            'landing_speed': 1.0,
            'waypoint_tolerance': 0.2,
            'approach_distance': 5.0,
            'min_speed': 0.2
        }
        
        # Navigation state
        self.mission_active = False
        self.home_position = [0.0, 0.0, 0.0]
        self.navigation_helper = None
        
        # Position logging
        self.data_folder = "/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/data"
        os.makedirs(self.data_folder, exist_ok=True)
        self.position_log_file = None

        
        # File paths for dynamic loading
        self.config_file_path = "/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/config/takeoff_config.yaml"
        self.waypoints_file_path = "/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/config/waypoints.txt"
        
        # Publishers to PX4
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos_profile)
        
        # Publishers for status/monitoring
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        
        # Subscribers from PX4
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, self.qos_profile)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, self.qos_profile)
        
        # Services for control - Only waypoint mission and emergency land
        self.land_srv = self.create_service(
            Trigger, '/drone/land', self.land_callback)
        self.waypoint_mission_srv = self.create_service(
            Trigger, '/drone/start_waypoint_mission', self.waypoint_mission_callback)
        
        # Control state
        self.flight_state = "IDLE"  # IDLE, ARMING, TAKEOFF, HOVER, WAYPOINT_NAV, RETURN_HOME, LANDING
        self.target_position = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter = 0
        
        # Continuous control timer (REQUIRED for PX4)
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f"Simple Drone Controller initialized")
        self.get_logger().info(f"Data folder: {self.data_folder}")
        self.get_logger().info(f"Config file: {self.config_file_path}")
        self.get_logger().info(f"Waypoints file: {self.waypoints_file_path}")
        self.get_logger().info("Available services:")
        self.get_logger().info("  - /drone/start_waypoint_mission: Start waypoint mission (includes takeoff)")
        self.get_logger().info("  - /drone/land: Emergency land (can be used during mission)")
        self.get_logger().info("Note: Config and waypoints are loaded fresh on each mission start")

    def load_configuration(self):
        """Load configuration using DroneUtilities"""
        self.config = DroneUtilities.load_config(self.config_file_path, self.get_logger())
        self.get_logger().info(f"Loaded config - Takeoff height: {self.config['takeoff_height']}m")
    
    def load_waypoint_data(self):
        """Load waypoints using DroneUtilities"""
        waypoints = DroneUtilities.load_waypoints(self.waypoints_file_path, self.get_logger())
        
        if waypoints:
            self.navigation_helper = NavigationHelper(waypoints, self.config['waypoint_tolerance'])
            self.get_logger().info(f"Loaded {len(waypoints)} waypoints")
            return True
        else:
            self.navigation_helper = None
            self.get_logger().error("No waypoints loaded")
            return False
    
    def vehicle_status_callback(self, msg):
        """Callback for vehicle status updates"""
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
    
    def vehicle_local_position_callback(self, msg):
        """Callback for vehicle position updates"""
        self.vehicle_local_position = msg
        
        # Store home position on first callback
        if self.flight_state == "IDLE" and not self.mission_active:
            self.home_position = [msg.x, msg.y, msg.z]
        
        # Write position to file during mission
        if self.waypoints_started and self.position_log_file:
            self.position_log_file.write(f"{msg.x}, {msg.y}, {msg.z}\n")
            self.position_log_file.flush()
        
        # Publish position as PoseStamped for visualization
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = msg.x
        pose_msg.pose.position.y = msg.y
        pose_msg.pose.position.z = msg.z
        self.pose_pub.publish(pose_msg)

    def control_loop(self):
        """Main control loop - runs continuously at 10Hz (REQUIRED for PX4)"""
        
        # Always publish offboard control mode when in offboard
        if self.flight_state in ["ARMING", "TAKEOFF", "HOVER", "WAYPOINT_NAV", "RETURN_HOME", "LANDING"]:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
        
        # State machine logic
        if self.flight_state == "ARMING":
            if self.offboard_setpoint_counter >= 10:
                # Switch to offboard mode after 10 setpoints
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.arm()
                self.flight_state = "TAKEOFF"
                self.target_position = [0.0, 0.0, -self.config['takeoff_height']]
                self.get_logger().info(f"Switching to offboard mode and taking off to {self.config['takeoff_height']}m")
        
        elif self.flight_state == "TAKEOFF":
            # Check if reached takeoff altitude
            current_altitude = -self.vehicle_local_position.z
            if abs(current_altitude - self.config['takeoff_height']) < 0.5:
                if self.mission_active and self.navigation_helper:
                    self.flight_state = "WAYPOINT_NAV"
                    self.navigation_helper.reset_mission()
                    first_waypoint = self.navigation_helper.get_current_waypoint()
                    if first_waypoint:
                        self.target_position = first_waypoint
                        self.get_logger().info(f"Starting waypoint navigation to waypoint 1: {first_waypoint}")
                else:
                    self.flight_state = "HOVER"
                    self.get_logger().info(f"Reached takeoff altitude: {current_altitude:.1f}m - Hovering")
        
        elif self.flight_state == "WAYPOINT_NAV":
            if self.navigation_helper:
                current_pos = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]
                
                if self.navigation_helper.check_waypoint_reached(current_pos,self.get_logger()):
                    progress = self.navigation_helper.get_mission_progress()
                    self.get_logger().info(f"Reached waypoint {progress['current_waypoint']}")

                    #Indidate that waypoints nav is started for logging and nozzle operation
                    self.waypoints_started = True
                    
                    has_next, next_waypoint,waypoint_number = self.navigation_helper.advance_to_next_waypoint()
                    
                    if has_next and next_waypoint:
                        # Go to next waypoint
                        self.target_position = next_waypoint
                        progress = self.navigation_helper.get_mission_progress()
                        self.get_logger().info(f"Navigating to waypoint {progress['current_waypoint']}: {next_waypoint}")
                    else:
                        # All waypoints visited, return home
                        self.flight_state = "RETURN_HOME"
                        self.waypoints_started = False
                        self.target_position = [self.home_position[0], self.home_position[1], -self.config['takeoff_height']]
                        self.get_logger().info("All waypoints visited, returning home")
        
        elif self.flight_state == "RETURN_HOME":
            current_pos = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]
            home_target = [self.home_position[0], self.home_position[1], -self.config['takeoff_height']]
            distance = DroneUtilities.calculate_distance(current_pos, home_target)
            
            if distance < self.config['waypoint_tolerance']:
                self.get_logger().info("Returned to home position, starting landing")
                self.flight_state = "LANDING"
                self.target_position = [self.home_position[0], self.home_position[1], 0.0]
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        elif self.flight_state == "LANDING":
            # Check if landed
            current_altitude = -self.vehicle_local_position.z
            if current_altitude < 0.5:
                self.flight_state = "IDLE"
                self.mission_active = False
                
                # Close position log file
                if self.position_log_file:
                    self.position_log_file.close()
                    self.position_log_file = None
                    self.get_logger().info("Position logging stopped")
                self.get_logger().info("Landing completed - Mission finished")
        
        # Increment counter for initial offboard setup
        if self.offboard_setpoint_counter < 20:
            self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        """Publish offboard control mode with velocity priority for speed control (REQUIRED continuously)"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Calculate distance to target to determine control mode
        current_pos = [self.vehicle_local_position.x, 
                      self.vehicle_local_position.y, 
                      self.vehicle_local_position.z]
        distance = DroneUtilities.calculate_distance(current_pos, self.target_position)
        
        if distance > 0.1 and self.flight_state in ["TAKEOFF", "WAYPOINT_NAV", "RETURN_HOME", "LANDING"]:
            # Use velocity control for speed management
            msg.position = False
            msg.velocity = True
        else:
            # Use position control for precision near targets
            msg.position = True  
            msg.velocity = False
            
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_trajectory_setpoint(self):
        """Publish trajectory setpoint with speed control (REQUIRED continuously)"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Calculate current position and velocity
        current_pos = [self.vehicle_local_position.x, 
                      self.vehicle_local_position.y, 
                      self.vehicle_local_position.z]
        
        # Calculate distance to target
        distance = DroneUtilities.calculate_distance(current_pos, self.target_position)
        
        # Use velocity-based control for speed limiting
        if distance > 0.1:  # Not at target
            # Calculate velocity vector with speed control using utilities
            velocity = DroneUtilities.calculate_velocity_vector(
                current_pos, self.target_position, self.flight_state, 
                self.config, self.get_logger()
            )
            msg.velocity = velocity
            
            # Set position to NaN to use velocity control
            msg.position = [float('nan'), float('nan'), float('nan')]
            
        else:
            # Close to target - use position control for precision
            msg.position = self.target_position
            msg.velocity = [0.0, 0.0, 0.0]
        
        # Set yaw (let autopilot control)
        msg.yaw = float('nan')
        
        self.trajectory_setpoint_pub.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish a vehicle command to PX4"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
    
    def arm(self):
        """Arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")
    
    def disarm(self):
        """Disarm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")
    
    def publish_status(self):
        """Publish current status"""
        altitude = -self.vehicle_local_position.z  # Convert NED to altitude
        
        # Base status information
        status = (f"State: {self.flight_state}, "
                 f"Armed: {self.is_armed}, "
                 f"Altitude: {altitude:.1f}m")
        
        # Add waypoint information if mission is active
        if self.mission_active and self.navigation_helper:
            progress = self.navigation_helper.get_mission_progress()
            if self.flight_state == "WAYPOINT_NAV":
                status += f", Waypoint: {progress['current_waypoint']}/{progress['total_waypoints']}"
            elif self.flight_state == "RETURN_HOME":
                status += f", Returning Home ({progress['total_waypoints']} waypoints completed)"
            else:
                status += f", Mission: {progress['total_waypoints']} waypoints planned"
        else:
            status += f", Target: {self.config['takeoff_height']}m"
        
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def land_callback(self, request, response):
        """Service callback for emergency landing - can be used during mission"""
        try:
            if self.flight_state in ["TAKEOFF", "HOVER", "WAYPOINT_NAV", "RETURN_HOME"]:
                previous_state = self.flight_state
                self.flight_state = "LANDING"
                self.mission_active = False  # Stop any active mission
                self.target_position = [self.vehicle_local_position.x, 
                                      self.vehicle_local_position.y, 
                                      0.0]
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                
                # Close position log file on emergency landing
                if self.position_log_file:
                    self.position_log_file.close()
                    self.position_log_file = None
                    self.get_logger().info("Position logging stopped due to emergency landing")
                
                response.success = True
                response.message = f"Emergency landing initiated from {previous_state} - Mission aborted"
                self.get_logger().info(f"Emergency landing initiated from {previous_state} - Mission aborted")
            elif self.flight_state == "LANDING":
                response.success = True
                response.message = "Already landing"
            elif self.flight_state == "IDLE":
                response.success = False
                response.message = "Drone is already on ground"
            else:
                response.success = False
                response.message = f"Cannot land from current state: {self.flight_state}"
        except Exception as e:
            response.success = False
            response.message = f"Landing failed: {str(e)}"
        return response
    
    def waypoint_mission_callback(self, request, response):
        """Service callback for starting a complete waypoint mission (includes takeoff)"""
        try:
            if self.flight_state == "IDLE":
                # Load fresh configuration and waypoints each time
                self.get_logger().info("Loading fresh configuration and waypoints...")
                self.load_configuration()
                
                if not self.load_waypoint_data():
                    response.success = False
                    response.message = "Failed to load waypoints for mission"
                    return response
                
                # Start mission with fresh data
                self.mission_active = True
                self.flight_state = "ARMING"  # Start mission from arming
                self.offboard_setpoint_counter = 0
                self.target_position = [0.0, 0.0, -self.config['takeoff_height']]
                
                # Open position log file when mission starts
                log_file_path = os.path.join(self.data_folder, "drone_position.txt")
                self.position_log_file = open(log_file_path, 'w')
                self.get_logger().info(f"Started position logging to: {log_file_path}")
                
                self.get_logger().info("Waypoint mission initiated with fresh data. Starting arming sequence...")
                self.get_logger().info("Mission will: ARM -> TAKEOFF -> NAVIGATE WAYPOINTS -> RETURN HOME -> LAND")
                response.success = True
                waypoint_count = len(self.navigation_helper.waypoints) if self.navigation_helper else 0
                response.message = f"Waypoint mission started with fresh config and {waypoint_count} waypoints"
                
            elif self.flight_state == "LANDING":
                response.success = False
                response.message = "Cannot start mission while landing. Wait for landing to complete."
            else:
                response.success = False
                response.message = f"Cannot start mission, current state: {self.flight_state}. Use /drone/land to abort current operation."
        except Exception as e:
            response.success = False
            response.message = f"Waypoint mission failed: {str(e)}"
        return response
    
    def __del__(self):
        """Destructor to ensure file is closed"""
        if self.position_log_file:
            self.position_log_file.close()


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleDroneController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        # Ensure logging file is closed before shutdown
        if node.position_log_file:
            node.position_log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()