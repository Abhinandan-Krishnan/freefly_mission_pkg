#!/usr/bin/env python3

import yaml
import math
from typing import List, Tuple, Optional


class DroneUtilities:
    """Utility class containing helper functions for drone navigation"""
    
    @staticmethod
    def load_config(config_file: str, logger=None) -> dict:
        """Load drone configuration from YAML file"""
        default_config = {
            'takeoff_height': 5.0,
            'max_horizontal_speed': 3.0,
            'max_vertical_speed': 1.5,
            'takeoff_speed': 2.0,
            'landing_speed': 1.0,
            'waypoint_tolerance': 1.0,
            'approach_distance': 5.0,
            'min_speed': 0.2
        }
        
        try:
            with open(config_file, 'r') as file:
                if config_file.endswith(('.yaml', '.yml')):
                    config = yaml.safe_load(file)
                    # Merge with defaults
                    for key, default_value in default_config.items():
                        if key not in config:
                            config[key] = default_value
                else:
                    # Simple text file with just the height
                    content = file.read().strip()
                    config = default_config.copy()
                    config['takeoff_height'] = float(content)
                
                if logger:
                    logger.info(f"Loaded configuration from {config_file}")
                    logger.info(f"Takeoff height: {config['takeoff_height']}m")
                    logger.info(f"Navigation speeds - H:{config['max_horizontal_speed']}m/s V:{config['max_vertical_speed']}m/s")
                
                return config
                
        except Exception as e:
            if logger:
                logger.error(f"Error loading config: {e}, using defaults")
            return default_config
    
    @staticmethod
    def load_waypoints(waypoints_file: str, logger=None) -> List[List[float]]:
        """Load waypoints from text file"""
        waypoints = []
        
        try:
            with open(waypoints_file, 'r') as file:
                for line in file:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split()
                        if len(parts) >= 3:
                            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                            waypoints.append([x, y, -z])  # Convert to NED frame (z negative)
                
                if logger:
                    logger.info(f"Loaded {len(waypoints)} waypoints from {waypoints_file}")
                    for i, wp in enumerate(waypoints):
                        logger.info(f"Waypoint {i+1}: [{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}]")
                
        except Exception as e:
            if logger:
                logger.error(f"Error loading waypoints: {e}")
            waypoints = []
        
        return waypoints
    
    @staticmethod
    def calculate_distance(pos1: List[float], pos2: List[float]) -> float:
        """Calculate 3D distance between two positions"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)
    
    @staticmethod
    def calculate_velocity_vector(current_pos: List[float], 
                                target_pos: List[float], 
                                flight_state: str,
                                config: dict,
                                logger=None) -> List[float]:
        """Calculate desired velocity vector for smooth navigation with speed control"""
        # Calculate position difference
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1] 
        dz = target_pos[2] - current_pos[2]
        
        # Calculate distances
        horizontal_distance = math.sqrt(dx**2 + dy**2)
        total_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # If very close to target, use minimal velocity
        if total_distance < 0.1:
            return [0.0, 0.0, 0.0]
        
        # Determine speed based on flight state
        if flight_state == "TAKEOFF":
            max_speed = config['takeoff_speed']
        elif flight_state == "LANDING":
            max_speed = config['landing_speed']
        else:
            # For waypoint navigation and return home
            max_speed = config['max_horizontal_speed']
        
        # Apply speed reduction when approaching target
        original_speed = max_speed
        if total_distance < config['approach_distance']:
            speed_factor = max(total_distance / config['approach_distance'], 
                             config['min_speed'] / max_speed)
            max_speed *= speed_factor
        
        # Ensure minimum speed
        final_speed = max(max_speed, config['min_speed'])
        if final_speed != max_speed and logger:
            logger.info(f"MIN_SPEED: Adjusted {max_speed:.1f}→{final_speed:.1f}m/s")
        
        # Calculate unit direction vector
        if total_distance > 0:
            unit_x = dx / total_distance
            unit_y = dy / total_distance
            unit_z = dz / total_distance
        else:
            return [0.0, 0.0, 0.0]
        
        # Apply speed limits using final calculated speed
        vx = unit_x * final_speed
        vy = unit_y * final_speed
        
        # Separate vertical speed limit
        if flight_state in ["TAKEOFF", "LANDING"]:
            vz = unit_z * min(final_speed, config['max_vertical_speed'])
        else:
            vz = unit_z * min(final_speed, config['max_vertical_speed'])
        
        return [vx, vy, vz]


class NavigationHelper:
    """Helper class for waypoint navigation logic"""
    
    def __init__(self, waypoints: List[List[float]], tolerance: float = 1.0):
        self.waypoints = waypoints
        self.tolerance = tolerance
        self.current_index = 0
        self.is_mission_complete = False
    
    def get_current_waypoint(self) -> Optional[List[float]]:
        """Get the current target waypoint"""
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None
    
    def check_waypoint_reached(self, current_pos: List[float]) -> bool:
        """Check if current waypoint is reached"""
        if self.current_index >= len(self.waypoints):
            return False
        
        current_waypoint = self.waypoints[self.current_index]
        distance = DroneUtilities.calculate_distance(current_pos, current_waypoint)
        return distance < self.tolerance
    
    def advance_to_next_waypoint(self) -> Tuple[bool, Optional[List[float]]]:
        """
        Advance to next waypoint
        Returns: (has_next_waypoint, next_waypoint_or_none)
        """
        self.current_index += 1
        
        if self.current_index >= len(self.waypoints):
            self.is_mission_complete = True
            return False, None
        
        return True, self.waypoints[self.current_index]
    
    def reset_mission(self):
        """Reset mission to start from first waypoint"""
        self.current_index = 0
        self.is_mission_complete = False
    
    def get_mission_progress(self) -> dict:
        """Get current mission progress information"""
        return {
            'current_waypoint': self.current_index + 1,
            'total_waypoints': len(self.waypoints),
            'is_complete': self.is_mission_complete,
            'progress_percentage': (self.current_index / len(self.waypoints)) * 100 if self.waypoints else 0
        } 