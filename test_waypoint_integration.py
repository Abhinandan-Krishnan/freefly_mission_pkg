#!/usr/bin/env python3

"""
Test script to verify waypoint integration between webapp and drone navigation node
"""

import os
import sys

# Add the package to the path
sys.path.append('/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/freefly_mission_pkg')

from drone_utilities import DroneUtilities

def test_waypoint_loading():
    """Test waypoint loading functionality"""
    
    # Test data directory and file
    test_data_dir = "/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/data/waypoints"
    waypoint_file = os.path.join(test_data_dir, "waypoints.txt")
    
    print("=== Simplified Waypoint Integration Test ===")
    print(f"Testing waypoint file: {waypoint_file}")
    
    # Check if directory exists
    if not os.path.exists(test_data_dir):
        print(f"❌ Waypoints directory does not exist: {test_data_dir}")
        print("Creating directory...")
        os.makedirs(test_data_dir, exist_ok=True)
        print("✅ Directory created")
    else:
        print("✅ Waypoints directory exists")
    
    # Check if waypoint file exists
    if os.path.exists(waypoint_file):
        print(f"✅ Waypoint file exists: {waypoint_file}")
        
        # Test loading the file
        try:
            waypoints = DroneUtilities.load_waypoints(waypoint_file)
            print(f"✅ Loaded {len(waypoints)} waypoints")
            for i, wp in enumerate(waypoints[:3]):  # Show first 3 waypoints
                print(f"  Waypoint {i+1}: [{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}]")
            if len(waypoints) > 3:
                print(f"  ... and {len(waypoints) - 3} more waypoints")
        except Exception as e:
            print(f"❌ Error loading waypoints: {e}")
    else:
        print("⚠️  No waypoint file found")
        print("   Upload a waypoint file through the webapp to test integration")
    
    # Test sample waypoint file
    sample_file = "/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/data/sample_waypoints.txt"
    if os.path.exists(sample_file):
        print(f"\n✅ Sample waypoint file exists: {sample_file}")
        try:
            waypoints = DroneUtilities.load_waypoints(sample_file)
            print(f"✅ Sample file loaded {len(waypoints)} waypoints")
            for i, wp in enumerate(waypoints):
                print(f"  Waypoint {i+1}: [{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}]")
        except Exception as e:
            print(f"❌ Error loading sample file: {e}")
    else:
        print(f"❌ Sample waypoint file not found: {sample_file}")
    
    print("\n=== Simplified Integration Summary ===")
    print("The system now works as follows:")
    print("1. Webapp uploads waypoint file → saves as 'waypoints.txt'")
    print("2. Previous waypoint file is automatically deleted/overwritten")
    print("3. Drone nav node loads from fixed path: 'data/waypoints/waypoints.txt'")
    print("4. Only one waypoint file exists at any time")
    print("5. Supports both comma-separated (X,Y,Z) and space-separated formats")
    print("6. Converts coordinates to NED frame (Z becomes negative)")
    
    if os.path.exists(waypoint_file):
        print(f"\n✅ Integration ready! Waypoint file available for missions.")
    else:
        print("\n⚠️  Upload a waypoint file through the webapp to enable waypoint missions.")

if __name__ == "__main__":
    test_waypoint_loading() 