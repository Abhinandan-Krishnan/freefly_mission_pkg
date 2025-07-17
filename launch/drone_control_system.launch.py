#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    """Launch the complete drone control system with web interface"""
    
    return LaunchDescription([
        # Launch the drone navigation node
        Node(
            package='freefly_mission_pkg',
            executable='drone_nav_node',
            name='drone_nav_node',
            output='screen',
            parameters=[
                {'config_file': 'takeoff_config.yaml'},
                {'waypoints_file': 'waypoints.txt'}
            ],
            remappings=[
                # Add any topic remappings if needed
            ]
        ),
        
        # Launch the web application node
        Node(
            package='freefly_mission_pkg',
            executable='drone_web_app',
            name='drone_web_app',
            output='screen',
            parameters=[
                # Add any web app parameters if needed
            ]
        ),
        
        # Optionally open browser after 3 seconds
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 3 && xdg-open http://localhost:5000'],
            output='screen'
        )
    ]) 