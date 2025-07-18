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
        )
    ]) 