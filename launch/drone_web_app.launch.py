#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    """Launch just the drone web application"""
    
    return LaunchDescription([
        # Launch the web application node
        Node(
            package='freefly_mission_pkg',
            executable='drone_web_app',
            name='drone_web_app',
            output='screen',
            parameters=[
                {'web_port': 5000},
                {'web_host': '0.0.0.0'}
            ]
        )
        
    ]) 