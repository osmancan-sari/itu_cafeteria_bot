#!/usr/bin/env python3
"""
FSM-Only Launch File for ITU Cafeteria Robot

This launch file starts ONLY the Robot State Machine (FSM) node.
Use this when you already have Gazebo running separately.

Usage:
    ros2 launch cafeteria_robot_fsm fsm_only.launch.py

Author: Hakan
Date: 2026-01-17
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for FSM-only mode."""
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    # Robot State Machine node
    robot_fsm_node = Node(
        package='cafeteria_robot_fsm',
        executable='robot_state_machine',
        name='robot_state_machine',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        
        # Logs
        LogInfo(msg=''),
        LogInfo(msg='🤖 Starting Robot State Machine (FSM-only mode)...'),
        LogInfo(msg=''),
        
        # FSM Node
        robot_fsm_node,
    ])
