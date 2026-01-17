#!/usr/bin/env python3
"""
Master Launch File for ITU Cafeteria Robot

This launch file starts the complete robot system:
1. Gazebo simulation with cafeteria world
2. TurtleBot3 robot with sensors
3. Nav2 navigation stack
4. Robot State Machine (FSM brain)

Usage:
    ros2 launch cafeteria_robot_fsm master.launch.py

Optional Arguments:
    use_sim_time:=true      Use simulation time (default: true)
    world:=<path>           Custom world file
    map:=<path>             Navigation map file

Author: Hakan
Date: 2026-01-17
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the master launch description."""
    
    # ==================== PACKAGE PATHS ====================
    pkg_cafeteria_sim = get_package_share_directory('cafeteria_simulation')
    pkg_cafeteria_fsm = get_package_share_directory('cafeteria_robot_fsm')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # ==================== LAUNCH ARGUMENTS ====================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # TurtleBot3 model
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # World file path
    world_file = os.path.join(
        pkg_cafeteria_sim, 'worlds', 'med_cafeteria_mapping.world'
    )
    
    # URDF path
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        f'turtlebot3_{turtlebot3_model}.urdf'
    )
    
    # ==================== DECLARE ARGUMENTS ====================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    # ==================== 1. GAZEBO SIMULATION ====================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
        }.items()
    )
    
    # ==================== 2. SPAWN ROBOT ====================
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.01',
        }.items()
    )
    
    # ==================== 3. ROBOT STATE PUBLISHER ====================
    from launch.substitutions import Command
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time,
        }]
    )
    
    # ==================== 4. NAV2 NAVIGATION (Optional) ====================
    # Note: Nav2 requires a map. If you don't have one, comment this out
    # and use SLAM instead to create one
    
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'map': '<path_to_your_map.yaml>',
    #     }.items()
    # )
    
    # ==================== 5. ROBOT STATE MACHINE (FSM) ====================
    # Start FSM after a delay to allow Gazebo to initialize
    robot_fsm_node = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to start
        actions=[
            LogInfo(msg='🤖 Starting Robot State Machine...'),
            Node(
                package='cafeteria_robot_fsm',
                executable='robot_state_machine',
                name='robot_state_machine',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }],
                # Remap topics if needed
                remappings=[
                    # ('/cmd_vel', '/cmd_vel'),
                    # ('/robot_state', '/robot_state'),
                ],
            )
        ]
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        
        # Log startup
        LogInfo(msg=''),
        LogInfo(msg='═══════════════════════════════════════════════════════════'),
        LogInfo(msg='  🍽️  ITU CAFETERIA ROBOT - MASTER LAUNCH'),
        LogInfo(msg='═══════════════════════════════════════════════════════════'),
        LogInfo(msg=''),
        
        # 1. Start Gazebo with cafeteria world
        LogInfo(msg='📦 Starting Gazebo simulation...'),
        gazebo_launch,
        
        # 2. Spawn the robot
        LogInfo(msg='🤖 Spawning TurtleBot3...'),
        spawn_robot,
        
        # 3. Robot state publisher
        robot_state_publisher,
        
        # 4. Robot State Machine (delayed start)
        robot_fsm_node,
        
        # 5. Final log
        LogInfo(msg=''),
        LogInfo(msg='✅ All nodes launched! Waiting for initialization...'),
        LogInfo(msg='   Use the operator panel to control the robot:'),
        LogInfo(msg='   ros2 run cafeteria_robot_fsm operator_panel'),
        LogInfo(msg=''),
    ])
