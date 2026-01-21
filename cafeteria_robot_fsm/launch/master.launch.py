#!/usr/bin/env python3
"""
Master Launch File for ITU Cafeteria Robot

This launch file starts the complete robot system:
1. Gazebo simulation with cafeteria world
2. Waiter robot with sensors
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
    OpaqueFunction,
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
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_waiter_robot = get_package_share_directory('waiter_robot_description')
    
    # ==================== LAUNCH ARGUMENTS ====================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # World file path
    world_file = os.path.join(
        pkg_cafeteria_sim, 'worlds', 'med_cafeteria_mapping.world'
    )
    
    # Gazebo params file
    gazebo_params_file = os.path.join(pkg_cafeteria_sim, 'config', 'gazebo_params.yaml')
    
    # ==================== DECLARE ARGUMENTS ====================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    # ==================== 1. GAZEBO SIMULATION ====================
    def _make_gazebo_action(context, *args, **kwargs):
        gazebo_params_file = kwargs["gazebo_params_file"]
        world_path = kwargs["world_path"]
        
        gazebo_launch = os.path.join(
            get_package_share_directory("gazebo_ros"),
            "launch",
            "gazebo.launch.py",
        )
        
        extra_args = "--ros-args --params-file " + gazebo_params_file
        
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "world": world_path,
                    "extra_gazebo_args": extra_args,
                }.items(),
            )
        ]
    
    gazebo_launch = OpaqueFunction(
        function=_make_gazebo_action,
        kwargs={"gazebo_params_file": gazebo_params_file, "world_path": world_file}
    )
    
    # ==================== 2. ROBOT STATE PUBLISHER ====================
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_waiter_robot, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )
    
    # ==================== 3. SPAWN ROBOT IN GAZEBO ====================
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "waiter_robot"],
        output="screen",
    )
    
    # ==================== 4. CONTROLLER SPAWNERS ====================
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )
    
    # ==================== 5. TWIST MUX ====================
    twist_mux_params = os.path.join(pkg_cafeteria_sim, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        output="screen",
    )
    
    # ==================== 6. NAV2 NAVIGATION (Optional) ====================
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
    
    # ==================== 7. ROBOT STATE MACHINE (FSM) ====================
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
        
        # 2. Robot state publisher (publishes robot_description topic)
        LogInfo(msg='📡 Starting Robot State Publisher...'),
        robot_state_publisher,
        
        # 3. Spawn the waiter robot in Gazebo
        LogInfo(msg='🤖 Spawning Waiter Robot...'),
        spawn_entity,
        
        # 4. Start controllers
        LogInfo(msg='⚙️ Starting Controllers...'),
        diff_drive_spawner,
        joint_broad_spawner,
        
        # 5. Start twist mux
        LogInfo(msg='🔀 Starting Twist Mux...'),
        twist_mux,
        
        # 6. Robot State Machine (delayed start)
        LogInfo(msg='🧠 Starting Robot State Machine in 5 seconds...'),
        robot_fsm_node,
        
        # 5. Final log
        LogInfo(msg=''),
        LogInfo(msg='✅ All nodes launched! Waiting for initialization...'),
        LogInfo(msg='   Use the operator panel to control the robot:'),
        LogInfo(msg='   ros2 run cafeteria_robot_fsm operator_panel'),
        LogInfo(msg=''),
    ])
