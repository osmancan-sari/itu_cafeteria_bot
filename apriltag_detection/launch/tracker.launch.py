from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    params_file_dec = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('apriltag_detection'),
            'config',
            'tracker_params.yaml'
        ),
        description='Full path to params file for all apriltag tracker nodes.'
    )

    detect_only = LaunchConfiguration('detect_only')
    detect_only_dec = DeclareLaunchArgument(
        'detect_only',
        default_value='false',
        description='Doesn\'t run the follow component. Useful for just testing the detections.'
    )

    follow_only = LaunchConfiguration('follow_only')
    follow_only_dec = DeclareLaunchArgument(
        'follow_only',
        default_value='false',
        description='Doesn\'t run the detect component. Useful for testing just the following.'
    )

    tune_detection = LaunchConfiguration('tune_detection')
    tune_detection_dec = DeclareLaunchArgument(
        'tune_detection',
        default_value='false',
        description='Enables tuning mode for the detection (ROI tuning).'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Enables sim time for the follow node.'
    )

    image_topic = LaunchConfiguration('image_topic')
    image_topic_dec = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='The name of the input image topic.'
    )

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_topic_dec = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel_tracker',
        description='The name of the output command vel topic.'
    )

    enable_3d_tracker = LaunchConfiguration('enable_3d_tracker')
    enable_3d_tracker_dec = DeclareLaunchArgument(
        'enable_3d_tracker',
        default_value='false',
        description='Enables the 3D tracker / marker node.'
    )

    detect_node = Node(
        package='apriltag_detection',
        executable='detect_apriltag',
        parameters=[params_file, {'tuning_mode': tune_detection}],
        remappings=[('/image_in', image_topic)],
        condition=UnlessCondition(follow_only),
        output='screen'
    )

    detect_3d_node = Node(
        package='apriltag_detection',
        executable='detect_apriltag_3d',
        parameters=[params_file],
        condition=IfCondition(enable_3d_tracker),
        output='screen'
    )

    follow_node = Node(
        package='apriltag_detection',
        executable='follow_apriltag',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', cmd_vel_topic)],
        condition=UnlessCondition(detect_only),
        output='screen'
    )

    return LaunchDescription([
        params_file_dec,
        detect_only_dec,
        follow_only_dec,
        tune_detection_dec,
        use_sim_time_dec,
        image_topic_dec,
        cmd_vel_topic_dec,
        enable_3d_tracker_dec,
        detect_node,
        detect_3d_node,
        follow_node,
    ])
