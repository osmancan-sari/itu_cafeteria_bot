from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = FindPackageShare('waiter_robot_description')
    model_path = PathJoinSubstitution([pkg_share, 'urdf', 'waiter_robot.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'urdf', 'waiter_robot_display.rviz'])

    
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    
    ld.add_action(DeclareLaunchArgument(name='model', default_value=model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'waiter_robot_description',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    return ld