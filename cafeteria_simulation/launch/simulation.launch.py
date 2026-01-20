import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def _make_gazebo_action(context, *args, **kwargs):

    gazebo_params_file = kwargs["gazebo_params_file"]
    world_arg = LaunchConfiguration("world").perform(context).strip()

    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py",
    )

    extra_args = "--ros-args --params-file " + gazebo_params_file

    if not world_arg:
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"extra_gazebo_args": extra_args}.items(),
            )
        ]

    if not world_arg.endswith(".world"):
        world_arg = world_arg + ".world"

    sim_share = get_package_share_directory("cafeteria_simulation")
    candidate_world_path = os.path.join(sim_share, "worlds", world_arg)

    if not os.path.isfile(candidate_world_path):
        print(
            f"[launch] World file not found: {candidate_world_path}\n"
            f"[launch] Falling back to empty world."
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"extra_gazebo_args": extra_args}.items(),
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world": candidate_world_path,
                "extra_gazebo_args": extra_args,
            }.items(),
        )
    ]



def generate_launch_description():
    package_name = "waiter_robot_description"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory("cafeteria_simulation"),'config','gazebo_params.yaml')

    twist_mux_params = os.path.join(get_package_share_directory("cafeteria_simulation"),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "waiter_robot"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="",
                description=(
                    "World filename under cafeteria_simulation/worlds (e.g. cafeteria.world). "
                    "If empty or not found, Gazebo opens the empty world."
                ),
            ),
            rsp,
            twist_mux,
            OpaqueFunction(function=_make_gazebo_action, kwargs={"gazebo_params_file": gazebo_params_file}),
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner
        ]
    )
