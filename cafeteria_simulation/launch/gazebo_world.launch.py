import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration 

def generate_launch_dexscription():
    pkg_cafeteria_sim = get_package_share_directory('cafeteria_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # 1. Robot Model Dosyasını Bul (Xacro İşlemi İçin Hazırlık)
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file_name = 'turtlebot3_' + turtlebot3_model + '.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    # 2. World Dosyamızın yolu
    world_file_name = 'med_cafeteria_mapping.world'
    world_path = os.path.join(pkg_cafeteria_sim, 'worlds', world_file_name)

    # 3. Gazebo Başlatma
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 4. Robotu Spawn Etme (Fiziksel Robot)
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.01'
        }.items()
    )

    # 5. Robot State Publisher (Robotun Beyni)
    # DÜZELTME BURADA: Dosyayı 'xacro' komutuyla işleyerek okuyoruz.
    # Böylece ${namespace} gibi hatalı isimler temizleniyor.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher_node 
    ])