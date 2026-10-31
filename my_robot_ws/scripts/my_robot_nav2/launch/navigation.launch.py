import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # --- Yolları Tanımla ---
    # Not: Bu scripti doğrudan çalıştıracağın için tam yol veriyoruz.
    # Normalde paket ismiyle alınır ama şu an geliştirme aşamasındasın.
    
    base_path = "/home/jetson/Desktop/my_robot_ws/scripts/my_robot_nav2"
    map_file = os.path.join(base_path, 'maps', 'colored_map.yaml')
    params_file = os.path.join(base_path, 'params', 'nav2_params.yaml')
    
    # Nav2'nin kendi hazır launch dosyasını bul
    # (Bilgisayarında kurulu olan ros-humble-nav2-bringup paketinden çeker)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # --- Argümanlar ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Simülasyon zamanı kullanılsın mı? (Gazebo için Evet)')

    # --- Nav2'yi Başlatma ---
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True' # True yaparsan Lifecycle manager otomatik başlar
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        bringup_cmd
    ])
