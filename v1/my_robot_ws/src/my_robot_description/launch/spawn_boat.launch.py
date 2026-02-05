import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('my_robot_description')
    world_file = os.path.join(pkg_dir, 'worlds', 'boat_world.sdf')
    model_file = os.path.join(pkg_dir, 'models', 'boat', 'model.sdf')
    
    # Gazebo'yu başlat
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )
    
    # 5 saniye bekle, sonra robotu spawn et
    spawn_boat = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/boat_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', f'sdf_filename: "{model_file}", name: "boat", pose: {{position: {{x: 0, y: 0, z: 0.15}}}}'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gz_sim,
        spawn_boat
    ])
