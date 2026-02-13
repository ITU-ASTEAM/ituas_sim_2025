from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'robot.urdf'
    )

    world_file = os.path.expanduser("~/my_robot_ws/worlds/empty.world")

    # Gazebo başlat
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
    "gz_args": "/home/beequeen/Downloads/my_robot_ws/worlds/empty.sdf"
}.items()
    )

    # 3 saniye sonra robot spawn
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file", urdf_file,
                    "-name", "my_robot",
                    "-x", "0",
                    "-y", "0",
                    "-z", "0.1"
                ],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_robot
    ])

