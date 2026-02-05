import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_file = os.path.join(
        os.path.expanduser('~/Downloads/my_robot_ws/worlds'),
        'ocean.sdf'  # empty.sdf yerine ocean.sdf
    )
    
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        pkg_share
    )
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )
    
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-z', '2.0'
        ],
        output='screen'
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/boat/joint/left_thruster_joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/boat/joint/right_thruster_joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])