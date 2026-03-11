import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg = get_package_share_directory('boat_navigation')
    nav2_params_file = os.path.join(pkg, 'params', 'nav2_params.yaml')
    map_file         = os.path.join(pkg, 'maps',   'costmap.yaml')

    # 1) GZ <-> ROS Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        remappings=[('/lidar', '/scan')],
        parameters=[{'use_sim_time': False}] 
    )

    cmd_vel_to_thrust = Node(
        package='boat_navigation',
        executable='cmd_vel_to_thrust.py',
        name='cmd_vel_to_thrust',
        output='screen',
    )

    # 2) Statik Map -> Odom TF (AMCL yerine bunu kullanıyoruz)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3) robot_state_publisher
    urdf_file = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc, 'publish_frequency': 30.0}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 4) Nav2 stack (AMCL Çıkarıldı)
    map_server = Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_file}])
    controller_server = Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[nav2_params_file, {'use_sim_time': use_sim_time}], remappings=[('cmd_vel', 'cmd_vel')])
    planner_server = Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])
    behavior_server = Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])
    bt_navigator = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator'],
        }],
    )

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', parameters=[{'use_sim_time': use_sim_time}])

    # 5) TF ve Ekstra Düğümler
    oak_camera_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='oak_camera_static_tf', arguments=['0.5', '0.0', '0.3', '0', '0', '0', 'base_link', 'oak_rgb_camera_optical_frame'], parameters=[{'use_sim_time': use_sim_time}])
    gazebo_lidar_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='gazebo_lidar_tf', arguments=['0', '0', '0', '0', '0', '0', 'gpu_lidar', 'boat/base_link/gpu_lidar'], parameters=[{'use_sim_time': use_sim_time}])
    
    object_detect_node = ExecuteProcess(cmd=['python3', 'pcl.py'], cwd='/home/gauss/Desktop/my_robot_ws/src/objectdetect/objectdetect', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_bridge, cmd_vel_to_thrust, map_to_odom_tf, robot_state_publisher, joint_state_publisher,
        oak_camera_tf, gazebo_lidar_tf, object_detect_node,
        TimerAction(period=3.0, actions=[map_server, controller_server, planner_server, behavior_server, bt_navigator, lifecycle_manager]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
