import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg = get_package_share_directory('boat_navigation')
    nav2_params_file = os.path.join(pkg, 'params', 'nav2_params.yaml')
    map_file         = os.path.join(pkg, 'maps',   'costmap.yaml')
    bridge_config    = os.path.join(pkg, 'config', 'bridge.yaml')
    model_sdf = '/home/gauss/Desktop/my_robot_ws/src/my_robot_description/models/boat/model.sdf'

    # ------------------------------------------------------------------
    # 1) GZ→ROS Bridge — YAML config ile lazy:false  (subscriber zorla açılır)
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # 1) GZ <-> ROS Bridge (Doğrudan Argüman Yöntemi)
    # ------------------------------------------------------------------
# ------------------------------------------------------------------
    # 1) GZ <-> ROS Bridge (Doğrudan Argüman ve Deadlock Kırıcı)
    # ------------------------------------------------------------------
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
        remappings=[
            ('/lidar', '/scan')
        ],
        # DİKKAT: Ölümcül kilitlenmeyi kırmak için köprüde sim_time KESİNLİKLE False olmalı!
        parameters=[{'use_sim_time': False}] 
    )
    cmd_vel_to_thrust = Node(
        package='boat_navigation',
        executable='cmd_vel_to_thrust.py',
        name='cmd_vel_to_thrust',
        output='screen',
    )

    # ------------------------------------------------------------------
    # 3) robot_state_publisher  (base_link → pervane TF'leri)
    # ------------------------------------------------------------------
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'robot.urdf'
    )
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            'publish_frequency': 30.0,
        }],
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 4) Nav2 stack
    # ------------------------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        gz_bridge,
        cmd_vel_to_thrust,  # YENİ EKLEDİĞİMİZ SATIR
        robot_state_publisher,
        joint_state_publisher,

        TimerAction(period=3.0, actions=[
            map_server,
            amcl,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]),

        TimerAction(period=5.0, actions=[rviz]),
    ])
