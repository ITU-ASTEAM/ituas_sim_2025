import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # --- Yollar ---
    pkg_nav2 = '/home/jetson/Desktop/my_robot_ws/scripts/my_robot_nav2'
    pkg_desc = '/home/jetson/Desktop/my_robot_ws/src/my_robot_description'

    # Dosyalar
    world_file = os.path.join(pkg_desc, 'worlds', 'final_boat_hydro.sdf')
    urdf_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf')

    # 1. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 2. Bridge
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    output='screen',
    arguments=[
        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

        # Model-specific odometry
        '/model/boat/odometry_with_covariance@nav_msgs/msg/Odometry@ignition.msgs.OdometryWithCovariance',

        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
    ],
    remappings=[
        ('/model/boat/odometry_with_covariance', '/odom')
    ]
    )



    # 3. Robot State Publisher
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description]
    )

    # 4. Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # 5. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        navigation,
        rviz
    ])

