import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Yolları Tanımla ---
    pkg_nav2 = '/home/gauss/Desktop/my_robot_ws/scripts/my_robot_nav2'
    pkg_desc = '/home/gauss/Desktop/my_robot_ws/src/my_robot_description'
    
    # Dosya Yolları
    world_file = os.path.join(pkg_desc, 'worlds', 'final_boat_hydro.sdf') # Senin World dosyan
    
    # DİKKAT: URDF dosyanın tam adını ve yolunu buraya yazmalısın!
    # Örnek: urdf_file = os.path.join(pkg_desc, 'urdf', 'boat.urdf')
    # Eğer xacro ise xacro olarak işlemen gerekir. Şimdilik düz urdf varsayıyorum.
    urdf_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf') 

    # 1. Gazebo Simülasyonu
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 2. ROS-GZ Bridge
    # TF topic'ini köprülemek ÇOK ÖNEMLİ. Gazebo'daki Odom TF'i buradan akar.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V', # <--- KRİTİK HAT
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image', # Kamera
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image' # Derinlik
        ],
        output='screen'
    )

    # 3. Robot State Publisher (URDF'i okur, iskeleti kurar)
    # Lidar, Kamera vs robotun neresinde bilgisini bu verir.
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description]
    )

    # 4. Map -> Odom Bağlantısı (Deniz Modu)
    # Denizde duvar olmadığı için AMCL kullanılamaz. 
    # Robotun GPS ile map üzerinde tam yerini bildiğini varsayıyoruz.
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # 5. Nav2 Başlatma
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # 6. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'config.rviz')],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        map_tf,
        navigation,
        rviz
    ])
