from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    urdf_file = "/home/beequeen/Downloads/my_robot/urdf/robot.urdf"

    return LaunchDescription([
        # Gazebo sim başlat
        ExecuteProcess(
            cmd=["gz", "sim", "-v", "8"],
            output="screen"
        ),
        # Robotu spawn et
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz", "create",
                "-file", urdf_file,
                "-name", "my_robot",
                "-x", "0", "-y", "0", "-z", "0.1"
            ],
            output="screen"
        )
    ])
