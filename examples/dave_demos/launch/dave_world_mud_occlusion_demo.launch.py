import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the directory of the dave_worlds package
    pkg_dave_worlds = get_package_share_directory('dave_worlds')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the world file
    world_file_name = 'dave_mud_occlusion_demo.world'  # Replace with your world file
    world_path = os.path.join(pkg_dave_worlds, 'worlds', world_file_name)

    # Gazebo simulation launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    return LaunchDescription([
        gz_sim
    ])
