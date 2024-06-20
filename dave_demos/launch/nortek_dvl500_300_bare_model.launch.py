from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the launch arguments with default values
    args = [
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="empty.sdf",
            description="Gazebo world file to launch",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Flag to indicate whether to use simulation",
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="nortek_dvl500_300_bare_model",
            description="Name of the model to load",
        ),
    ]

    use_sim = LaunchConfiguration("use_sim")
    model_name = LaunchConfiguration("model_name")
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")

    # Include the first launch file
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            (
                "gz_args",
                [
                    "-v",
                    "4",
                    " ",
                    "-r",
                    " ",
                    gazebo_world_file,
                ],
            ),
        ],
        condition=IfCondition(use_sim),
    )

    model_launch_file = PythonExpression(["'upload_' + '", model_name, "' + '.launch.py'"])

    # Include the second launch file with model name
    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("dave_model_description"),
                        "launch",
                        model_name,
                        model_launch_file,
                    ]
                )
            ]
        ),
        launch_arguments={
            "model_name": model_name,
            "use_sim": use_sim,
        }.items(),
    )

    include = [gz_sim_launch, model_launch]

    return LaunchDescription(args + include)
