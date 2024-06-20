from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "model_name",
            default_value="mossy_cinder_block",
            description="Name of the model to load",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Flag to indicate whether to use simulation",
        ),
    ]

    model_name = LaunchConfiguration("model_name")
    use_sim = LaunchConfiguration("use_sim")

    description_file = PathJoinSubstitution(
        [
            FindPackageShare("dave_model_description"),
            "description",
            model_name,
            "model.sdf",
        ]
    )

    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", model_name, "-file", description_file],
        output="both",
        condition=IfCondition(use_sim),
        parameters=[{"use_sim_time": use_sim}],
    )

    nodes = [gz_spawner]

    return LaunchDescription(args + nodes)
