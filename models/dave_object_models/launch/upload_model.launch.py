from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    model_name = LaunchConfiguration("model_name")
    use_sim = LaunchConfiguration("use_sim")

    args = [
        DeclareLaunchArgument(
            "model_name",
            default_value="model",
            description="Name of the model to load",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Flag to indicate whether to use simulation",
        ),
    ]

    description_file = PathJoinSubstitution(
        [
            FindPackageShare("dave_object_models"),
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

    event_handlers = [
        RegisterEventHandler(
            OnProcessExit(target_action=gz_spawner, on_exit=LogInfo(msg="Model Uploaded"))
        )
    ]

    return LaunchDescription(args + nodes + event_handlers)
