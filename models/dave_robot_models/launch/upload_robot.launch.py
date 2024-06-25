from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    use_sim = LaunchConfiguration("use_sim")
    namespace = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    use_ned_frame = LaunchConfiguration("use_ned_frame")

    args = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Flag to indicate whether to use simulation",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0",
            description="Initial x position",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0",
            description="Initial y position",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="-20",
            description="Initial z position",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="Initial roll",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="Initial pitch",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Initial yaw",
        ),
        DeclareLaunchArgument(
            "use_ned_frame",
            default_value="false",
            description="Use NED frame",
        ),
    ]

    description_file = PathJoinSubstitution(
        [
            FindPackageShare("dave_robot_models"),
            "description",
            namespace,
            "model.sdf",
        ]
    )

    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", namespace, "-file", description_file],
        output="both",
        condition=IfCondition(use_sim),
        parameters=[{"use_sim_time": use_sim}],
    )

    nodes = [gz_spawner]

    event_handlers = [
        RegisterEventHandler(
            OnProcessExit(target_action=gz_spawner, on_exit=LogInfo(msg="Robot Model Uploaded"))
        )
    ]

    return LaunchDescription(args + nodes + event_handlers)
