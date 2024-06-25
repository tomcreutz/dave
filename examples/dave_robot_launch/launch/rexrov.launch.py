from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")
    paused = LaunchConfiguration("paused")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    debug = LaunchConfiguration("debug")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    verbose = LaunchConfiguration("verbose")
    namespace = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    # Declare the launch arguments with default values
    args = [
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="empty.sdf",
            description="Gazebo world file to launch",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Flag to indicate whether to use simulation time",
        ),
        DeclareLaunchArgument(
            "paused",
            default_value="false",
            description="Start the simulation paused",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Flag to enable the gazebo gui",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Flag to enable the gazebo debug flag",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Flag to enable the gazebo headless mode",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Flag to enable verbose mode in gazebo",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="rexrov",
            description="Namespace",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="4",
            description="Initial x position",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="4",
            description="Initial y position",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="-93",
            description="Initial z position",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="-1.8",
            description="Initial yaw angle",
        ),
    ]

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
                    "-r ",
                    gazebo_world_file,
                ],
            ),
        ],
        condition=IfCondition(gui),
    )

    # Include the second launch file with model name
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("dave_robot_models"),
                        "launch",
                        "upload_rexrov.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "namespace": namespace,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
        }.items(),
    )

    include = [gz_sim_launch, robot_launch]

    return LaunchDescription(args + include)
