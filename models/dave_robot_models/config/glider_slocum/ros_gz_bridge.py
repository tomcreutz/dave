from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    glider_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/glider_slocum/battery/battery/state@sensor_msgs/msg/BatteryState@gz.msgs.BatteryState",
            "/model/glider_slocum/joint/propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
            "/model/glider_slocum/joint/propeller_joint/ang_vel@std_msgs/msg/Float64@gz.msgs.Double",
            "/model/glider_slocum/joint/propeller_joint/enable_deadband@std_msgs/msg/Bool@gz.msgs.Boolean",
            "/model/glider_slocum/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/model/glider_slocum/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance",
            "/model/glider_slocum/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
            "/model/glider_slocum/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/model/glider_slocum/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen",
    )

    return LaunchDescription([glider_bridge])
