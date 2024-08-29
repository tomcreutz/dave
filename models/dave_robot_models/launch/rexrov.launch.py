from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    thruster_joints = [
        "/model/rexrov/joint/thruster1_joint",
        "/model/rexrov/joint/thruster2_joint",
        "/model/rexrov/joint/thruster3_joint",
        "/model/rexrov/joint/thruster4_joint",
        "/model/rexrov/joint/thruster5_joint",
        "/model/rexrov/joint/thruster6_joint",
        "/model/rexrov/joint/thruster7_joint",
        "/model/rexrov/joint/thruster8_joint",
    ]

    rexrov_arguments = (
        [
            f"{joint}/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double"
            for joint in thruster_joints
        ]
        + [
            f"{joint}/ang_vel@std_msgs/msg/Float64@gz.msgs.Double"
            for joint in thruster_joints
        ]
        + [
            f"{joint}/enable_deadband@std_msgs/msg/Bool@gz.msgs.Boolean"
            for joint in thruster_joints
        ]
        + [
            "/model/rexrov/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/model/rexrov/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance",
            "/model/rexrov/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
            "/model/rexrov/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/model/rexrov/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer",
        ]
    )

    rexrov_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=rexrov_arguments,
        output="screen",
    )

    return LaunchDescription([rexrov_bridge])
