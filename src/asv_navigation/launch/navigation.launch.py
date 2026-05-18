from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoint_file = LaunchConfiguration("waypoint_file")
    course = LaunchConfiguration("course")
    use_robot_localization = LaunchConfiguration("use_robot_localization")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav_share = FindPackageShare("asv_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=PathJoinSubstitution(
                    [nav_share, "config", "kki_waypoints.yaml"]
                ),
            ),
            DeclareLaunchArgument("course", default_value="a"),
            DeclareLaunchArgument("use_robot_localization", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="asv_navigation",
                executable="gps_waypoint_follower",
                name="gps_waypoint_follower",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "waypoint_file": waypoint_file,
                        "max_speed_mps": 0.62,
                        "reach_radius_m": 0.65,
                        "segment_pass_margin_m": 0.35,
                        "gate_cross_track_radius_m": 0.75,
                        "lidar_process_every_n_scans": 3,
                        "lidar_sample_stride": 3,
                        "lidar_detection_max_distance_m": 2.5,
                    }
                ],
            ),
            Node(
                package="asv_navigation",
                executable="mission_supervisor",
                name="mission_supervisor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "course": course,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_robot_localization),
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav_share, "config", "ekf.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                condition=IfCondition(use_robot_localization),
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav_share, "config", "navsat_transform.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("imu", "/asv/imu/data"),
                    ("gps/fix", "/asv/gps/fix"),
                    ("odometry/filtered", "/odometry/filtered"),
                ],
            ),
        ]
    )
