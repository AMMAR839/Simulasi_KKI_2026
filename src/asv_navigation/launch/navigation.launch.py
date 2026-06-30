import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BASE_LD_LIBRARY_PATH = os.environ.get("LD_LIBRARY_PATH", "")
BASE_GZ_SIM_RESOURCE_PATH = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
CLEAN_LD_LIBRARY_PATH = EnvironmentVariable(
    "ASV_CLEAN_LD_LIBRARY_PATH", default_value=BASE_LD_LIBRARY_PATH
)
CLEAN_GZ_SIM_RESOURCE_PATH = EnvironmentVariable(
    "ASV_CLEAN_GZ_SIM_RESOURCE_PATH", default_value=BASE_GZ_SIM_RESOURCE_PATH
)


def generate_launch_description():
    waypoint_file = LaunchConfiguration("waypoint_file")
    course = LaunchConfiguration("course")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay_s")
    mission_start_delay = LaunchConfiguration("mission_start_delay_s")
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
            DeclareLaunchArgument(
                "navigation_start_delay_s",
                default_value="35.0",
                description="Delay waypoint follower startup until Gazebo, bridge, and model spawn are stable.",
            ),
            DeclareLaunchArgument(
                "mission_start_delay_s",
                default_value="45.0",
                description="Delay mission supervisor startup until navigation and controller nodes are stable.",
            ),
            DeclareLaunchArgument("use_robot_localization", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            TimerAction(
                period=navigation_start_delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "env",
                            "-u",
                            "GZ_SIM_SYSTEM_PLUGIN_PATH",
                            "-u",
                            "GZ_RENDERING_PLUGIN_PATH",
                            "ros2",
                            "run",
                            "asv_navigation",
                            "gps_waypoint_follower",
                            "--ros-args",
                            "-r",
                            "__node:=gps_waypoint_follower",
                            "-p",
                            ["use_sim_time:=", use_sim_time],
                            "-p",
                            ["waypoint_file:=", waypoint_file],
                            "-p",
                            "max_speed_mps:=0.62",
                            "-p",
                            "reach_radius_m:=0.65",
                            "-p",
                            "segment_pass_margin_m:=0.35",
                            "-p",
                            "gate_cross_track_radius_m:=1.50",
                            "-p",
                            "lidar_process_every_n_scans:=3",
                            "-p",
                            "lidar_sample_stride:=3",
                            "-p",
                            "lidar_detection_max_distance_m:=1.8",
                        ],
                        name="gps_waypoint_follower",
                        output="screen",
                        additional_env={
                            "LD_LIBRARY_PATH": CLEAN_LD_LIBRARY_PATH,
                            "GZ_SIM_RESOURCE_PATH": CLEAN_GZ_SIM_RESOURCE_PATH,
                        },
                    )
                ],
            ),
            TimerAction(
                period=mission_start_delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "env",
                            "-u",
                            "GZ_SIM_SYSTEM_PLUGIN_PATH",
                            "-u",
                            "GZ_RENDERING_PLUGIN_PATH",
                            "ros2",
                            "run",
                            "asv_navigation",
                            "mission_supervisor",
                            "--ros-args",
                            "-r",
                            "__node:=mission_supervisor",
                            "-p",
                            ["use_sim_time:=", use_sim_time],
                            "-p",
                            ["course:=", course],
                        ],
                        name="mission_supervisor",
                        output="screen",
                        additional_env={
                            "LD_LIBRARY_PATH": CLEAN_LD_LIBRARY_PATH,
                            "GZ_SIM_RESOURCE_PATH": CLEAN_GZ_SIM_RESOURCE_PATH,
                        },
                    )
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
