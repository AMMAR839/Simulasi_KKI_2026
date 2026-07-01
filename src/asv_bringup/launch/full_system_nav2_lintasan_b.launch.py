"""Full Nav2, perception, and two-lap mission for KKI 2026 course B."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    show_lidar_visual = LaunchConfiguration("show_lidar_visual")
    auto_mode = LaunchConfiguration("auto_mode")
    use_navsat = LaunchConfiguration("use_navsat")
    enable_lidar_pointcloud = LaunchConfiguration("enable_lidar_pointcloud")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay_s")
    mission_start_delay = LaunchConfiguration("mission_start_delay_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("show_lidar_visual", default_value="false"),
            DeclareLaunchArgument("auto_mode", default_value="true"),
            DeclareLaunchArgument("use_navsat", default_value="false"),
            DeclareLaunchArgument("enable_lidar_pointcloud", default_value="false"),
            DeclareLaunchArgument("nav2_start_delay_s", default_value="35.0"),
            DeclareLaunchArgument("mission_start_delay_s", default_value="50.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("asv_bringup"), "launch", "sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "world": PathJoinSubstitution(
                        [
                            FindPackageShare("asv_gazebo"),
                            "worlds",
                            "kki_2026_lintasan_b.sdf",
                        ]
                    ),
                    "spawn_x": "-10.8",
                    "spawn_y": "-8.7",
                    "spawn_z": "0.02",
                    "spawn_yaw": "1.9011",
                    "world_name": "kki_2026_lintasan_b",
                    "auto_mode": auto_mode,
                    "use_mux": "true",
                    "mux_auto_topic": "/cmd_vel_auto",
                    "mux_output_topic": "/cmd_vel_selected",
                    "show_lidar_visual": show_lidar_visual,
                    "headless": headless,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("asv_bringup"), "launch", "sensors.launch.py"]
                    )
                ),
                launch_arguments={
                    "enable_lidar_pointcloud": enable_lidar_pointcloud,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("asv_bringup"),
                            "launch",
                            "navigation_nav2.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "waypoint_file": PathJoinSubstitution(
                        [
                            FindPackageShare("asv_navigation"),
                            "config",
                            "kki_waypoints_b.yaml",
                        ]
                    ),
                    "course": "b",
                    "use_sim_time": "true",
                    "nav2_start_delay_s": nav2_start_delay,
                    "mission_start_delay_s": mission_start_delay,
                    "use_navsat": use_navsat,
                }.items(),
            ),
        ]
    )
