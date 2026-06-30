"""
full_system_nav2_lintasan_a.launch.py — Full Nav2 System for KKI 2026 Course A
================================================================================
Menggantikan full_system_lintasan_a.launch.py dengan Nav2 full-stack.

Perbedaan dari versi lama:
  - Menggunakan navigation_nav2.launch.py (Nav2) bukan navigation.launch.py (custom)
  - Nav2 BT navigator menangani path planning + recovery
  - Nav2 Collision Monitor (polygon zones) menggantikan custom collision_monitor.py
  - OpenNav Docking Server untuk docking ke buoy biru
  - robot_localization dual-EKF untuk GPS + IMU fusion

Cara menjalankan:
  ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py
  ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py headless:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    show_lidar_visual = LaunchConfiguration("show_lidar_visual")
    auto_mode = LaunchConfiguration("auto_mode")
    use_joystick = LaunchConfiguration("use_joystick")
    use_navsat = LaunchConfiguration("use_navsat")

    return LaunchDescription(
        [
            # ── Launch Arguments ──────────────────────────────
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without GUI.",
            ),
            DeclareLaunchArgument(
                "show_lidar_visual",
                default_value="false",
                description="Show Gazebo LiDAR rays.",
            ),
            DeclareLaunchArgument(
                "auto_mode",
                default_value="true",
                description="Start in autonomous mode.",
            ),
            DeclareLaunchArgument(
                "use_joystick",
                default_value="false",
                description="Also start joystick nodes.",
            ),
            DeclareLaunchArgument(
                "use_navsat",
                default_value="false",
                description="Use GPS/NavSat global EKF instead of local simulator map frame.",
            ),

            # ── Include sim.launch.py (Gazebo + robot spawn) ──
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
                            "kki_2026_lintasan_a.sdf",
                        ]
                    ),
                    "spawn_x": "10.8",
                    "spawn_y": "-8.7",
                    "spawn_z": "0.02",
                    "spawn_yaw": "1.2405",
                    "world_name": "kki_2026_lintasan_a",
                    "auto_mode": auto_mode,
                    "use_mux": "false",
                    "show_lidar_visual": show_lidar_visual,
                    "headless": headless,
                }.items(),
            ),

            # ── Include sensors.launch.py (bridge, camera, lidar) ──
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("asv_bringup"), "launch", "sensors.launch.py"]
                    )
                ),
            ),

            # ── Include navigation_nav2.launch.py (Nav2 full stack) ──
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
                            "kki_waypoints_a.yaml",
                        ]
                    ),
                    "course": "a",
                    "use_sim_time": "true",
                    "nav2_start_delay_s": "35.0",
                    "mission_start_delay_s": "50.0",
                    "use_navsat": use_navsat,
                }.items(),
            ),
        ]
    )
