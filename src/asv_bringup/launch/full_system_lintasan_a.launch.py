from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_mode = LaunchConfiguration("auto_mode")
    use_joystick = LaunchConfiguration("use_joystick")
    show_lidar_visual = LaunchConfiguration("show_lidar_visual")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_mode",
                default_value="true",
                description="true starts autonomous waypoint mode; false starts manual joystick mode.",
            ),
            DeclareLaunchArgument(
                "use_joystick",
                default_value="false",
                description="Also start joystick nodes in auto mode. Manual mode starts joystick automatically.",
            ),
            DeclareLaunchArgument(
                "show_lidar_visual",
                default_value="true",
                description="Show Gazebo LiDAR rays. Set false to hide rays while keeping the sensor active.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("asv_bringup"), "launch", "full_system.launch.py"]
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
                    "waypoint_file": PathJoinSubstitution(
                        [
                            FindPackageShare("asv_navigation"),
                            "config",
                            "kki_waypoints_a.yaml",
                        ]
                    ),
                    "spawn_x": "10.8",
                    "spawn_y": "-8.7",
                    "spawn_z": "0.12",
                    "spawn_yaw": "1.2405",
                    "world_name": "kki_2026_lintasan_a",
                    "course": "a",
                    "auto_mode": auto_mode,
                    "use_joystick": use_joystick,
                    "show_lidar_visual": show_lidar_visual,
                }.items(),
            )
        ]
    )
