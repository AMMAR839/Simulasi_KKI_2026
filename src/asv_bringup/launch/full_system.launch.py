from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def include_launch(package, filename, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_navigation = LaunchConfiguration("use_navigation")
    use_joystick = LaunchConfiguration("use_joystick")
    auto_mode = LaunchConfiguration("auto_mode")
    world = LaunchConfiguration("world")
    waypoint_file = LaunchConfiguration("waypoint_file")
    course = LaunchConfiguration("course")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    world_name = LaunchConfiguration("world_name")
    show_lidar_visual = LaunchConfiguration("show_lidar_visual")
    headless = LaunchConfiguration("headless")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay_s")
    mission_start_delay = LaunchConfiguration("mission_start_delay_s")
    joystick_condition = IfCondition(
        PythonExpression(
            [
                "'",
                use_joystick,
                "'.lower() in ['true', '1', 'yes'] or '",
                auto_mode,
                "'.lower() in ['false', '0', 'no']",
            ]
            
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_navigation", default_value="true"),
            DeclareLaunchArgument(
                "auto_mode",
                default_value="true",
                description="true starts autonomous waypoint mode; false starts manual joystick mode.",
            ),
            DeclareLaunchArgument(
                "use_joystick",
                default_value="false",
                description="Also start joystick nodes while in auto. Manual startup starts joystick automatically.",
            ),
            DeclareLaunchArgument("course", default_value="a"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("asv_gazebo"),
                        "worlds",
                        "kki_2026_lintasan_a.sdf",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("asv_navigation"),
                        "config",
                        "kki_waypoints_a.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="10.8"),
            DeclareLaunchArgument("spawn_y", default_value="-8.7"),
            DeclareLaunchArgument("spawn_z", default_value="0.12"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.2405"),
            DeclareLaunchArgument("world_name", default_value="kki_2026_lintasan_a"),
            DeclareLaunchArgument("navigation_start_delay_s", default_value="35.0"),
            DeclareLaunchArgument("mission_start_delay_s", default_value="45.0"),

            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Gazebo server without GUI.",
            ),
            DeclareLaunchArgument(
                "show_lidar_visual",
                default_value="true",
                description="Show Gazebo LiDAR rays. Set false to hide rays while keeping /asv/lidar/scan active.",
            ),
            include_launch(
                "asv_bringup",
                "sim.launch.py",
                launch_arguments={
                    "world": world,
                    "spawn_x": spawn_x,
                    "spawn_y": spawn_y,
                    "spawn_z": spawn_z,
                    "spawn_yaw": spawn_yaw,
                    "world_name": world_name,
                    "auto_mode": auto_mode,
                    "show_lidar_visual": show_lidar_visual,
                    "headless": headless,
                },
            ),
            include_launch("asv_bringup", "sensors.launch.py"),
            include_launch(
                "asv_bringup",
                "navigation.launch.py",
                launch_arguments={
                    "waypoint_file": waypoint_file,
                    "course": course,
                    "navigation_start_delay_s": navigation_start_delay,
                    "mission_start_delay_s": mission_start_delay,
                },
                condition=IfCondition(use_navigation),
            ),
            include_launch(
                "asv_bringup",
                "joystick.launch.py",
                condition=joystick_condition,
            ),
        ]
    )
