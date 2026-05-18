from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoint_file = LaunchConfiguration("waypoint_file")
    course = LaunchConfiguration("course")
    return LaunchDescription(
        [
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
            DeclareLaunchArgument("course", default_value="a"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("asv_navigation"), "launch", "navigation.launch.py"]
                    )
                ),
                launch_arguments={"waypoint_file": waypoint_file, "course": course}.items(),
            )
        ]
    )
