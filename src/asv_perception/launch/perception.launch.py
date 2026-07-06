from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    course = LaunchConfiguration("course")
    lidar_max_range = LaunchConfiguration("lidar_max_range_m")
    use_sim_time = LaunchConfiguration("use_sim_time")
    waypoint_file = LaunchConfiguration("waypoint_file")
    preload_map_dir = LaunchConfiguration("preload_map_dir")

    # Resolve: <preload_map_dir>/<course>/lap1_map.yaml
    preload_map_yaml = PathJoinSubstitution(
        [preload_map_dir, course, "lap1_map.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("course", default_value="a"),
            DeclareLaunchArgument("lidar_max_range_m", default_value="6.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("waypoint_file", default_value=""),
            DeclareLaunchArgument(
                "preload_map_dir",
                default_value=(
                    "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_maps"
                ),
                description=(
                    "Directory containing saved maps (<dir>/<course>/lap1_map.yaml). "
                    "Set to empty string to disable preload."
                ),
            ),
            Node(
                package="asv_perception",
                executable="hsv_target_detector",
                name="hsv_target_detector",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="asv_perception",
                executable="survey_mapper",
                name="survey_mapper",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "course": course,
                        "max_range_m": lidar_max_range,
                        "preload_map_yaml": preload_map_yaml,
                    }
                ],
            ),
            Node(
                package="asv_perception",
                executable="semantic_landmark_mapper",
                name="semantic_landmark_mapper",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "course": course,
                        "max_range_m": lidar_max_range,
                    }
                ],
            ),
            Node(
                package="asv_perception",
                executable="preferred_lane_server",
                name="preferred_lane_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "waypoint_file": waypoint_file,
                    }
                ],
            ),
        ]
    )
