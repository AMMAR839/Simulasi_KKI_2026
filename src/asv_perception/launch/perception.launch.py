from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    course = LaunchConfiguration("course")
    lidar_max_range = LaunchConfiguration("lidar_max_range_m")
    use_sim_time = LaunchConfiguration("use_sim_time")
    waypoint_file = LaunchConfiguration("waypoint_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("course", default_value="a"),
            DeclareLaunchArgument("lidar_max_range_m", default_value="6.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("waypoint_file", default_value=""),
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
