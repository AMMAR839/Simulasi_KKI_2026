from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = PathJoinSubstitution(
        [FindPackageShare("asv_control"), "config", "joystick.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Joystick uses wall time so /joy works even before Gazebo /clock is ready.",
            ),
            Node(
                package="asv_control",
                executable="simple_joystick_teleop",
                name="simple_joystick_teleop",
                output="screen",
                parameters=[config, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="asv_control",
                executable="joystick_mode_manager",
                name="joystick_mode_manager",
                output="screen",
                parameters=[config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
