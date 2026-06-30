import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ASV_ROOT = "/home/ammar/Documents/asv_simulation"
BASE_LD_LIBRARY_PATH = os.environ.get("LD_LIBRARY_PATH", "")
BASE_GZ_SIM_RESOURCE_PATH = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
REFERENCE_RESOURCE_PATHS = [
    f"{ASV_ROOT}/vrx/vrx_gz/models",
    f"{ASV_ROOT}/vrx/vrx_urdf/vrx_gazebo/models",
    f"{ASV_ROOT}/SINGABOAT-VRX/vrx/vrx_gazebo/models",
    f"{ASV_ROOT}/SINGABOAT-VRX/vrx/wave_gazebo/world_models",
    f"{ASV_ROOT}/SINGABOAT-VRX/singaboat_vrx/worlds",
    f"{ASV_ROOT}/asv_wave_sim/gz-waves-models/models",
    f"{ASV_ROOT}/asv_wave_sim/gz-waves-models/world_models",
]
REFERENCE_PLUGIN_PATHS = [
    "/usr/lib/x86_64-linux-gnu",
    "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins",
    f"{ASV_ROOT}/SINGABOAT-VRX/vrx/wave_gazebo/build",
    f"{ASV_ROOT}/SINGABOAT-VRX/vrx/wave_gazebo/build/lib",
    f"{ASV_ROOT}/asv_wave_sim/gz-waves/build/lib",
    f"{ASV_ROOT}/asv_wave_sim/gz-waves/build/src/systems",
    f"{ASV_ROOT}/asv_wave_sim/gz-waves/build/src/gui",
    f"{ASV_ROOT}/asv_wave_sim/gz-waves/install/lib",
    f"{ASV_ROOT}/vrx/install/lib",
    f"{ASV_ROOT}/SINGABOAT-VRX/install/lib",
]


def joined_path(parts):
    value = []
    for index, part in enumerate(parts):
        if index > 0:
            value.append(":")
        value.append(part)
    return value


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mux = LaunchConfiguration("use_mux")
    auto_mode = LaunchConfiguration("auto_mode")
    default_mode = LaunchConfiguration("default_mode")
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    world_name = LaunchConfiguration("world_name")
    headless = LaunchConfiguration("headless")

    desc_share = FindPackageShare("asv_description")
    gazebo_share = FindPackageShare("asv_gazebo")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    robot_xacro = PathJoinSubstitution(
        [desc_share, "urdf", "asv_kki_2026.urdf.xacro"]
    )
    default_world = PathJoinSubstitution(
        [gazebo_share, "worlds", "kki_2026_lintasan_a.sdf"]
    )

    resource_paths = [
        PathJoinSubstitution([desc_share, "models"]),
        PathJoinSubstitution([gazebo_share, "models"]),
    ] + REFERENCE_RESOURCE_PATHS
    plugin_paths = [
        PathJoinSubstitution([gazebo_share, "plugins"]),
        f"{ASV_ROOT}/asv_kki_2026_ws/src/asv_gazebo/plugins",
    ] + REFERENCE_PLUGIN_PATHS

    robot_description = Command(["xacro", " ", robot_xacro])
    planar_pose_controller = ExecuteProcess(
        cmd=[
            "env",
            "-u",
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            "-u",
            "GZ_RENDERING_PLUGIN_PATH",
            "ros2",
            "run",
            "asv_control",
            "planar_pose_controller",
            "--ros-args",
            "-r",
            "__node:=planar_pose_controller",
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            ["world_name:=", world_name],
            "-p",
            "odom_topic:=/asv/planar_odom",
            "-p",
            ["spawn_x:=", spawn_x],
            "-p",
            ["spawn_y:=", spawn_y],
            "-p",
            ["surface_z:=", spawn_z],
            "-p",
            ["spawn_yaw:=", spawn_yaw],
            "-p",
            "pose_rate_hz:=10.0",
            "-p",
            "minimum_visual_z:=0.01",
        ],
        name="planar_pose_controller",
        output="screen",
        additional_env={
            "LD_LIBRARY_PATH": BASE_LD_LIBRARY_PATH,
            "GZ_SIM_RESOURCE_PATH": BASE_GZ_SIM_RESOURCE_PATH,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_mux", default_value="true"),
            DeclareLaunchArgument(
                "auto_mode",
                default_value="true",
                description="true starts autonomous waypoint mode; false starts manual joystick mode.",
            ),
            DeclareLaunchArgument(
                "default_mode",
                default_value="",
                description="Deprecated override: manual or auto. Leave empty to use auto_mode.",
            ),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("spawn_x", default_value="10.8"),
            DeclareLaunchArgument("spawn_y", default_value="-8.7"),
            DeclareLaunchArgument("spawn_z", default_value="0.02"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.2405"),
            DeclareLaunchArgument("world_name", default_value="kki_2026_lintasan_a"),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Gazebo server without GUI.",
            ),
            DeclareLaunchArgument(
                "show_lidar_visual",
                default_value="true",
                description="Show Gazebo LiDAR rays. Set false to hide rays without disabling /asv/lidar/scan.",
            ),
            SetEnvironmentVariable(
                name="ASV_CLEAN_LD_LIBRARY_PATH",
                value=BASE_LD_LIBRARY_PATH,
            ),
            SetEnvironmentVariable(
                name="ASV_CLEAN_GZ_SIM_RESOURCE_PATH",
                value=BASE_GZ_SIM_RESOURCE_PATH,
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=joined_path(resource_paths),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=joined_path(plugin_paths),
            ),
            SetEnvironmentVariable(
                name="GZ_RENDERING_PLUGIN_PATH",
                value=joined_path(plugin_paths),
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=joined_path(plugin_paths + [EnvironmentVariable("LD_LIBRARY_PATH", default_value="")]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
                ),
                launch_arguments={
                    "gz_args": [
                        PythonExpression([
                            "'-s -r -v 3 --physics-engine gz-physics-bullet-featherstone-plugin ' if '",
                            headless,
                            "'.lower() in ['true', '1', 'yes'] else '-r -v 3 --physics-engine gz-physics-bullet-featherstone-plugin '"
                        ]),
                        world,
                    ]
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    }
                ],
            ),
            TimerAction(
                period=25.0,
                actions=[planar_pose_controller],
            ),
            Node(
                condition=IfCondition(use_mux),
                package="asv_control",
                executable="manual_autonomy_mux",
                name="manual_autonomy_mux",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "auto_mode": auto_mode,
                        "default_mode": default_mode,
                    }
                ],
            ),
            Node(
                package="asv_control",
                executable="cmd_vel_to_thrusters",
                name="cmd_vel_to_thrusters",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
