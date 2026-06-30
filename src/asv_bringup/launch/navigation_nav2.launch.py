"""
navigation_nav2.launch.py — Nav2-based navigation launch for ASV KKI 2026
==========================================================================
Replaces navigation.launch.py (custom gps_waypoint_follower).

Launches:
  1. asv_nav2/nav2_stack.launch.py  → full Nav2 + EKF + collision monitor + docking
  2. mission_supervisor_nav2        → mission orchestrator via nav2_simple_commander
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay_s")
    mission_start_delay = LaunchConfiguration("mission_start_delay_s")
    use_navsat = LaunchConfiguration("use_navsat")

    nav_share = FindPackageShare("asv_navigation")
    nav2_share = FindPackageShare("asv_nav2")

    return LaunchDescription(
        [
            # ── Launch Arguments ──────────────────────────────
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=PathJoinSubstitution(
                    [nav_share, "config", "kki_waypoints.yaml"]
                ),
            ),
            DeclareLaunchArgument("course", default_value="a"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "nav2_start_delay_s",
                default_value="35.0",
                description="Delay Nav2 startup until Gazebo + sensors are stable.",
            ),
            DeclareLaunchArgument(
                "mission_start_delay_s",
                default_value="50.0",
                description="Delay mission supervisor until Nav2 stack is fully active.",
            ),
            DeclareLaunchArgument(
                "use_navsat",
                default_value="false",
                description="Use GPS/NavSat global EKF instead of local simulator map frame.",
            ),

            # ── Nav2 Full Stack ───────────────────────────────
            # Includes: EKF odom, optional EKF map/NavSat, BT navigator,
            #           planner, controller, collision monitor, docking server
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [nav2_share, "launch", "nav2_stack.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "nav2_start_delay_s": nav2_start_delay,
                    "use_navsat": use_navsat,
                }.items(),
            ),

            # ── Mission Supervisor Nav2 ───────────────────────
            # Starts after Nav2 is fully up
            TimerAction(
                period=mission_start_delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "env",
                            "-u", "GZ_SIM_SYSTEM_PLUGIN_PATH",
                            "-u", "GZ_RENDERING_PLUGIN_PATH",
                            "ros2", "run",
                            "asv_navigation",
                            "mission_supervisor_nav2",
                            "--ros-args",
                            "-r", "__node:=mission_supervisor",
                            "-p", ["use_sim_time:=", use_sim_time],
                            "-p", ["course:=", course],
                            "-p", ["waypoint_file:=", waypoint_file],
                            "-p", "use_docking_server:=true",
                            "-p", "surface_capture_radius_m:=4.0",
                            "-p", "underwater_capture_radius_m:=3.0",
                        ],
                        name="mission_supervisor_nav2",
                        output="screen",
                        additional_env={
                            "LD_LIBRARY_PATH": CLEAN_LD_LIBRARY_PATH,
                            "GZ_SIM_RESOURCE_PATH": CLEAN_GZ_SIM_RESOURCE_PATH,
                        },
                    )
                ],
            ),
        ]
    )
