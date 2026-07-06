"""
nav2_stack.launch.py — Full Nav2 Stack for ASV KKI 2026
=========================================================
Launches:
  1. robot_localization (EKF odom + EKF map + NavSat transform)
  2. Static transform publishers for map->odom and lidar
  3. Nav2 core nodes (BT navigator, planner, controller, behavior server, smoother, waypoint follower)
  4. OpenNav Docking Server (replaces docking part of mission_supervisor)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    docking_params_file = LaunchConfiguration("docking_params_file")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay_s")
    use_navsat = LaunchConfiguration("use_navsat")
    speed_mask_yaml = LaunchConfiguration("speed_mask_yaml")

    nav2_share = FindPackageShare("asv_nav2")
    nav2_bt_share = FindPackageShare("nav2_bt_navigator")

    default_nav2_params = PathJoinSubstitution(
        [nav2_share, "config", "nav2_params.yaml"]
    )
    default_docking_params = PathJoinSubstitution(
        [nav2_share, "config", "docking_params.yaml"]
    )
    default_nav_to_pose_bt_xml = PathJoinSubstitution(
        [nav2_share, "behavior_trees", "kki_navigate_to_pose_bt.xml"]
    )
    default_nav_through_poses_bt_xml = PathJoinSubstitution(
        [
            nav2_bt_share,
            "behavior_trees",
            "navigate_through_poses_w_replanning_and_recovery.xml",
        ]
    )
    mission_bt_xml = PathJoinSubstitution(
        [nav2_share, "behavior_trees", "kki_mission_bt.xml"]
    )

    return LaunchDescription(
        [
            # ── Launch Arguments ──────────────────────────────
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "nav2_params_file", default_value=default_nav2_params
            ),
            DeclareLaunchArgument(
                "docking_params_file", default_value=default_docking_params
            ),
            DeclareLaunchArgument(
                "nav2_start_delay_s",
                default_value="35.0",
                description="Wait for Gazebo + sensors before starting Nav2",
            ),
            DeclareLaunchArgument(
                "use_navsat",
                default_value="false",
                description=(
                    "Enable GPS/NavSat global EKF. Sim courses use local Gazebo "
                    "coordinates by default for deterministic map/odom TF."
                ),
            ),
            DeclareLaunchArgument(
                "speed_mask_yaml",
                default_value="",
                description=(
                    "Path to speed_mask.yaml for the Nav2 SpeedFilter. "
                    "Set by the top-level launch file based on course selection."
                ),
            ),

            # ── robot_localization: EKF odom ──
            # Note: publish_tf=false (planar_pose_controller handles TF directly)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav2_share, "config", "ekf_odom.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/local"),
                ],
            ),

            # ── robot_localization: EKF map (map→odom TF) ──────
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                condition=IfCondition(use_navsat),
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav2_share, "config", "ekf_map.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/global"),
                    ("set_pose", "/set_map_pose"),
                ],
            ),

            # ── robot_localization: NavSat Transform ───────────
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                condition=IfCondition(use_navsat),
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [nav2_share, "config", "navsat_transform.yaml"]
                    ),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("imu/data", "/asv/imu/data"),
                    ("gps/fix", "/asv/gps/fix"),
                    ("odometry/filtered", "/odometry/global"),
                    ("odometry/gps", "/odometry/gps"),
                ],
            ),

            # Static transform for map -> odom (identity transform)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf",
                condition=UnlessCondition(use_navsat),
                output="screen",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
                parameters=[{"use_sim_time": use_sim_time}],
            ),

            # Alias for LiDAR link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_sensor_frame_alias_tf",
                output="screen",
                arguments=[
                    "--frame-id",
                    "lidar_link",
                    "--child-frame-id",
                    "asv_kki_2026/lidar_link/lidar_sensor",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),

            # ── Nav2 stack (delayed to wait for sim) ──────────
            TimerAction(
                period=nav2_start_delay,
                actions=[
                    # ── BT Navigator ──────────────────────────
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "use_bond": False,
                                "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                                "default_nav_through_poses_bt_xml": mission_bt_xml,
                            },
                        ],
                    ),

                    # ── Planner Server (Smac Hybrid-A*) ───────
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "use_bond": False,
                            },
                        ],
                        remappings=[
                            ("odom", "/odometry/local"),
                        ],
                    ),

                    # ── Controller Server (RPP) ───────────────
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "use_bond": False,
                            },
                        ],
                        remappings=[
                            ("odom", "/odometry/local"),
                            ("cmd_vel", "/cmd_vel_nav"),
                        ],
                    ),

                    # ── Smoother Server (RViz2 needs this to load its plugins) ──
                    Node(
                        package="nav2_smoother",
                        executable="smoother_server",
                        name="smoother_server",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "use_bond": False,
                            },
                        ],
                    ),

                    # ── Behavior Server ──
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "use_bond": False,
                            },
                        ],
                        remappings=[
                            ("odom", "/odometry/local"),
                            ("cmd_vel", "/cmd_vel_nav"),
                        ],
                    ),

                    # ── Waypoint Follower ─────────────────────
                    Node(
                        package="nav2_waypoint_follower",
                        executable="waypoint_follower",
                        name="waypoint_follower",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "use_bond": False,
                            },
                        ],
                    ),

                    Node(
                        package="asv_control",
                        executable="autonomy_cmd_mux",
                        name="autonomy_cmd_mux",
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),

                    Node(
                        package="nav2_velocity_smoother",
                        executable="velocity_smoother",
                        name="velocity_smoother",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {"use_sim_time": use_sim_time, "use_bond": False},
                        ],
                        remappings=[
                            ("cmd_vel", "/cmd_vel_auto_raw"),
                            ("cmd_vel_smoothed", "/cmd_vel_auto"),
                            ("odom", "/odometry/local"),
                        ],
                    ),

                    Node(
                        package="nav2_collision_monitor",
                        executable="collision_monitor",
                        name="nav2_collision_monitor",
                        output="screen",
                        parameters=[
                            PathJoinSubstitution(
                                [nav2_share, "config", "collision_monitor_params.yaml"]
                            ),
                            {"use_sim_time": use_sim_time, "use_bond": False},
                        ],
                    ),

                    # ── Speed Mask Server (for SpeedFilter) ──────
                    # Publishes speed_mask.pgm as /asv/speed_mask OccupancyGrid
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="speed_mask_server",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "yaml_filename": speed_mask_yaml,
                                "topic_name": "/asv/speed_mask",
                                "frame_id": "map",
                            },
                        ],
                    ),

                    # ── CostmapFilterInfo Server (Speed) ──────────
                    # Publishes encoding metadata for SpeedFilter plugin
                    Node(
                        package="nav2_map_server",
                        executable="costmap_filter_info_server",
                        name="speed_filter_info_server",
                        output="screen",
                        parameters=[
                            nav2_params_file,
                            {"use_sim_time": use_sim_time},
                        ],
                    ),

                    # ── Lifecycle Manager ─────────────────────

                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                "bond_timeout": 4.0,
                                "node_names": [
                                    "planner_server",
                                    "controller_server",
                                    "smoother_server",
                                    "behavior_server",
                                    "bt_navigator",
                                    "waypoint_follower",
                                    "velocity_smoother",
                                    "nav2_collision_monitor",
                                    "speed_mask_server",
                                    "speed_filter_info_server",
                                ],
                            }
                        ],
                    ),

                    # ── OpenNav Docking Server ─────────────────
                    Node(
                        package="opennav_docking",
                        executable="opennav_docking",
                        name="docking_server",
                        output="screen",
                        parameters=[
                            docking_params_file,
                            {
                                "use_sim_time": use_sim_time,
                                "use_bond": False,
                            },
                        ],
                        remappings=[
                            ("odom", "/odometry/local"),
                            ("cmd_vel", "/cmd_vel_dock"),
                            ("scan", "/asv/lidar/scan"),
                        ],
                    ),

                    # ── Lifecycle Manager for Docking ─────────
                    TimerAction(
                        period=2.0,
                        actions=[
                            Node(
                                package="nav2_lifecycle_manager",
                                executable="lifecycle_manager",
                                name="lifecycle_manager_docking",
                                output="screen",
                                parameters=[
                                    {
                                        "use_sim_time": use_sim_time,
                                        "autostart": autostart,
                                        "bond_timeout": 15.0,
                                        "node_names": ["docking_server"],
                                    }
                                ],
                            )
                        ],
                    ),
                ],
            ),
        ]
    )
