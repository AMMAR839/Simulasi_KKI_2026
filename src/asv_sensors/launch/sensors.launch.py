import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


BASE_LD_LIBRARY_PATH = os.environ.get("LD_LIBRARY_PATH", "")
BASE_GZ_SIM_RESOURCE_PATH = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
CLEAN_LD_LIBRARY_PATH = EnvironmentVariable(
    "ASV_CLEAN_LD_LIBRARY_PATH", default_value=BASE_LD_LIBRARY_PATH
)
CLEAN_GZ_SIM_RESOURCE_PATH = EnvironmentVariable(
    "ASV_CLEAN_GZ_SIM_RESOURCE_PATH", default_value=BASE_GZ_SIM_RESOURCE_PATH
)


BRIDGE_ARGUMENTS = [
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/asv/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
    "/asv/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/asv/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/asv/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/asv/camera/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/asv/camera/down/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/asv/camera/down/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/asv/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    "/model/asv_kki_2026/joint/left_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
    "/model/asv_kki_2026/joint/right_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
    "/model/asv_kki_2026/joint/left_propeller_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
    "/model/asv_kki_2026/joint/right_propeller_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
    "/asv/thrusters/left_steering/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
    "/asv/thrusters/right_steering/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
    "/asv/collisions/hull@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/asv/collisions/left_thruster@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/asv/collisions/right_thruster@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/asv/collisions/left_propeller@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/asv/collisions/right_propeller@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/asv/collisions/lidar@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/asv/collisions/front_camera@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/asv/collisions/down_camera@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "/kki/collisions/objects@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_lidar_pointcloud = LaunchConfiguration("enable_lidar_pointcloud")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "enable_lidar_pointcloud",
                default_value="false",
                description=(
                    "Bridge the simulator-only multi-layer LiDAR point cloud. "
                    "LaserScan remains the minimum hardware interface."
                ),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        name="asv_gz_bridge",
                        output="screen",
                        arguments=BRIDGE_ARGUMENTS,
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        name="asv_lidar_points_bridge",
                        output="screen",
                        arguments=[
                            "/asv/lidar/scan/points"
                            "@sensor_msgs/msg/PointCloud2"
                            "[gz.msgs.PointCloudPacked"
                        ],
                        parameters=[{"use_sim_time": use_sim_time}],
                        condition=IfCondition(enable_lidar_pointcloud),
                    )
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="asv_sensors",
                        executable="collision_monitor",
                        name="contact_monitor",
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),
                ],
            ),
            TimerAction(
                period=50.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "env",
                            "-u",
                            "GZ_SIM_SYSTEM_PLUGIN_PATH",
                            "-u",
                            "GZ_RENDERING_PLUGIN_PATH",
                            "ros2",
                            "run",
                            "asv_sensors",
                            "sensor_status_monitor",
                            "--ros-args",
                            "-r",
                            "__node:=sensor_status_monitor",
                            "-p",
                            ["use_sim_time:=", use_sim_time],
                        ],
                        name="sensor_status_monitor",
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
