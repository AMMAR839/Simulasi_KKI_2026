from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="asv_gz_bridge",
                output="screen",
                arguments=BRIDGE_ARGUMENTS,
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="asv_sensors",
                executable="sensor_status_monitor",
                name="sensor_status_monitor",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="asv_sensors",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
