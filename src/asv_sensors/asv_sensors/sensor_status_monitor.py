#!/usr/bin/env python3
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix
from std_msgs.msg import String


class SensorStatusMonitor(Node):
    """Publish a compact status line for core ASV simulation sensors."""

    def __init__(self):
        super().__init__("sensor_status_monitor")
        self.declare_parameter("timeout_s", 1.5)
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.last_seen = {}
        self._sensor_subscriptions = []

        subscriptions = [
            (NavSatFix, "/asv/gps/fix", "gps"),
            (Imu, "/asv/imu/data", "imu"),
            (LaserScan, "/asv/lidar/scan", "lidar"),
            (Image, "/asv/camera/front/image", "front_camera"),
            (Image, "/asv/camera/down/image", "down_camera"),
            (Odometry, "/asv/odom", "odom"),
        ]
        for msg_type, topic, key in subscriptions:
            self._sensor_subscriptions.append(
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda _msg, sensor_key=key: self.mark_seen(sensor_key),
                    10,
                )
            )

        self.status_pub = self.create_publisher(String, "/asv/sensors/status", 10)
        self.status_timer = self.create_timer(1.0, self.on_timer)

    def mark_seen(self, key):
        self.last_seen[key] = time.monotonic()

    def on_timer(self):
        now = time.monotonic()
        keys = ["gps", "imu", "lidar", "front_camera", "down_camera", "odom"]
        parts = []
        for key in keys:
            age = now - self.last_seen.get(key, 0.0)
            state = "ok" if age <= self.timeout else "missing"
            parts.append(f"{key}={state}")
        self.status_pub.publish(String(data=" ".join(parts)))


def main(args=None):
    rclpy.init(args=args)
    node = SensorStatusMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # During launch shutdown, the Gazebo/ROS bridge can disappear while
        # rclpy is taking a sensor message. Treat only that teardown race as
        # clean shutdown; keep other runtime errors visible.
        if "Unable to convert call argument" not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
