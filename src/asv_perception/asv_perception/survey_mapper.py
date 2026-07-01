#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .mapping_core import OccupancyGridMapper


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SurveyMapper(Node):
    def __init__(self):
        super().__init__("survey_mapper")
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("min_range_m", 0.45)
        self.declare_parameter("beam_stride", 2)
        self.declare_parameter("scan_stride", 3)
        self.declare_parameter("resolution_m", 0.10)
        self.declare_parameter("map_width_m", 60.0)
        self.declare_parameter("map_height_m", 60.0)
        self.declare_parameter("origin_x", -30.0)
        self.declare_parameter("origin_y", -30.0)
        self.declare_parameter(
            "save_directory",
            "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_maps",
        )
        self.declare_parameter("course", "a")

        self.max_range = float(self.get_parameter("max_range_m").value)
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.beam_stride = max(1, int(self.get_parameter("beam_stride").value))
        self.scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self.mapper = OccupancyGridMapper(
            width_m=float(self.get_parameter("map_width_m").value),
            height_m=float(self.get_parameter("map_height_m").value),
            resolution=float(self.get_parameter("resolution_m").value),
            origin_x=float(self.get_parameter("origin_x").value),
            origin_y=float(self.get_parameter("origin_y").value),
        )
        self.pose = None
        self.scan_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(
            OccupancyGrid, "/asv/mapping/occupancy", qos
        )
        self.coverage_pub = self.create_publisher(
            Float32, "/asv/mapping/coverage", 10
        )
        self.create_subscription(
            Odometry, "/asv/planar_odom", self._on_odom, 10
        )
        self.create_subscription(LaserScan, "/asv/lidar/scan", self._on_scan, 10)
        self.create_service(Trigger, "/asv/mapping/save", self._save_map)
        self.create_timer(1.0, self._publish)

    def _on_odom(self, msg):
        position = msg.pose.pose.position
        self.pose = (
            float(position.x),
            float(position.y),
            quaternion_to_yaw(msg.pose.pose.orientation),
        )

    def _on_scan(self, msg):
        self.scan_count += 1
        if self.scan_count % self.scan_stride:
            return
        pose = self._scan_pose_in_map(msg)
        if pose is None:
            return
        x, y, yaw = pose
        valid_max = min(float(msg.range_max), self.max_range)
        valid_min = max(float(msg.range_min), self.min_range)
        for index in range(0, len(msg.ranges), self.beam_stride):
            measured = float(msg.ranges[index])
            hit = math.isfinite(measured) and valid_min <= measured <= valid_max
            distance = measured if hit else valid_max
            if not math.isfinite(distance) or distance < valid_min:
                continue
            angle = yaw + float(msg.angle_min) + index * float(msg.angle_increment)
            self.mapper.integrate_ray(
                x,
                y,
                x + distance * math.cos(angle),
                y + distance * math.sin(angle),
                hit=hit,
            )

    def _scan_pose_in_map(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.03),
            )
            translation = transform.transform.translation
            return (
                float(translation.x),
                float(translation.y),
                quaternion_to_yaw(transform.transform.rotation),
            )
        except TransformException:
            return self.pose

    def _make_message(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.mapper.resolution
        msg.info.width = self.mapper.width
        msg.info.height = self.mapper.height
        msg.info.origin.position.x = self.mapper.origin_x
        msg.info.origin.position.y = self.mapper.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.mapper.occupancy_values().reshape(-1).tolist()
        return msg

    def _publish(self):
        self.map_pub.publish(self._make_message())
        self.coverage_pub.publish(
            Float32(data=float(self.mapper.coverage_percent))
        )

    def _save_map(self, _request, response):
        course = str(self.get_parameter("course").value).strip().lower()
        root = Path(str(self.get_parameter("save_directory").value)) / course
        root.mkdir(parents=True, exist_ok=True)
        values = self.mapper.occupancy_values()
        image = np.full(values.shape, 205, dtype=np.uint8)
        image[(values >= 0) & (values < 35)] = 254
        image[values >= 65] = 0
        pgm_path = root / "lap1_map.pgm"
        yaml_path = root / "lap1_map.yaml"
        with pgm_path.open("wb") as stream:
            stream.write(
                f"P5\n{self.mapper.width} {self.mapper.height}\n255\n".encode(
                    "ascii"
                )
            )
            stream.write(np.flipud(image).tobytes())
        metadata = {
            "image": pgm_path.name,
            "mode": "trinary",
            "resolution": self.mapper.resolution,
            "origin": [self.mapper.origin_x, self.mapper.origin_y, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.35,
            "coverage_percent": round(self.mapper.coverage_percent, 3),
        }
        yaml_path.write_text(yaml.safe_dump(metadata, sort_keys=False))
        response.success = True
        response.message = str(yaml_path)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SurveyMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
