#!/usr/bin/env python3

import csv
import math
import re
import time
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String
from vision_msgs.msg import Detection2DArray


class MissionMetricsLogger(Node):
    """Record comparable lap telemetry without affecting mission control."""

    FIELDS = (
        "wall_time_s",
        "lap_elapsed_s",
        "lap",
        "phase",
        "x_m",
        "y_m",
        "cmd_vx_mps",
        "cmd_wz_radps",
        "odom_vx_mps",
        "odom_wz_radps",
        "command_speed_error_mps",
        "cross_track_error_m",
        "replan_count",
        "collision_count",
        "hsv_confidence",
        "coverage_percent",
        "thruster_trim",
        "mission_status",
    )

    def __init__(self):
        super().__init__("mission_metrics_logger")
        self.declare_parameter("course", "a")
        self.declare_parameter("record_rate_hz", 5.0)
        self.declare_parameter(
            "output_directory",
            "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_metrics",
        )
        course = str(self.get_parameter("course").value).strip().lower()
        directory = FilePath(
            str(self.get_parameter("output_directory").value)
        ) / course
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.output_path = directory / f"mission_{course}_{stamp}.csv"
        self.stream = self.output_path.open("w", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=self.FIELDS)
        self.writer.writeheader()

        self.started = time.monotonic()
        self.lap_started = self.started
        self.lap = 1
        self.phase = "WAIT"
        self.status = ""
        self.pose = None
        self.odom_twist = Twist()
        self.command = Twist()
        self.path = []
        self.replan_count = 0
        self.collision_count = 0
        self.collision_active = False
        self.hsv_confidences = []
        self.coverage = 0.0
        self.trim = 0.0

        self.create_subscription(Odometry, "/asv/planar_odom", self._odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._command, 10)
        self.create_subscription(Path, "/plan", self._path, 10)
        self.create_subscription(
            String, "/asv/mission/status", self._mission_status, 10
        )
        self.create_subscription(
            Bool, "/asv/collision/detected", self._collision, 10
        )
        self.create_subscription(
            Float32, "/asv/mapping/coverage", self._coverage, 10
        )
        self.create_subscription(
            Float64, "/asv/control/adaptive_trim", self._trim, 10
        )
        self.create_subscription(
            Detection2DArray,
            "/asv/perception/front/detections",
            self._detections,
            10,
        )
        self.create_subscription(
            Detection2DArray,
            "/asv/perception/down/detections",
            self._detections,
            10,
        )
        rate = max(1.0, float(self.get_parameter("record_rate_hz").value))
        self.create_timer(1.0 / rate, self._record)
        self.get_logger().info(f"Mission metrics: {self.output_path}")

    def _odom(self, msg):
        self.pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        self.odom_twist = msg.twist.twist

    def _command(self, msg):
        self.command = msg

    def _path(self, msg):
        self.path = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        self.replan_count += 1

    def _mission_status(self, msg):
        self.status = msg.data
        lap_match = re.search(r"\blap=(\d+)/", msg.data)
        phase_match = re.search(r"\bphase=([A-Z0-9_]+)", msg.data)
        new_lap = int(lap_match.group(1)) if lap_match else self.lap
        if new_lap != self.lap:
            self.lap = new_lap
            self.lap_started = time.monotonic()
            self.replan_count = 0
            self.collision_count = 0
        if phase_match:
            self.phase = phase_match.group(1)

    def _collision(self, msg):
        active = bool(msg.data)
        if active and not self.collision_active:
            self.collision_count += 1
        self.collision_active = active

    def _coverage(self, msg):
        self.coverage = float(msg.data)

    def _trim(self, msg):
        self.trim = max(-0.10, min(0.10, float(msg.data)))

    def _detections(self, msg):
        for detection in msg.detections:
            if detection.results:
                self.hsv_confidences.append(
                    float(detection.results[0].hypothesis.score)
                )
        self.hsv_confidences = self.hsv_confidences[-100:]

    def _cross_track_error(self):
        if self.pose is None or not self.path:
            return float("nan")
        return min(
            math.hypot(self.pose[0] - x, self.pose[1] - y)
            for x, y in self.path
        )

    def _record(self):
        now = time.monotonic()
        x, y = self.pose if self.pose is not None else (float("nan"), float("nan"))
        confidence = (
            sum(self.hsv_confidences) / len(self.hsv_confidences)
            if self.hsv_confidences
            else 0.0
        )
        self.writer.writerow(
            {
                "wall_time_s": round(now - self.started, 3),
                "lap_elapsed_s": round(now - self.lap_started, 3),
                "lap": self.lap,
                "phase": self.phase,
                "x_m": round(x, 4),
                "y_m": round(y, 4),
                "cmd_vx_mps": round(float(self.command.linear.x), 4),
                "cmd_wz_radps": round(float(self.command.angular.z), 4),
                "odom_vx_mps": round(float(self.odom_twist.linear.x), 4),
                "odom_wz_radps": round(float(self.odom_twist.angular.z), 4),
                "command_speed_error_mps": round(
                    float(self.command.linear.x - self.odom_twist.linear.x), 4
                ),
                "cross_track_error_m": round(self._cross_track_error(), 4),
                "replan_count": self.replan_count,
                "collision_count": self.collision_count,
                "hsv_confidence": round(confidence, 4),
                "coverage_percent": round(self.coverage, 3),
                "thruster_trim": round(self.trim, 4),
                "mission_status": self.status,
            }
        )
        self.stream.flush()

    def close(self):
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


def main(args=None):
    rclpy.init(args=args)
    node = MissionMetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
