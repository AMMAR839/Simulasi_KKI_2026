#!/usr/bin/env python3

import math
import statistics
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass
class Landmark:
    class_id: str
    x: float
    y: float
    confidence: float
    observations: int
    variance: float
    capture_x: float
    capture_y: float
    capture_yaw: float


class SemanticLandmarkMapper(Node):
    def __init__(self):
        super().__init__("semantic_landmark_mapper")
        self.declare_parameter("course", "a")
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("association_radius_m", 0.8)
        self.declare_parameter("front_camera_yaw_rad", 0.0)
        self.declare_parameter("down_camera_yaw_rad", 1.570796)
        self.declare_parameter(
            "save_directory",
            "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_maps",
        )
        self.pose = None
        self.scan = None
        self.camera_info = {"front": None, "down": None}
        self.landmarks = []
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.association_radius = float(
            self.get_parameter("association_radius_m").value
        )
        self.camera_yaw = {
            "front": float(self.get_parameter("front_camera_yaw_rad").value),
            "down": float(self.get_parameter("down_camera_yaw_rad").value),
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.landmark_pub = self.create_publisher(
            Detection3DArray, "/asv/perception/landmarks", 10
        )
        self.create_subscription(
            Odometry, "/asv/planar_odom", self._on_odom, 10
        )
        self.create_subscription(LaserScan, "/asv/lidar/scan", self._on_scan, 10)
        self.create_subscription(
            CameraInfo,
            "/asv/camera/front/camera_info",
            lambda msg: self._on_camera_info("front", msg),
            5,
        )
        self.create_subscription(
            CameraInfo,
            "/asv/camera/down/camera_info",
            lambda msg: self._on_camera_info("down", msg),
            5,
        )
        self.create_subscription(
            Detection2DArray,
            "/asv/perception/front/detections",
            lambda msg: self._on_detections("front", msg),
            10,
        )
        self.create_subscription(
            Detection2DArray,
            "/asv/perception/down/detections",
            lambda msg: self._on_detections("down", msg),
            10,
        )
        self.create_service(
            Trigger, "/asv/perception/save_landmarks", self._save_landmarks
        )
        self.create_timer(1.0, self._publish)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (
            float(p.x),
            float(p.y),
            quaternion_to_yaw(msg.pose.pose.orientation),
        )

    def _on_scan(self, msg):
        self.scan = msg

    def _on_camera_info(self, camera_name, msg):
        self.camera_info[camera_name] = msg

    def _on_detections(self, camera_name, msg):
        if self.scan is None:
            return
        pose = self._base_pose_in_map(msg.header.stamp)
        if pose is None:
            return
        info = self.camera_info[camera_name]
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            class_id = result.hypothesis.class_id
            score = float(result.hypothesis.score)
            bearing = self._pixel_bearing(
                detection.bbox.center.position.x, info, camera_name
            )
            distance = self._range_near_bearing(bearing)
            if distance is None:
                continue
            boat_x, boat_y, boat_yaw = pose
            global_bearing = boat_yaw + bearing
            x = boat_x + distance * math.cos(global_bearing)
            y = boat_y + distance * math.sin(global_bearing)
            self._associate(
                class_id, x, y, score, boat_x, boat_y, boat_yaw
            )

    def _base_pose_in_map(self, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time.from_msg(stamp),
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

    def _pixel_bearing(self, pixel_x, info, camera_name):
        if info is not None and info.k[0] > 1e-6:
            relative = math.atan2(info.k[2] - float(pixel_x), info.k[0])
        elif info is not None and info.width:
            normalized = (info.width * 0.5 - float(pixel_x)) / info.width
            relative = normalized * 1.3962634
        else:
            relative = 0.0
        return self.camera_yaw[camera_name] + relative

    def _range_near_bearing(self, bearing):
        scan = self.scan
        normalized = math.atan2(math.sin(bearing), math.cos(bearing))
        center = int(round((normalized - scan.angle_min) / scan.angle_increment))
        half_window = max(
            2, int(round(math.radians(4.0) / abs(scan.angle_increment)))
        )
        values = []
        for index in range(center - half_window, center + half_window + 1):
            wrapped = index % len(scan.ranges)
            value = float(scan.ranges[wrapped])
            if (
                math.isfinite(value)
                and max(0.45, scan.range_min) <= value <= min(self.max_range, scan.range_max)
            ):
                values.append(value)
        return statistics.median(values) if values else None

    def _associate(
        self, class_id, x, y, score, capture_x, capture_y, capture_yaw
    ):
        nearest = None
        nearest_distance = float("inf")
        for landmark in self.landmarks:
            if landmark.class_id != class_id:
                continue
            distance = math.hypot(x - landmark.x, y - landmark.y)
            if distance < nearest_distance:
                nearest = landmark
                nearest_distance = distance
        if nearest is None or nearest_distance > self.association_radius:
            self.landmarks.append(
                Landmark(
                    class_id,
                    x,
                    y,
                    score,
                    1,
                    0.25,
                    capture_x,
                    capture_y,
                    capture_yaw,
                )
            )
            return
        count = nearest.observations + 1
        old_x, old_y = nearest.x, nearest.y
        nearest.x += (x - nearest.x) / count
        nearest.y += (y - nearest.y) / count
        residual = (x - old_x) ** 2 + (y - old_y) ** 2
        nearest.variance = max(
            0.0025,
            ((count - 2) * nearest.variance + residual) / max(count - 1, 1),
        )
        nearest.observations = count
        if score >= nearest.confidence:
            nearest.confidence = score
            nearest.capture_x = capture_x
            nearest.capture_y = capture_y
            nearest.capture_yaw = capture_yaw

    def _publish(self):
        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for index, landmark in enumerate(self.landmarks):
            detection = Detection3D()
            detection.header = msg.header
            detection.id = f"{landmark.class_id}:{index}"
            detection.bbox.center.position.x = landmark.x
            detection.bbox.center.position.y = landmark.y
            detection.bbox.center.orientation.w = 1.0
            size = 0.6 if "box" in landmark.class_id else 0.2
            detection.bbox.size.x = size
            detection.bbox.size.y = size
            detection.bbox.size.z = size
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = landmark.class_id
            result.hypothesis.score = landmark.confidence
            result.pose.pose.position.x = landmark.x
            result.pose.pose.position.y = landmark.y
            result.pose.pose.orientation.w = 1.0
            result.pose.covariance[0] = landmark.variance
            result.pose.covariance[7] = landmark.variance
            detection.results.append(result)
            msg.detections.append(detection)
        self.landmark_pub.publish(msg)

    def _save_landmarks(self, _request, response):
        course = str(self.get_parameter("course").value).strip().lower()
        root = Path(str(self.get_parameter("save_directory").value)) / course
        root.mkdir(parents=True, exist_ok=True)
        path = root / "landmarks.yaml"
        payload = {
            "frame_id": "map",
            "landmarks": [
                {
                    "class_id": item.class_id,
                    "x": round(item.x, 4),
                    "y": round(item.y, 4),
                    "covariance": round(item.variance, 6),
                    "observations": item.observations,
                    "confidence": round(item.confidence, 4),
                    "capture_pose": [
                        round(item.capture_x, 4),
                        round(item.capture_y, 4),
                        round(item.capture_yaw, 5),
                    ],
                }
                for item in self.landmarks
            ],
        }
        path.write_text(yaml.safe_dump(payload, sort_keys=False))
        response.success = True
        response.message = str(path)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SemanticLandmarkMapper()
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
