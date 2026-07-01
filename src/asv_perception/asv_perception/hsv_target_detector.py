#!/usr/bin/env python3

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .hsv_core import StableHsvDetector


class HsvTargetDetector(Node):
    def __init__(self):
        super().__init__("hsv_target_detector")
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("max_image_width", 640)
        self.declare_parameter("stable_frames", 5)
        self.declare_parameter("min_area_px", 120.0)
        self.declare_parameter("box_area_ratio", 0.008)

        kwargs = {
            "stable_frames": self.get_parameter("stable_frames").value,
            "min_area_px": self.get_parameter("min_area_px").value,
            "box_area_ratio": self.get_parameter("box_area_ratio").value,
            "max_width": self.get_parameter("max_image_width").value,
        }
        self.detectors = {
            "front": StableHsvDetector(**kwargs),
            "down": StableHsvDetector(**kwargs),
        }
        self.bridge = CvBridge()
        self.latest = {"front": None, "down": None}
        self.detection_pubs = {
            "front": self.create_publisher(
                Detection2DArray, "/asv/perception/front/detections", 10
            ),
            "down": self.create_publisher(
                Detection2DArray, "/asv/perception/down/detections", 10
            ),
        }
        self.create_subscription(
            Image,
            "/asv/camera/front/image",
            lambda msg: self._on_image("front", msg),
            5,
        )
        self.create_subscription(
            Image,
            "/asv/camera/down/image",
            lambda msg: self._on_image("down", msg),
            5,
        )
        rate = max(1.0, float(self.get_parameter("process_rate_hz").value))
        self.create_timer(1.0 / rate, self._process)

    def _on_image(self, camera_name, msg):
        self.latest[camera_name] = msg

    def _process(self):
        for camera_name, msg in self.latest.items():
            if msg is None:
                continue
            self.latest[camera_name] = None
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warn(f"Failed to decode {camera_name} image: {exc}")
                continue

            output = Detection2DArray()
            output.header = msg.header
            for index, candidate in enumerate(
                self.detectors[camera_name].detect(image, camera_name)
            ):
                detection = Detection2D()
                detection.header = msg.header
                detection.id = f"{camera_name}:{candidate.class_id}:{index}"
                detection.bbox.center.position.x = candidate.x
                detection.bbox.center.position.y = candidate.y
                detection.bbox.size_x = candidate.width
                detection.bbox.size_y = candidate.height
                result = ObjectHypothesisWithPose()
                result.hypothesis.class_id = candidate.class_id
                result.hypothesis.score = candidate.score
                detection.results.append(result)
                output.detections.append(detection)
            self.detection_pubs[camera_name].publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = HsvTargetDetector()
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
