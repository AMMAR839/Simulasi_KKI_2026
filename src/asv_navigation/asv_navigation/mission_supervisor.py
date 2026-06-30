#!/usr/bin/env python3
import math
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _count_color_pixels(msg, target: str) -> int:
    """Count pixels that are predominantly the target color (green or blue)."""
    if msg is None or not msg.data:
        return 0
    enc = msg.encoding.lower()
    channels = 4 if enc in ("rgba8", "bgra8") else 3
    src = bytes(msg.data)
    count = 0
    stride = 6  # sample every 6th pixel for speed
    for row in range(0, msg.height, stride):
        for col in range(0, msg.width, stride):
            idx = row * msg.step + col * channels
            if idx + channels > len(src):
                continue
            px = src[idx: idx + channels]
            if enc in ("bgr8", "bgra8"):
                r, g, b = px[2], px[1], px[0]
            else:
                r, g, b = px[0], px[1], px[2]
            if target == "green":
                # Green dominant: g > r+30 and g > b+30
                if g > r + 30 and g > b + 30 and g > 80:
                    count += 1
            elif target == "blue":
                # Blue dominant: b > r+30 and b > g+20
                if b > r + 30 and b > g + 20 and b > 80:
                    count += 1
    return count


COURSE_TARGETS = {
    "a": {
        "surface": (-10.2, -5.4),
        "underwater": (-11.0, -7.7),
        "dock_buoys": [
            ("docking_a_blue_1", 12.35, -10.0),
            ("docking_a_blue_2", 12.35, -9.7),
            ("docking_a_blue_3", 12.35, -9.4),
        ],
    },
    "b": {
        "surface": (10.2, -5.4),
        "underwater": (11.0, -7.7),
        "dock_buoys": [
            ("docking_b_blue_1", -12.35, -10.0),
            ("docking_b_blue_2", -12.35, -9.7),
            ("docking_b_blue_3", -12.35, -9.4),
        ],
    },
}


class MissionSupervisor(Node):
    """Track KKI mission events and save camera evidence near target boxes."""

    def __init__(self):
        super().__init__("mission_supervisor")
        self.declare_parameter("course", "a")
        self.declare_parameter(
            "capture_dir",
            "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_captures",
        )
        self.declare_parameter("surface_capture_radius_m", 3.50)
        self.declare_parameter("underwater_capture_radius_m", 2.50)
        self.declare_parameter("docking_contact_radius_m", 1.20)
        # Minimum colored pixels to consider box visible in camera frame
        self.declare_parameter("min_color_pixels", 20)

        self.course = str(self.get_parameter("course").value).strip().lower()
        if self.course not in COURSE_TARGETS:
            self.get_logger().warn(f"Unknown course '{self.course}', using course a")
            self.course = "a"
        self.targets = COURSE_TARGETS[self.course]
        self.capture_dir = Path(str(self.get_parameter("capture_dir").value))
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.surface_radius = float(self.get_parameter("surface_capture_radius_m").value)
        self.underwater_radius = float(
            self.get_parameter("underwater_capture_radius_m").value
        )
        self.docking_radius = float(self.get_parameter("docking_contact_radius_m").value)
        self.min_color_pixels = int(self.get_parameter("min_color_pixels").value)

        self.pose_xy = None
        self.latest_front_image = None
        self.latest_down_image = None
        self.surface_photo = None
        self.underwater_photo = None
        self.touched_buoys = set()
        self.track_complete = False
        self.first_docking_contact_time = None
        self.last_status_time = 0.0

        self.status_pub = self.create_publisher(String, "/asv/mission/status", 10)
        self.create_subscription(Odometry, "/asv/odom", self.on_odom, 10)
        self.create_subscription(Image, "/asv/camera/front/image", self.on_front_image, 5)
        self.create_subscription(Image, "/asv/camera/down/image", self.on_down_image, 5)
        self.create_subscription(String, "/asv/navigation/status", self.on_nav_status, 10)
        self.create_timer(0.2, self.on_timer)

    def on_odom(self, msg):
        pose = msg.pose.pose.position
        self.pose_xy = (float(pose.x), float(pose.y))

    def on_front_image(self, msg):
        self.latest_front_image = msg

    def on_down_image(self, msg):
        self.latest_down_image = msg

    def on_nav_status(self, msg):
        if "mission_complete" in msg.data:
            self.track_complete = True

    def on_timer(self):
        if self.pose_xy is None:
            self.publish_status("waiting_for_pose")
            return

        x, y = self.pose_xy
        tx, ty = self.targets["surface"]
        ux, uy = self.targets["underwater"]
        dist_surf = math.hypot(x - tx, y - ty)
        dist_und = math.hypot(x - ux, y - uy)
        self.get_logger().debug(
            f"pose=({x:.2f}, {y:.2f}) "
            f"surf_dist={dist_surf:.2f} (r={self.surface_radius:.2f}) "
            f"und_dist={dist_und:.2f} (r={self.underwater_radius:.2f})"
        )

        self.check_surface_photo(x, y)
        self.check_underwater_photo(x, y)
        self.check_docking_contact(x, y)
        self.publish_status("active")

    def check_surface_photo(self, x, y):
        if self.surface_photo is not None:
            return
        tx, ty = self.targets["surface"]
        dist = math.hypot(x - tx, y - ty)
        if dist > self.surface_radius:
            return
        # Try to take photo only when green box is visible in camera
        green_pixels = _count_color_pixels(self.latest_front_image, "green")
        self.get_logger().debug(
            f"surface check: dist={dist:.2f}m green_px={green_pixels}"
        )
        if green_pixels >= self.min_color_pixels or dist <= 1.5:
            # Box visible in camera OR very close — take photo
            path = self.save_image(self.latest_front_image, "surface_green_box_front")
            if path is not None:
                self.surface_photo = path
                self.get_logger().info(
                    f"Surface photo captured! dist={dist:.2f}m green_px={green_pixels}"
                )

    def check_underwater_photo(self, x, y):
        if self.underwater_photo is not None:
            return
        tx, ty = self.targets["underwater"]
        dist = math.hypot(x - tx, y - ty)
        if dist > self.underwater_radius:
            return
        # Try to take photo only when blue box is visible in camera
        blue_pixels = _count_color_pixels(self.latest_down_image, "blue")
        self.get_logger().debug(
            f"underwater check: dist={dist:.2f}m blue_px={blue_pixels}"
        )
        if blue_pixels >= self.min_color_pixels or dist <= 1.5:
            # Box visible in camera OR very close — take photo
            path = self.save_image(self.latest_down_image, "underwater_blue_box_down")
            if path is not None:
                self.underwater_photo = path
                self.get_logger().info(
                    f"Underwater photo captured! dist={dist:.2f}m blue_px={blue_pixels}"
                )

    def check_docking_contact(self, x, y):
        for name, bx, by in self.targets["dock_buoys"]:
            if name in self.touched_buoys:
                continue
            if math.hypot(x - bx, y - by) <= self.docking_radius:
                self.touched_buoys.add(name)
                if self.first_docking_contact_time is None:
                    self.first_docking_contact_time = time.time()

    def save_image(self, msg, stem):
        if msg is None or not msg.data:
            return None
        encoding = msg.encoding.lower()
        suffix = "ppm" if encoding in ("rgb8", "bgr8", "rgba8", "bgra8") else "pgm"
        path = self.capture_dir / f"{stem}.{suffix}"
        try:
            if suffix == "ppm":
                payload = self.rgb_payload(msg, encoding)
                header = f"P6\n{msg.width} {msg.height}\n255\n".encode("ascii")
            else:
                payload = self.mono_payload(msg)
                header = f"P5\n{msg.width} {msg.height}\n255\n".encode("ascii")
            path.write_bytes(header + payload)
            self.get_logger().info(f"Saved mission image: {path}")
            return str(path)
        except Exception as exc:
            self.get_logger().warn(f"Failed to save mission image: {exc}")
            return None

    def rgb_payload(self, msg, encoding):
        source = bytes(msg.data)
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        rows = []
        for row in range(msg.height):
            start = row * msg.step
            end = start + msg.width * channels
            raw = source[start:end]
            pixels = bytearray()
            for index in range(0, len(raw), channels):
                px = raw[index : index + channels]
                if len(px) < channels:
                    continue
                if encoding in ("bgr8", "bgra8"):
                    pixels.extend((px[2], px[1], px[0]))
                else:
                    pixels.extend((px[0], px[1], px[2]))
            rows.append(bytes(pixels))
        return b"".join(rows)

    def mono_payload(self, msg):
        source = bytes(msg.data)
        rows = []
        for row in range(msg.height):
            start = row * msg.step
            rows.append(source[start : start + msg.width])
        return b"".join(rows)

    def publish_status(self, state):
        now = time.monotonic()
        if now - self.last_status_time < 1.0:
            return
        self.last_status_time = now
        completed = all(
            [
                self.track_complete,
                self.surface_photo is not None,
                self.underwater_photo is not None,
                len(self.touched_buoys) > 0,
            ]
        )
        status_msg = (
            f"state={state} course={self.course.upper()} "
            f"track_complete={self.track_complete} "
            f"surface_photo={'yes' if self.surface_photo else 'no'} "
            f"underwater_photo={'yes' if self.underwater_photo else 'no'} "
            f"docking_touched={len(self.touched_buoys)} "
            f"touched={','.join(sorted(self.touched_buoys)) if self.touched_buoys else '-'} "
            f"mission_complete={completed}"
        )
        self.get_logger().info(f"STATUS: {status_msg}")
        self.status_pub.publish(String(data=status_msg))


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
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
