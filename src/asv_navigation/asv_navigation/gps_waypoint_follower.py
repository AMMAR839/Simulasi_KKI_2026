#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import String


EARTH_RADIUS_M = 6378137.0


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def gps_to_enu(lat, lon, origin_lat, origin_lon):
    lat_rad = math.radians(lat)
    origin_lat_rad = math.radians(origin_lat)
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    x = EARTH_RADIUS_M * d_lon * math.cos(0.5 * (lat_rad + origin_lat_rad))
    y = EARTH_RADIUS_M * d_lat
    return x, y


class GpsWaypointFollower(Node):
    """Marine line-of-sight path follower using GPS/IMU and LiDAR stop/avoid."""

    def __init__(self):
        super().__init__("gps_waypoint_follower")
        self.declare_parameter(
            "waypoint_file",
            "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/"
            "src/asv_navigation/config/kki_waypoints.yaml",
        )
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_auto")
        self.declare_parameter("reach_radius_m", 0.75)
        # Main autonomous speed cap. Keep this lower than the simulated hull limit.
        self.declare_parameter("max_speed_mps", 0.62)
        self.declare_parameter("min_speed_mps", 0.12)
        self.declare_parameter("heading_kp", 0.95)
        self.declare_parameter("heading_kd", 0.18)
        self.declare_parameter("max_yaw_rate_radps", 0.58)
        self.declare_parameter("obstacle_stop_distance_m", 0.38)
        self.declare_parameter("obstacle_slow_distance_m", 0.85)
        self.declare_parameter("lidar_self_filter_distance_m", 0.45)
        self.declare_parameter("front_scan_half_angle_deg", 8.0)
        self.declare_parameter("side_scan_min_angle_deg", 35.0)
        self.declare_parameter("side_scan_max_angle_deg", 105.0)
        self.declare_parameter("prefer_gps", True)
        self.declare_parameter("allow_odom_fallback", True)
        self.declare_parameter("gps_fallback_max_distance_m", 60.0)
        self.declare_parameter("speed_filter_alpha", 0.35)
        self.declare_parameter("yaw_filter_alpha", 0.38)
        self.declare_parameter("path_lookahead_distance_m", 2.10)
        self.declare_parameter("cross_track_gain", 1.20)
        self.declare_parameter("cross_track_slow_distance_m", 0.80)
        self.declare_parameter("segment_pass_margin_m", 0.65)
        self.declare_parameter("gate_cross_track_radius_m", 0.85)
        self.declare_parameter("turn_slow_heading_rad", 0.85)
        self.declare_parameter("approach_slow_distance_m", 1.20)
        self.declare_parameter("compass_yaw_offset_rad", 0.0)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("lidar_enabled", True)
        self.declare_parameter("lidar_process_every_n_scans", 3)
        self.declare_parameter("lidar_sample_stride", 3)
        self.declare_parameter("lidar_detection_max_distance_m", 1.80)

        self.waypoints, self.origin_lat, self.origin_lon = self.load_waypoints(
            Path(str(self.get_parameter("waypoint_file").value))
        )
        self.apply_navigation_parameters(self.parameter_values())

        self.current_fix = None
        self.current_odom = None
        self.current_odom_yaw = None
        self.current_yaw = None
        self.front_obstacle = math.inf
        self.left_obstacle = math.inf
        self.right_obstacle = math.inf
        self.current_index = 0
        self.last_heading_error = 0.0
        self.last_position = None
        self.filtered_speed = 0.0
        self.filtered_yaw_rate = 0.0
        self.scan_count = 0
        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.status_pub = self.create_publisher(String, "/asv/navigation/status", 10)

        self.create_subscription(NavSatFix, "/asv/gps/fix", self.on_gps, 10)
        self.create_subscription(Odometry, "/asv/odom", self.on_odom, 10)
        self.create_subscription(Imu, "/asv/imu/data", self.on_imu, 10)
        self.create_subscription(LaserScan, "/asv/lidar/scan", self.on_scan, 10)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

    def load_waypoints(self, path: Path):
        if not path.exists():
            raise FileNotFoundError(f"Waypoint file does not exist: {path}")
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)

        origin = data.get("origin", {})
        origin_lat = float(origin.get("latitude", 1.470000))
        origin_lon = float(origin.get("longitude", 102.110000))
        waypoints = []
        for item in data.get("waypoints", []):
            if "x" in item and "y" in item:
                x = float(item["x"])
                y = float(item["y"])
            else:
                x, y = gps_to_enu(
                    float(item["latitude"]),
                    float(item["longitude"]),
                    origin_lat,
                    origin_lon,
                )
            waypoints.append(
                {
                    "name": str(item.get("name", f"wp_{len(waypoints) + 1}")),
                    "x": x,
                    "y": y,
                    "speed": float(item.get("speed", data.get("default_speed_mps", 0.8))),
                }
            )
        if not waypoints:
            raise ValueError(f"No waypoints found in {path}")
        return waypoints, origin_lat, origin_lon

    def parameter_values(self):
        return {
            "reach_radius_m": float(self.get_parameter("reach_radius_m").value),
            "max_speed_mps": float(self.get_parameter("max_speed_mps").value),
            "min_speed_mps": float(self.get_parameter("min_speed_mps").value),
            "heading_kp": float(self.get_parameter("heading_kp").value),
            "heading_kd": float(self.get_parameter("heading_kd").value),
            "max_yaw_rate_radps": float(
                self.get_parameter("max_yaw_rate_radps").value
            ),
            "obstacle_stop_distance_m": float(
                self.get_parameter("obstacle_stop_distance_m").value
            ),
            "obstacle_slow_distance_m": float(
                self.get_parameter("obstacle_slow_distance_m").value
            ),
            "lidar_self_filter_distance_m": float(
                self.get_parameter("lidar_self_filter_distance_m").value
            ),
            "front_scan_half_angle_deg": float(
                self.get_parameter("front_scan_half_angle_deg").value
            ),
            "side_scan_min_angle_deg": float(
                self.get_parameter("side_scan_min_angle_deg").value
            ),
            "side_scan_max_angle_deg": float(
                self.get_parameter("side_scan_max_angle_deg").value
            ),
            "prefer_gps": bool(self.get_parameter("prefer_gps").value),
            "allow_odom_fallback": bool(
                self.get_parameter("allow_odom_fallback").value
            ),
            "gps_fallback_max_distance_m": float(
                self.get_parameter("gps_fallback_max_distance_m").value
            ),
            "speed_filter_alpha": float(
                self.get_parameter("speed_filter_alpha").value
            ),
            "yaw_filter_alpha": float(self.get_parameter("yaw_filter_alpha").value),
            "path_lookahead_distance_m": float(
                self.get_parameter("path_lookahead_distance_m").value
            ),
            "cross_track_gain": float(self.get_parameter("cross_track_gain").value),
            "cross_track_slow_distance_m": float(
                self.get_parameter("cross_track_slow_distance_m").value
            ),
            "segment_pass_margin_m": float(
                self.get_parameter("segment_pass_margin_m").value
            ),
            "gate_cross_track_radius_m": float(
                self.get_parameter("gate_cross_track_radius_m").value
            ),
            "turn_slow_heading_rad": float(
                self.get_parameter("turn_slow_heading_rad").value
            ),
            "approach_slow_distance_m": float(
                self.get_parameter("approach_slow_distance_m").value
            ),
            "compass_yaw_offset_rad": float(
                self.get_parameter("compass_yaw_offset_rad").value
            ),
            "lidar_enabled": bool(self.get_parameter("lidar_enabled").value),
            "lidar_process_every_n_scans": int(
                self.get_parameter("lidar_process_every_n_scans").value
            ),
            "lidar_sample_stride": int(
                self.get_parameter("lidar_sample_stride").value
            ),
            "lidar_detection_max_distance_m": float(
                self.get_parameter("lidar_detection_max_distance_m").value
            ),
        }

    def apply_navigation_parameters(self, values):
        self.reach_radius = max(0.05, values["reach_radius_m"])
        self.max_speed = max(0.0, values["max_speed_mps"])
        self.min_speed = max(0.0, min(values["min_speed_mps"], self.max_speed))
        self.heading_kp = max(0.0, values["heading_kp"])
        self.heading_kd = max(0.0, values["heading_kd"])
        self.max_yaw_rate = max(0.01, values["max_yaw_rate_radps"])
        self.stop_distance = max(0.05, values["obstacle_stop_distance_m"])
        self.slow_distance = max(self.stop_distance, values["obstacle_slow_distance_m"])
        self.lidar_self_filter = max(0.0, values["lidar_self_filter_distance_m"])
        self.front_scan_half_angle = math.radians(
            max(1.0, values["front_scan_half_angle_deg"])
        )
        self.side_scan_min_angle = math.radians(
            max(0.0, values["side_scan_min_angle_deg"])
        )
        self.side_scan_max_angle = math.radians(
            max(values["side_scan_min_angle_deg"], values["side_scan_max_angle_deg"])
        )
        self.prefer_gps = values["prefer_gps"]
        self.allow_odom_fallback = values["allow_odom_fallback"]
        self.gps_fallback_max_distance = max(
            1.0, values["gps_fallback_max_distance_m"]
        )
        self.speed_filter_alpha = max(0.01, min(1.0, values["speed_filter_alpha"]))
        self.yaw_filter_alpha = max(0.01, min(1.0, values["yaw_filter_alpha"]))
        self.path_lookahead = max(0.20, values["path_lookahead_distance_m"])
        self.cross_track_gain = max(0.0, values["cross_track_gain"])
        self.cross_track_slow_distance = max(
            0.05, values["cross_track_slow_distance_m"]
        )
        self.segment_pass_margin = max(0.0, values["segment_pass_margin_m"])
        self.gate_cross_track_radius = max(
            0.05, values["gate_cross_track_radius_m"]
        )
        self.turn_slow_heading = max(0.05, values["turn_slow_heading_rad"])
        self.approach_slow_distance = max(0.05, values["approach_slow_distance_m"])
        self.compass_yaw_offset = values["compass_yaw_offset_rad"]
        self.lidar_enabled = values["lidar_enabled"]
        self.lidar_process_every = max(1, int(values["lidar_process_every_n_scans"]))
        self.lidar_sample_stride = max(1, int(values["lidar_sample_stride"]))
        self.lidar_detection_max = max(
            self.stop_distance, values["lidar_detection_max_distance_m"]
        )

    def on_parameter_update(self, parameters):
        values = self.parameter_values()
        waypoint_file = None
        for parameter in parameters:
            if parameter.name == "waypoint_file":
                waypoint_file = Path(str(parameter.value))
            elif parameter.name in values:
                values[parameter.name] = parameter.value
        try:
            if waypoint_file is not None:
                self.waypoints, self.origin_lat, self.origin_lon = self.load_waypoints(
                    waypoint_file
                )
                self.current_index = 0
                self.last_heading_error = 0.0
                self.last_position = None
            self.apply_navigation_parameters(values)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_gps(self, msg: NavSatFix):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.current_fix = msg

    def on_odom(self, msg: Odometry):
        self.current_odom = msg
        self.current_odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_imu(self, msg: Imu):
        self.current_yaw = yaw_from_quaternion(msg.orientation)

    def on_scan(self, msg: LaserScan):
        if not self.lidar_enabled:
            self.front_obstacle = math.inf
            self.left_obstacle = math.inf
            self.right_obstacle = math.inf
            return
        if not msg.ranges:
            return
        self.scan_count += 1
        if (self.scan_count - 1) % self.lidar_process_every != 0:
            return

        front = []
        left = []
        right = []
        valid_min = max(float(msg.range_min), self.lidar_self_filter)
        valid_max = min(float(msg.range_max), self.lidar_detection_max)

        # Process only selected scans and beams. This keeps obstacle detection
        # effective in the front sectors without spending CPU on every LiDAR ray.
        for index in range(0, len(msg.ranges), self.lidar_sample_stride):
            distance = msg.ranges[index]
            angle = msg.angle_min + index * msg.angle_increment
            if math.isfinite(distance) and valid_min <= distance <= valid_max:
                if abs(angle) <= self.front_scan_half_angle:
                    front.append(distance)
                elif self.side_scan_min_angle < angle < self.side_scan_max_angle:
                    left.append(distance)
                elif -self.side_scan_max_angle < angle < -self.side_scan_min_angle:
                    right.append(distance)
        self.front_obstacle = min(front) if front else math.inf
        self.left_obstacle = min(left) if left else math.inf
        self.right_obstacle = min(right) if right else math.inf

    def on_timer(self):
        cmd = Twist()
        pose = self.current_pose()
        if pose is None:
            self.publish_zero("waiting_for_position")
            return

        x, y, yaw, position_source = pose
        if yaw is None:
            yaw = self.estimate_yaw_from_motion(x, y)
        if yaw is None:
            self.publish_zero("waiting_for_heading")
            return

        reached = self.advance_completed_segments(x, y)
        if self.current_index >= len(self.waypoints):
            self.publish_zero("mission_complete")
            return

        guidance = self.compute_guidance(x, y)
        target = guidance["target"]
        distance = guidance["distance"]
        desired_heading = guidance["desired_heading"]
        heading_error = normalize_angle(desired_heading - yaw)
        derivative = normalize_angle(heading_error - self.last_heading_error)
        self.last_heading_error = heading_error

        yaw_rate = self.heading_kp * heading_error + self.heading_kd * derivative
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        # Speed is limited by both the global autonomous cap and each waypoint.
        speed = min(self.max_speed, float(target["speed"]))
        speed *= max(
            0.20,
            min(1.0, 1.0 - abs(heading_error) / max(self.turn_slow_heading, 1e-6)),
        )
        speed *= max(
            0.35,
            min(
                1.0,
                1.0
                - abs(guidance["cross_track_error"])
                / max(self.cross_track_slow_distance, 1e-6),
            ),
        )
        speed *= max(
            0.35,
            min(1.0, distance / max(self.approach_slow_distance, self.reach_radius)),
        )

        avoid_note = "clear"
        if self.front_obstacle < self.stop_distance:
            speed = 0.0
            yaw_rate = -0.55 if self.left_obstacle < self.right_obstacle else 0.55
            yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
            avoid_note = "stop_turn"
        elif self.front_obstacle < self.slow_distance:
            scale = max(0.25, self.front_obstacle / self.slow_distance)
            speed = max(self.min_speed, speed * scale)
            if self.left_obstacle < self.right_obstacle:
                yaw_rate -= 0.20
                avoid_note = "slow_bias_right"
            elif self.right_obstacle < self.left_obstacle:
                yaw_rate += 0.20
                avoid_note = "slow_bias_left"
            else:
                avoid_note = "slow"
            yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))

        self.filtered_speed += self.speed_filter_alpha * (speed - self.filtered_speed)
        self.filtered_yaw_rate += self.yaw_filter_alpha * (
            yaw_rate - self.filtered_yaw_rate
        )
        cmd.linear.x = self.filtered_speed
        cmd.angular.z = self.filtered_yaw_rate
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(
            String(
                data=(
                    f"source={position_source} target={target['name']} "
                    f"index={self.current_index + 1}/{len(self.waypoints)} "
                    f"distance={distance:.2f} along={guidance['along']:.2f} "
                    f"cte={guidance['cross_track_error']:.2f} "
                    f"heading_error={heading_error:.2f} "
                    f"cmd_v={cmd.linear.x:.2f} cmd_w={cmd.angular.z:.2f} "
                    f"front_lidar={self.front_obstacle:.2f} avoid={avoid_note} "
                    f"reached={','.join(reached) if reached else '-'}"
                )
            )
        )

    def advance_completed_segments(self, x: float, y: float) -> list[str]:
        reached = []
        while self.current_index < len(self.waypoints):
            target = self.waypoints[self.current_index]
            dx = target["x"] - x
            dy = target["y"] - y
            distance = math.hypot(dx, dy)
            if distance <= self.reach_radius:
                reached.append(target["name"])
                self.current_index += 1
                self.last_heading_error = 0.0
                continue

            if self.current_index > 0:
                start = self.waypoints[self.current_index - 1]
                along, cross_track, seg_len = self.segment_errors(x, y, start, target)
                # A waypoint is complete when the boat is close enough or has
                # crossed the waypoint line while still near the gate centerline.
                past_gate = along >= seg_len - self.segment_pass_margin
                close_to_gate_line = abs(cross_track) <= self.gate_cross_track_radius
                if past_gate and close_to_gate_line:
                    reached.append(target["name"])
                    self.current_index += 1
                    self.last_heading_error = 0.0
                    continue
            break
        return reached

    def compute_guidance(self, x: float, y: float) -> dict:
        target = self.waypoints[self.current_index]
        dx = target["x"] - x
        dy = target["y"] - y
        distance = math.hypot(dx, dy)

        if self.current_index <= 0:
            desired_heading = math.atan2(dy, dx)
            return {
                "target": target,
                "distance": distance,
                "desired_heading": desired_heading,
                "along": 0.0,
                "cross_track_error": 0.0,
            }

        start = self.waypoints[self.current_index - 1]
        along, cross_track, seg_len = self.segment_errors(x, y, start, target)
        guide_x, guide_y = self.guidance_point(x, y, target)
        desired_heading = math.atan2(guide_y - y, guide_x - x)
        cross_track_correction = math.atan2(
            -self.cross_track_gain * cross_track,
            max(self.path_lookahead, 1e-6),
        )
        cross_track_correction = max(-0.35, min(0.35, cross_track_correction))
        desired_heading = normalize_angle(desired_heading + cross_track_correction)
        return {
            "target": target,
            "distance": distance,
            "desired_heading": desired_heading,
            "along": along,
            "cross_track_error": cross_track,
        }

    def segment_errors(self, x: float, y: float, start: dict, target: dict):
        sx = start["x"]
        sy = start["y"]
        ex = target["x"]
        ey = target["y"]
        vx = ex - sx
        vy = ey - sy
        seg_len = math.hypot(vx, vy)
        if seg_len < 1e-6:
            return 0.0, 0.0, 0.0
        ux = vx / seg_len
        uy = vy / seg_len
        rel_x = x - sx
        rel_y = y - sy
        along = rel_x * ux + rel_y * uy
        cross_track = -uy * rel_x + ux * rel_y
        return along, cross_track, seg_len

    def guidance_point(self, x: float, y: float, target: dict) -> tuple[float, float]:
        if self.current_index <= 0:
            return target["x"], target["y"]

        start_index = self.current_index - 1
        start = self.waypoints[start_index]
        along, _, seg_len = self.segment_errors(x, y, start, target)
        distance_ahead = self.path_lookahead

        for index in range(start_index, len(self.waypoints) - 1):
            seg_start = self.waypoints[index]
            seg_end = self.waypoints[index + 1]
            sx = seg_start["x"]
            sy = seg_start["y"]
            ex = seg_end["x"]
            ey = seg_end["y"]
            vx = ex - sx
            vy = ey - sy
            current_seg_len = math.hypot(vx, vy)
            if current_seg_len < 1e-6:
                continue

            if index == start_index:
                segment_offset = max(0.0, min(current_seg_len, along))
            else:
                segment_offset = 0.0

            remaining = current_seg_len - segment_offset
            if distance_ahead <= remaining:
                ratio = (segment_offset + distance_ahead) / current_seg_len
                return sx + ratio * vx, sy + ratio * vy
            distance_ahead -= remaining

        last = self.waypoints[-1]
        return last["x"], last["y"]

    def current_pose(self):
        gps_pose = self.gps_pose()
        if self.prefer_gps and gps_pose is not None:
            x, y = gps_pose
            nearest_waypoint = min(
                math.hypot(wp["x"] - x, wp["y"] - y) for wp in self.waypoints
            )
            if nearest_waypoint <= self.gps_fallback_max_distance:
                yaw = self.current_yaw
                if yaw is None:
                    yaw = self.current_odom_yaw
                yaw = self.apply_heading_offset(yaw)
                return x, y, yaw, "gps_compass"

        if self.allow_odom_fallback and self.current_odom is not None:
            pose = self.current_odom.pose.pose
            x = float(pose.position.x)
            y = float(pose.position.y)
            return (
                x,
                y,
                self.apply_heading_offset(self.current_odom_yaw),
                "odom_fallback",
            )

        if gps_pose is not None:
            x, y = gps_pose
            yaw = self.current_yaw
            if yaw is None:
                yaw = self.current_odom_yaw
            yaw = self.apply_heading_offset(yaw)
            return x, y, yaw, "gps_compass_unchecked"

        return None

    def apply_heading_offset(self, yaw):
        if yaw is None:
            return None
        return normalize_angle(yaw + self.compass_yaw_offset)

    def gps_pose(self):
        if self.current_fix is None:
            return None
        x, y = gps_to_enu(
            self.current_fix.latitude,
            self.current_fix.longitude,
            self.origin_lat,
            self.origin_lon,
        )
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return x, y

    def publish_zero(self, status: str):
        self.filtered_speed = 0.0
        self.filtered_yaw_rate = 0.0
        self.cmd_pub.publish(Twist())
        self.status_pub.publish(String(data=status))

    def estimate_yaw_from_motion(self, x, y):
        if self.last_position is None:
            self.last_position = (x, y)
            return None
        lx, ly = self.last_position
        self.last_position = (x, y)
        if math.hypot(x - lx, y - ly) < 0.05:
            return None
        return math.atan2(y - ly, x - lx)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
