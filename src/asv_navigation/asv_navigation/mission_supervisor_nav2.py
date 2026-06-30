#!/usr/bin/env python3
"""
mission_supervisor_nav2.py — KKI 2026 Mission Supervisor (Nav2 Edition)
========================================================================
Full Nav2 migration: menggantikan gps_waypoint_follower.py + mission_supervisor.py

Menggunakan:
  - nav2_msgs/FollowWaypoints action      → waypoint survey
  - opennav_docking (DockRobot action)    → docking ke buoy biru
  - Behavior Tree via bt_navigator        → replanning + recovery otomatis

Urutan misi:
  1. Muat waypoints dari YAML (format yang sama dengan sebelumnya)
  2. Kirim semua waypoints ke Nav2 followWaypoints
  3. Monitor feedback.current_waypoint untuk trigger foto
  4. Setelah semua waypoints selesai → trigger opennav DockRobot
  5. Log status misi lengkap
"""

import math
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.action import DockRobot, FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ──────────────────────────────────────────────────────────────────────────────
# Helper: Euler yaw → quaternion (z-axis rotation only)
# ──────────────────────────────────────────────────────────────────────────────
def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ──────────────────────────────────────────────────────────────────────────────
# Helper: count color pixels in camera image
# ──────────────────────────────────────────────────────────────────────────────
def _count_color_pixels(msg, target: str) -> int:
    if msg is None or not msg.data:
        return 0
    enc = msg.encoding.lower()
    channels = 4 if enc in ("rgba8", "bgra8") else 3
    src = bytes(msg.data)
    count = 0
    stride = 6
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
                if g > r + 30 and g > b + 30 and g > 80:
                    count += 1
            elif target == "blue":
                if b > r + 30 and b > g + 20 and b > 80:
                    count += 1
    return count


# ──────────────────────────────────────────────────────────────────────────────
# Course definitions (same as old mission_supervisor.py)
# ──────────────────────────────────────────────────────────────────────────────
COURSE_CONFIG = {
    "a": {
        "surface_target": (-10.2, -5.4),
        "underwater_target": (-11.0, -7.7),
        "dock_id": "kki_buoy_dock_a",
        "dock_buoys": [
            ("docking_a_blue_1", 12.35, -10.0),
            ("docking_a_blue_2", 12.35, -9.7),
            ("docking_a_blue_3", 12.35, -9.4),
        ],
    },
    "b": {
        "surface_target": (10.2, -5.4),
        "underwater_target": (11.0, -7.7),
        "dock_id": "kki_buoy_dock_b",
        "dock_buoys": [
            ("docking_b_blue_1", -12.35, -10.0),
            ("docking_b_blue_2", -12.35, -9.7),
            ("docking_b_blue_3", -12.35, -9.4),
        ],
    },
}

# Waypoint names that trigger photo capture (from YAML)
SURFACE_PHOTO_WPS = {"surface_photo_green_box", "surface_photo_approach"}
UNDERWATER_PHOTO_WPS = {"underwater_photo_blue_box", "underwater_photo_approach"}
DOCKING_WPS = {"dock_contact_blue_buoys", "dock_side_align"}


# ──────────────────────────────────────────────────────────────────────────────
# Main Mission Supervisor
# ──────────────────────────────────────────────────────────────────────────────
class MissionSupervisorNav2(Node):
    """
    KKI 2026 mission supervisor using Nav2 full stack.

    Replaces:
      - gps_waypoint_follower.py  → nav2_simple_commander.followWaypoints
      - mission_supervisor.py     → this class
      - custom docking logic      → opennav_docking DockRobot action
    """

    def __init__(self):
        super().__init__("mission_supervisor_nav2")

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter("course", "a")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter(
            "capture_dir",
            "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_captures",
        )
        self.declare_parameter("surface_capture_radius_m", 4.0)
        self.declare_parameter("underwater_capture_radius_m", 3.0)
        self.declare_parameter("docking_contact_radius_m", 1.20)
        self.declare_parameter("min_color_pixels", 20)
        self.declare_parameter("use_docking_server", True)
        self.declare_parameter("speed_limit_topic", "/speed_limit")
        self.declare_parameter("manual_docking_timeout_s", 45.0)
        self.declare_parameter("manual_docking_linear_mps", 0.14)
        self.declare_parameter("manual_docking_angular_radps", 0.45)
        self.declare_parameter("initial_waypoint_skip_radius_m", 1.0)

        self.course = str(self.get_parameter("course").value).strip().lower()
        if self.course not in COURSE_CONFIG:
            self.get_logger().warn(f"Unknown course '{self.course}', defaulting to 'a'")
            self.course = "a"
        self.config = COURSE_CONFIG[self.course]
        self.capture_dir = Path(str(self.get_parameter("capture_dir").value))
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.surface_radius = float(self.get_parameter("surface_capture_radius_m").value)
        self.underwater_radius = float(
            self.get_parameter("underwater_capture_radius_m").value
        )
        self.docking_radius = float(
            self.get_parameter("docking_contact_radius_m").value
        )
        self.min_color_pixels = int(self.get_parameter("min_color_pixels").value)
        self.use_docking_server = bool(
            self.get_parameter("use_docking_server").value
        )
        self.manual_docking_timeout = float(
            self.get_parameter("manual_docking_timeout_s").value
        )
        self.manual_docking_linear = float(
            self.get_parameter("manual_docking_linear_mps").value
        )
        self.manual_docking_angular = float(
            self.get_parameter("manual_docking_angular_radps").value
        )
        self.initial_skip_radius = float(
            self.get_parameter("initial_waypoint_skip_radius_m").value
        )

        # ── State ────────────────────────────────────────────
        self.pose_xy = None
        self.pose_yaw = 0.0
        self.latest_front_image = None
        self.latest_down_image = None
        self.surface_photo = None
        self.underwater_photo = None
        self.touched_buoys: set = set()
        self.first_docking_contact_time = None
        self.mission_phase = "init"   # init → survey → docking → done
        self.last_status_time = 0.0
        self.survey_complete = False
        self.track_complete = False
        self._last_waypoint_idx = None
        self._current_waypoint_idx = None
        self._survey_goal_sent = False
        self._survey_goal_handle = None
        self._survey_result_future = None
        self._nav2_state_future = None
        self._dock_goal_handle = None
        self._last_wait_log_time = 0.0
        self._manual_docking_active = False
        self._manual_docking_start_time = None
        self._last_speed_limit_publish_time = 0.0

        # ── Subscribers ──────────────────────────────────────
        # Subscribe to both EKF output and raw planar_odom for robustness
        self.create_subscription(Odometry, "/odometry/local", self.on_odom, 10)
        self.create_subscription(Odometry, "/asv/planar_odom", self.on_odom, 10)
        self.create_subscription(
            Image, "/asv/camera/front/image", self.on_front_image, 5
        )
        self.create_subscription(
            Image, "/asv/camera/down/image", self.on_down_image, 5
        )

        # ── Publishers ───────────────────────────────────────
        self.status_pub = self.create_publisher(String, "/asv/mission/status", 10)
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            str(self.get_parameter("speed_limit_topic").value),
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Docking action client ────────────────────────────
        self._dock_client = ActionClient(self, DockRobot, "/dock_robot")

        # ── Nav2 action/lifecycle clients ────────────────────
        self._waypoint_client = ActionClient(
            self, FollowWaypoints, "/follow_waypoints"
        )
        self._bt_state_client = self.create_client(
            GetState, "/bt_navigator/get_state"
        )

        # ── Load waypoints ───────────────────────────────────
        wp_file = str(self.get_parameter("waypoint_file").value)
        self.waypoints_data, self.origin = self._load_waypoints(wp_file)
        self.nav_waypoints_data = [
            wp for wp in self.waypoints_data if not wp.get("docking", False)
        ]
        if not self.nav_waypoints_data:
            self.nav_waypoints_data = list(self.waypoints_data)
        self.waypoint_names = [wp["name"] for wp in self.nav_waypoints_data]
        self.get_logger().info(
            f"Loaded {len(self.waypoints_data)} waypoints from: {wp_file}; "
            f"{len(self.nav_waypoints_data)} sent to Nav2 survey"
        )

        self._nav2_ready = False
        # ── Start mission on a timer (wait for Nav2 to be active) ──
        self.create_timer(1.0, self._check_nav2_ready)

    # ── Data callbacks ────────────────────────────────────────
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose_xy = (float(p.x), float(p.y))
        self.pose_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def on_front_image(self, msg):
        self.latest_front_image = msg

    def on_down_image(self, msg):
        self.latest_down_image = msg

    # ── Nav2 ready check ──────────────────────────────────────
    def _check_nav2_ready(self):
        if self._nav2_ready:
            return
        now = time.monotonic()
        if not self._bt_state_client.service_is_ready():
            if now - self._last_wait_log_time > 5.0:
                self.get_logger().info("Waiting for bt_navigator/get_state service...")
                self._last_wait_log_time = now
            return

        if self._nav2_state_future is None:
            self._nav2_state_future = self._bt_state_client.call_async(
                GetState.Request()
            )
            return

        if not self._nav2_state_future.done():
            return

        try:
            result = self._nav2_state_future.result()
            state = result.current_state.label if result is not None else "unknown"
            self._nav2_state_future = None
            if state != "active":
                if now - self._last_wait_log_time > 5.0:
                    self.get_logger().info(f"Waiting for Nav2 active state: {state}")
                    self._last_wait_log_time = now
                return

            self._nav2_ready = True
            self.get_logger().info("Nav2 is active — starting mission!")
            self.create_timer(0.1, self._run_mission_tick)
        except Exception as e:
            self._nav2_state_future = None
            self.get_logger().warn(f"Waiting for Nav2: {e}")

    # ──────────────────────────────────────────────────────────
    # Mission State Machine (called at 10 Hz)
    # ──────────────────────────────────────────────────────────
    def _run_mission_tick(self):
        now = time.monotonic()
        if now - self.last_status_time >= 1.0:
            self._publish_status()
            self.last_status_time = now

        if self.mission_phase == "init":
            self._start_survey()

        elif self.mission_phase == "survey":
            self._monitor_survey()

        elif self.mission_phase == "docking":
            if self.pose_xy is not None:
                self._check_docking_contact(*self.pose_xy)
                self._complete_docking_if_contacted()
            if self._manual_docking_active:
                self._run_manual_docking(now)

        elif self.mission_phase == "done":
            pass  # Mission complete

    # ──────────────────────────────────────────────────────────
    # Phase 1: Survey — send all waypoints to Nav2
    # ──────────────────────────────────────────────────────────
    def _start_survey(self):
        if self._survey_goal_sent:
            return
        if not self._waypoint_client.wait_for_server(timeout_sec=0.0):
            now = time.monotonic()
            if now - self._last_wait_log_time > 5.0:
                self.get_logger().info("Waiting for /follow_waypoints action server...")
                self._last_wait_log_time = now
            return

        self.get_logger().info(
            f"Starting survey: {len(self.nav_waypoints_data)} Nav2 waypoints, "
            f"course {self.course.upper()}"
        )
        self.mission_phase = "survey"
        self._skip_initial_waypoint_if_already_at_spawn()

        # Build PoseStamped list for followWaypoints
        poses = self._build_nav2_poses(self.nav_waypoints_data)
        self._publish_waypoint_speed_limit(0)

        # Start waypoint following (non-blocking)
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._survey_goal_sent = True
        future = self._waypoint_client.send_goal_async(
            goal, feedback_callback=self._waypoint_feedback_cb
        )
        future.add_done_callback(self._waypoint_goal_response_cb)
        self.get_logger().info(f"followWaypoints sent ({len(poses)} poses)")

    def _skip_initial_waypoint_if_already_at_spawn(self):
        if self.pose_xy is None or len(self.nav_waypoints_data) < 2:
            return

        first_wp = self.nav_waypoints_data[0]
        distance = math.hypot(
            self.pose_xy[0] - first_wp["x"],
            self.pose_xy[1] - first_wp["y"],
        )
        if distance > self.initial_skip_radius:
            return

        self.get_logger().info(
            f"Skipping initial waypoint '{first_wp['name']}' "
            f"(already within {distance:.2f} m of spawn pose)"
        )
        self.nav_waypoints_data = self.nav_waypoints_data[1:]
        self.waypoint_names = [wp["name"] for wp in self.nav_waypoints_data]

    def _monitor_survey(self):
        """Monitor followWaypoints progress, trigger photos at right waypoints."""
        now = time.monotonic()
        current_idx = self._current_waypoint_idx
        if current_idx is None:
            current_idx = 0

        if 0 <= current_idx < len(self.waypoint_names):
            current_wp_name = self.waypoint_names[current_idx]
            if current_idx != self._last_waypoint_idx:
                self._last_waypoint_idx = current_idx
                self._publish_waypoint_speed_limit(current_idx)
            elif now - self._last_speed_limit_publish_time >= 1.0:
                self._publish_waypoint_speed_limit(current_idx, log=False)

            # Trigger photo at photo waypoints
            if self.pose_xy is not None:
                if current_wp_name in SURFACE_PHOTO_WPS:
                    self._try_surface_photo(*self.pose_xy, wp_name=current_wp_name)
                elif current_wp_name in UNDERWATER_PHOTO_WPS:
                    self._try_underwater_photo(*self.pose_xy, wp_name=current_wp_name)

        # Also check by proximity (fallback)
        if self.pose_xy is not None:
            self._try_surface_photo(*self.pose_xy)
            self._try_underwater_photo(*self.pose_xy)

    def _waypoint_feedback_cb(self, feedback_msg):
        self._current_waypoint_idx = int(feedback_msg.feedback.current_waypoint)

    def _waypoint_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"followWaypoints send failed: {exc}")
            self._start_docking()
            return

        if not goal_handle.accepted:
            self.get_logger().error("followWaypoints goal rejected; proceeding to docking")
            self._start_docking()
            return

        self._survey_goal_handle = goal_handle
        self.get_logger().info("followWaypoints goal accepted")
        self._survey_result_future = goal_handle.get_result_async()
        self._survey_result_future.add_done_callback(self._waypoint_result_cb)

    def _waypoint_result_cb(self, future):
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
            missed = list(getattr(result, "missed_waypoints", []))
        except Exception as exc:
            self.get_logger().error(f"followWaypoints result failed: {exc}")
            self._start_docking()
            return

        if status == GoalStatus.STATUS_SUCCEEDED and not missed:
            self.get_logger().info("Survey COMPLETE — all waypoints reached!")
            self.survey_complete = True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Survey CANCELED")
        elif missed:
            self.get_logger().error(
                f"Survey finished with missed waypoints: {missed}; proceeding to docking"
            )
        else:
            self.get_logger().error(
                f"Survey FAILED with action status {status}; proceeding to docking"
            )
        self._start_docking()

    # ──────────────────────────────────────────────────────────
    # Phase 2: Docking — use opennav_docking DockRobot action
    # ──────────────────────────────────────────────────────────
    def _start_docking(self):
        self.mission_phase = "docking"
        dock_id = self.config["dock_id"]

        if not self.use_docking_server:
            self.get_logger().info(
                "use_docking_server=false — using mission supervisor docking fallback"
            )
            self._start_manual_docking("docking server disabled")
            return

        self.get_logger().info(
            f"Starting docking via opennav_docking — dock_id='{dock_id}'"
        )

        # Wait for docking server
        if not self._dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Docking server not available; using fallback.")
            self._start_manual_docking("docking server unavailable")
            return

        # Send DockRobot action
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.dock_type = "kki_blue_buoys"
        goal.navigate_to_staging_pose = True  # Let Nav2 drive to staging pose

        future = self._dock_client.send_goal_async(
            goal, feedback_callback=self._dock_feedback_cb
        )
        future.add_done_callback(self._dock_goal_response_cb)

    def _dock_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Docking goal REJECTED by server!")
            self._start_manual_docking("docking goal rejected")
            return
        self._dock_goal_handle = goal_handle
        self.get_logger().info("Docking goal accepted — approaching buoys...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_cb)

    def _dock_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f"Docking feedback: state={fb.state} retries={fb.num_retries}"
        )

    def _dock_result_cb(self, future):
        if self.mission_phase == "done":
            return
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f"DOCKING COMPLETE! num_retries={result.num_retries}"
            )
            # Mark all buoys as touched
            for name, bx, by in self.config["dock_buoys"]:
                self.touched_buoys.add(name)
            if self.first_docking_contact_time is None:
                self.first_docking_contact_time = time.time()
            self.track_complete = self.survey_complete
            self._publish_stop()
            self.mission_phase = "done"
        else:
            self.get_logger().error(
                f"Docking FAILED: error_code={result.error_code}; using fallback"
            )
            self._start_manual_docking("docking action failed")

    def _required_docking_contacts(self) -> int:
        return max(1, len(self.config["dock_buoys"]))

    def _has_required_docking_contact(self) -> bool:
        return len(self.touched_buoys) >= self._required_docking_contacts()

    def _complete_docking_if_contacted(self):
        if not self._has_required_docking_contact():
            return

        if self._dock_goal_handle is not None:
            try:
                self._dock_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel docking goal: {exc}")
            self._dock_goal_handle = None

        self.track_complete = self.survey_complete
        self._manual_docking_active = False
        self._publish_stop()
        self.get_logger().info(
            "Docking contact complete; all docking buoys touched."
        )
        self.mission_phase = "done"

    def _start_manual_docking(self, reason: str):
        self.get_logger().warn(f"Manual docking fallback active: {reason}")
        self.mission_phase = "docking"
        self._manual_docking_active = True
        self._manual_docking_start_time = time.monotonic()

    def _run_manual_docking(self, now: float):
        if self.pose_xy is None:
            return
        x, y = self.pose_xy
        self._check_docking_contact(x, y)

        if self._has_required_docking_contact():
            self._complete_docking_if_contacted()
            return

        elapsed = now - (self._manual_docking_start_time or now)
        if elapsed > self.manual_docking_timeout:
            self._manual_docking_active = False
            self._publish_stop()
            self.get_logger().error("Manual docking fallback timed out.")
            self.mission_phase = "done"
            return

        _, dock_x, dock_y = self.config["dock_buoys"][1]
        desired_yaw = math.atan2(dock_y - y, dock_x - x)
        yaw_error = normalize_angle(desired_yaw - self.pose_yaw)
        distance = math.hypot(dock_x - x, dock_y - y)

        cmd = Twist()
        cmd.linear.x = clamp(distance * 0.25, 0.04, self.manual_docking_linear)
        if abs(yaw_error) > 0.8:
            cmd.linear.x = 0.04
        cmd.angular.z = clamp(
            1.2 * yaw_error,
            -self.manual_docking_angular,
            self.manual_docking_angular,
        )
        self.cmd_pub.publish(cmd)

    # ──────────────────────────────────────────────────────────
    # Photo capture helpers
    # ──────────────────────────────────────────────────────────
    def _try_surface_photo(self, x, y, wp_name=""):
        if self.surface_photo is not None:
            return
        tx, ty = self.config["surface_target"]
        dist = math.hypot(x - tx, y - ty)
        if dist > self.surface_radius:
            return
        green_px = _count_color_pixels(self.latest_front_image, "green")
        at_wp = wp_name in SURFACE_PHOTO_WPS
        if green_px >= self.min_color_pixels or dist <= 1.5 or at_wp:
            path = self._save_image(self.latest_front_image, "surface_green_box_front")
            if path:
                self.surface_photo = path
                self.get_logger().info(
                    f"[PHOTO] Surface green box captured! "
                    f"dist={dist:.2f}m green_px={green_px} wp={wp_name}"
                )

    def _try_underwater_photo(self, x, y, wp_name=""):
        if self.underwater_photo is not None:
            return
        tx, ty = self.config["underwater_target"]
        dist = math.hypot(x - tx, y - ty)
        if dist > self.underwater_radius:
            return
        blue_px = _count_color_pixels(self.latest_down_image, "blue")
        at_wp = wp_name in UNDERWATER_PHOTO_WPS
        if blue_px >= self.min_color_pixels or dist <= 1.5 or at_wp:
            path = self._save_image(self.latest_down_image, "underwater_blue_box_down")
            if path:
                self.underwater_photo = path
                self.get_logger().info(
                    f"[PHOTO] Underwater blue box captured! "
                    f"dist={dist:.2f}m blue_px={blue_px} wp={wp_name}"
                )

    def _check_docking_contact(self, x, y):
        for name, bx, by in self.config["dock_buoys"]:
            if name in self.touched_buoys:
                continue
            if math.hypot(x - bx, y - by) <= self.docking_radius:
                self.touched_buoys.add(name)
                if self.first_docking_contact_time is None:
                    self.first_docking_contact_time = time.time()
                self.get_logger().info(
                    f"[DOCKING] Contact with buoy '{name}' detected!"
                )

    def _publish_waypoint_speed_limit(self, waypoint_idx: int, log: bool = True):
        if not (0 <= waypoint_idx < len(self.nav_waypoints_data)):
            return
        speed = float(self.nav_waypoints_data[waypoint_idx].get("speed", 0.5))
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.percentage = False
        msg.speed_limit = clamp(speed, 0.05, 0.75)
        self.speed_limit_pub.publish(msg)
        self._last_speed_limit_publish_time = time.monotonic()
        if log:
            self.get_logger().info(
                f"[SPEED] waypoint={self.nav_waypoints_data[waypoint_idx]['name']} "
                f"limit={msg.speed_limit:.2f} m/s"
            )

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    # ──────────────────────────────────────────────────────────
    # Waypoint loading & pose building
    # ──────────────────────────────────────────────────────────
    def _load_waypoints(self, filepath: str):
        """Load kki_waypoints_*.yaml and return list of waypoint dicts."""
        path = Path(filepath)
        if not path.exists():
            self.get_logger().error(f"Waypoint file not found: {filepath}")
            return [], {}
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        origin = data.get("origin", {})
        waypoints = []
        for item in data.get("waypoints", []):
            waypoints.append(
                {
                    "name": str(item.get("name", f"wp_{len(waypoints)}")),
                    "x": float(item.get("x", 0.0)),
                    "y": float(item.get("y", 0.0)),
                    "speed": float(item.get("speed", 0.6)),
                    "docking": bool(item.get("docking", False)),
                }
            )
        return waypoints, origin

    def _build_nav2_poses(self, waypoints: list) -> list:
        """
        Convert waypoint dicts → PoseStamped list in map frame.
        Heading = direction leading into this waypoint (or forward from spawn for the first one).
        Docking-contact waypoints are filtered out before this method.
        """
        poses = []
        now = self.get_clock().now().to_msg()

        # Fallback spawn position if pose_xy is not yet populated
        start_x, start_y = 10.80, -8.70
        if self.pose_xy is not None:
            start_x, start_y = self.pose_xy

        for i, wp in enumerate(waypoints):
            # Compute heading leading into this waypoint
            if i == 0:
                dx = wp["x"] - start_x
                dy = wp["y"] - start_y
            else:
                dx = wp["x"] - waypoints[i - 1]["x"]
                dy = wp["y"] - waypoints[i - 1]["y"]

            yaw = math.atan2(dy, dx)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = wp["x"]
            pose.pose.position.y = wp["y"]
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            poses.append(pose)

        return poses

    # ──────────────────────────────────────────────────────────
    # Image saving
    # ──────────────────────────────────────────────────────────
    def _save_image(self, msg, stem):
        if msg is None or not msg.data:
            return None
        enc = msg.encoding.lower()
        suffix = "ppm" if enc in ("rgb8", "bgr8", "rgba8", "bgra8") else "pgm"
        path = self.capture_dir / f"{stem}.{suffix}"
        try:
            channels = 4 if enc in ("rgba8", "bgra8") else 3
            src = bytes(msg.data)
            if suffix == "ppm":
                rows = []
                for row in range(msg.height):
                    start = row * msg.step
                    raw = src[start: start + msg.width * channels]
                    pixels = bytearray()
                    for idx in range(0, len(raw), channels):
                        px = raw[idx: idx + channels]
                        if len(px) < channels:
                            continue
                        if enc in ("bgr8", "bgra8"):
                            pixels.extend((px[2], px[1], px[0]))
                        else:
                            pixels.extend((px[0], px[1], px[2]))
                    rows.append(bytes(pixels))
                header = f"P6\n{msg.width} {msg.height}\n255\n".encode("ascii")
                path.write_bytes(header + b"".join(rows))
            else:
                rows = []
                for row in range(msg.height):
                    start = row * msg.step
                    rows.append(src[start: start + msg.width])
                header = f"P5\n{msg.width} {msg.height}\n255\n".encode("ascii")
                path.write_bytes(header + b"".join(rows))
            self.get_logger().info(f"[PHOTO] Saved: {path}")
            return str(path)
        except Exception as exc:
            self.get_logger().warn(f"Failed to save image: {exc}")
            return None

    # ──────────────────────────────────────────────────────────
    # Status publishing
    # ──────────────────────────────────────────────────────────
    def _publish_status(self):
        complete = all(
            [
                self.track_complete,
                self.surface_photo is not None,
                self.underwater_photo is not None,
                self._has_required_docking_contact(),
            ]
        )
        msg = (
            f"phase={self.mission_phase} course={self.course.upper()} "
            f"track_complete={self.track_complete} "
            f"surface_photo={'yes' if self.surface_photo else 'no'} "
            f"underwater_photo={'yes' if self.underwater_photo else 'no'} "
            f"docking_touched={len(self.touched_buoys)}/3 "
            f"touched={','.join(sorted(self.touched_buoys)) or '-'} "
            f"mission_complete={complete}"
        )
        self.get_logger().info(f"[STATUS] {msg}")
        self.status_pub.publish(String(data=msg))


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisorNav2()
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
