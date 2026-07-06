#!/usr/bin/env python3

import math
import statistics
import time
from pathlib import Path

import cv2
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import Toggle
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Float64, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray, Detection3DArray

from .mission_logic import (
    bounded_thruster_trim,
    clamp,
    crossed_gate,
    is_optional_recovery_waypoint,
    optimized_lap2_route,
    optimized_capture_waypoint,
    photo_capture_ready,
)


SURFACE_WAYPOINTS = {"surface_photo_green_box", "surface_photo_search"}
UNDERWATER_WAYPOINTS = {
    "underwater_photo_blue_box",
    "underwater_photo_search",
}

COURSE_CONFIG = {
    "a": {
        "dock_id": "kki_buoy_dock_a",
        "dock_type": "kki_blue_buoys",
        "dock_names": (
            "docking_a_blue_1",
            "docking_a_blue_2",
            "docking_a_blue_3",
        ),
        "target_hints": {
            "surface_box": (-10.2, -5.4),
            "underwater_box": (-11.0, -7.7),
        },
    },
    "b": {
        "dock_id": "kki_buoy_dock_b",
        "dock_type": "kki_blue_buoys",
        "dock_names": (
            "docking_b_blue_1",
            "docking_b_blue_2",
            "docking_b_blue_3",
        ),
        "target_hints": {
            "surface_box": (10.2, -5.4),
            "underwater_box": (11.0, -7.7),
        },
    },
}

GATES = {
    "gate1": ((-13.0, 0.0), (-11.0, 0.0)),
    "gate2": ((-13.0, 4.8), (-11.0, 4.8)),
    "gate3": ((-13.0, 8.8), (-11.0, 8.8)),
    "gate4": ((-5.2, 13.0), (-5.2, 11.0)),
    "gate5": ((-1.4, 13.0), (-1.4, 11.0)),
    "gate6": ((2.4, 13.0), (2.4, 11.0)),
    "gate7": ((6.2, 13.0), (6.2, 11.0)),
    "gate8": ((13.0, 0.0), (11.0, 0.0)),
    "gate9": ((13.0, 4.8), (11.0, 4.8)),
    "gate10": ((13.0, 8.8), (11.0, 8.8)),
}


def yaw_to_quaternion(yaw):
    msg = Quaternion()
    msg.z = math.sin(yaw * 0.5)
    msg.w = math.cos(yaw * 0.5)
    return msg


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MissionSupervisorNav2V2(Node):
    """Sequential two-lap Nav2 mission with bounded HSV search tasks."""

    def __init__(self):
        super().__init__("mission_supervisor_nav2")
        self.declare_parameter("course", "a")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("initial_waypoint_index", 0)
        self.declare_parameter(
            "capture_dir",
            "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_captures",
        )
        self.declare_parameter("search_yaw_rate_radps", 0.22)
        self.declare_parameter("search_timeout_lap1_s", 110.0)
        self.declare_parameter("search_timeout_lap2_s", 32.0)
        self.declare_parameter("search_timeout_lap2_retry_s", 120.0)
        self.declare_parameter("photo_lap2_max_retries", 2)
        self.declare_parameter("search_tracking_timeout_s", 45.0)
        self.declare_parameter("search_hint_timeout_s", 35.0)
        self.declare_parameter("photo_min_range_m", 1.0)
        self.declare_parameter("photo_max_range_m", 3.5)
        self.declare_parameter("photo_target_range_m", 1.8)
        self.declare_parameter("photo_range_tolerance_m", 0.40)
        self.declare_parameter("photo_down_range_tolerance_m", 1.60)
        self.declare_parameter("photo_front_center_tolerance", 0.10)
        self.declare_parameter("photo_down_center_tolerance", 0.35)
        self.declare_parameter("photo_range_adjust_speed_mps", 0.14)
        self.declare_parameter("photo_hint_association_radius_m", 1.2)
        self.declare_parameter("photo_down_hint_association_radius_m", 2.2)
        self.declare_parameter("photo_search_start_radius_m", 4.5)
        self.declare_parameter("lap2_search_limit_rad", math.radians(30.0))
        self.declare_parameter("photo_escape_distance_m", 1.0)
        self.declare_parameter("photo_escape_speed_mps", 0.18)
        self.declare_parameter("photo_escape_yaw_rate_radps", 0.32)
        self.declare_parameter("photo_escape_timeout_s", 35.0)
        self.declare_parameter("photo_exit_corridor_y_m", -9.3)
        self.declare_parameter("photo_exit_corridor_x_min_m", -13.0)
        self.declare_parameter("photo_exit_corridor_x_max_m", 8.0)
        self.declare_parameter("bottom_corridor_y_m", -9.2)
        self.declare_parameter("bottom_corridor_x_min_m", -13.0)
        self.declare_parameter("bottom_corridor_x_max_m", 8.0)
        self.declare_parameter("bottom_corridor_pass_margin_m", 2.0)
        self.declare_parameter("dock_nudge_distance_m", 4.0)
        self.declare_parameter("dock_nudge_finish_radius_m", 0.65)
        self.declare_parameter("dock_nudge_speed_mps", 0.22)
        self.declare_parameter("dock_nudge_timeout_s", 60.0)
        self.declare_parameter("dock_action_timeout_s", 30.0)
        self.declare_parameter("dock_contact_speed_mps", 0.08)
        self.declare_parameter("dock_contact_timeout_s", 240.0)
        self.declare_parameter("dock_contact_lateral_retry_after_s", 22.0)
        self.declare_parameter("dock_contact_push_cycle_s", 24.0)
        self.declare_parameter("dock_contact_stage_offset_m", 0.95)
        self.declare_parameter("dock_contact_max_retries", 1)
        self.declare_parameter("speed_limit_max_mps", 0.68)
        self.declare_parameter("max_nav_retries", 2)
        self.declare_parameter("nav_goal_stall_timeout_s", 8.0)
        self.declare_parameter("nav_retry_backoff_s", 2.0)
        self.declare_parameter("gate_approach_accept_radius_m", 1.0)
        self.declare_parameter("gate_approach_recovery_accept_radius_m", 2.2)
        self.declare_parameter("gate_exit_accept_radius_m", 1.2)
        self.declare_parameter("gate_exit_min_cross_clearance_m", 0.20)
        self.declare_parameter("optional_accept_radius_m", 1.4)
        self.declare_parameter("optional_recovery_skip_radius_m", 3.0)
        self.declare_parameter("turn_accept_radius_m", 0.9)
        self.declare_parameter("photo_approach_accept_radius_m", 1.8)
        self.declare_parameter("gate_nudge_trigger_distance_m", 3.6)
        self.declare_parameter("gate_approach_direct_nudge_distance_m", 3.0)
        self.declare_parameter("gate_approach_nudge_distance_m", 3.2)
        self.declare_parameter("gate_approach_nudge_finish_radius_m", 0.85)
        self.declare_parameter("gate_approach_projection_margin", 0.90)
        self.declare_parameter("pre_gate_nudge_distance_m", 3.5)
        self.declare_parameter("pre_gate_nudge_finish_radius_m", 1.0)
        self.declare_parameter("turn_nudge_distance_m", 4.5)
        self.declare_parameter("turn_nudge_finish_radius_m", 1.3)
        self.declare_parameter("gate_nudge_projection_margin", 0.25)
        self.declare_parameter("gate_nudge_exit_margin_m", 0.90)
        self.declare_parameter("gate_nudge_finish_clearance_m", 0.25)
        self.declare_parameter("gate_nudge_speed_mps", 0.28)
        self.declare_parameter("lap2_gate_nudge_speed_mps", 0.34)
        self.declare_parameter("gate_nudge_timeout_s", 22.0)
        self.declare_parameter("gate_collision_recenter_distance_m", 0.75)
        self.declare_parameter("gate_collision_recenter_radius_m", 0.45)
        self.declare_parameter("gate_collision_recenter_speed_mps", 0.16)
        self.declare_parameter("lap2_min_coverage_percent", 8.0)
        self.declare_parameter("lap2_compress_gate_waypoints", True)

        requested_course = str(self.get_parameter("course").value).lower().strip()
        self.course = requested_course if requested_course in COURSE_CONFIG else "a"
        self.config = COURSE_CONFIG[self.course]
        self.capture_dir = Path(str(self.get_parameter("capture_dir").value))
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()

        self.waypoints = self._load_waypoints(
            str(self.get_parameter("waypoint_file").value)
        )
        self.base_navigation_waypoints = [
            item for item in self.waypoints if not item.get("docking", False)
        ]
        self.navigation_waypoints = [dict(item) for item in self.base_navigation_waypoints]
        self.pose = None
        self.previous_pose_xy = None
        self.scan = None
        self.latest_images = {"front": None, "down": None}
        self.image_widths = {"front": 1280, "down": 640}
        self.latest_detections = {}
        self.semantic_classes = set()
        self.coverage = 0.0
        self.phase = "WAIT_NAV2"
        self.lap = 1
        self.waypoint_index = max(
            0,
            min(
                int(self.get_parameter("initial_waypoint_index").value),
                len(self.navigation_waypoints),
            ),
        )
        self.nav_retries = 0
        self.nav_goal_handle = None
        self.dock_goal_handle = None
        self.nav_goal_token = 0
        self.nav_goal_sent_time = 0.0
        self.last_nav_activity_time = 0.0
        self.nav_retry_after = 0.0
        self.action_in_progress = False
        self.gate_nudge = None
        self.search_target = None
        self.search_camera = None
        self.search_started = 0.0
        self.search_rotation = 0.0
        self.search_direction = 1.0
        self.last_search_time = 0.0
        self.last_search_log_time = 0.0
        self.search_lock_started = 0.0
        self.search_hint_yaw = None
        self.search_hint_aligned_time = 0.0
        self.search_last_yaw = None
        self.photo_escape = None
        self.capture_poses = {}
        self.photos = {1: set(), 2: set()}
        self.photo_search_retries = {}
        self.gates_crossed = {1: set(), 2: set()}
        self.contacts = {1: set(), 2: set()}
        self.dock_success = {1: False, 2: False}
        self.dock_contact_stage = None
        self.dock_contact_stage_started = 0.0
        self.dock_contact_target_y = None
        self.dock_contact_retries = {1: 0, 2: 0}
        self.latest_collision_status = "clear"
        self.latest_collision_time = 0.0
        self.last_collision_enable_request = 0.0
        self.survey_success = {1: False, 2: False}
        self.lap2_gate_route_compressed = False
        self.straight_yaw_commands = []
        self.adaptive_trim = 0.0
        self.last_status_time = 0.0
        self.task_started = 0.0
        self.pending_service_calls = []

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.dock_client = ActionClient(self, DockRobot, "/dock_robot")
        self.undock_client = ActionClient(self, UndockRobot, "/undock_robot")
        self.bt_state_client = self.create_client(
            GetState, "/bt_navigator/get_state"
        )
        self.map_save_client = self.create_client(
            Trigger, "/asv/mapping/save"
        )
        self.landmark_save_client = self.create_client(
            Trigger, "/asv/perception/save_landmarks"
        )
        self.collision_toggle_client = self.create_client(
            Toggle, "/nav2_collision_monitor/toggle"
        )

        self.status_pub = self.create_publisher(
            String, "/asv/mission/status", 10
        )
        self.source_pub = self.create_publisher(
            String, "/asv/autonomy/source", 10
        )
        self.task_cmd_pub = self.create_publisher(Twist, "/cmd_vel_task", 10)
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit, "/speed_limit", 10
        )
        self.trim_pub = self.create_publisher(
            Float64, "/asv/control/adaptive_trim", 10
        )

        self.create_subscription(
            Odometry, "/asv/planar_odom", self._on_odom, 10
        )
        self.create_subscription(LaserScan, "/asv/lidar/scan", self._on_scan, 10)
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
        self.create_subscription(
            Detection2DArray,
            "/asv/perception/front/detections",
            lambda msg: self._on_detection("front", msg),
            10,
        )
        self.create_subscription(
            Detection2DArray,
            "/asv/perception/down/detections",
            lambda msg: self._on_detection("down", msg),
            10,
        )
        self.create_subscription(
            Detection3DArray,
            "/asv/perception/landmarks",
            self._on_landmarks,
            10,
        )
        self.create_subscription(
            Float32, "/asv/mapping/coverage", self._on_coverage, 10
        )
        self.create_subscription(
            String, "/asv/collision/status", self._on_contact, 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_nav", self._on_nav_command, 10
        )
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"Two-lap mission loaded {len(self.navigation_waypoints)} waypoints "
            f"for course {self.course.upper()}"
        )

    def _load_waypoints(self, filepath):
        path = Path(filepath)
        if not path.exists():
            raise FileNotFoundError(f"Waypoint file not found: {filepath}")
        data = yaml.safe_load(path.read_text()) or {}
        result = []
        for index, item in enumerate(data.get("waypoints", [])):
            result.append(
                {
                    "name": str(item.get("name", f"wp_{index}")),
                    "x": float(item.get("x", 0.0)),
                    "y": float(item.get("y", 0.0)),
                    "yaw": (
                        float(item["yaw"]) if "yaw" in item else None
                    ),
                    "speed": float(item.get("speed", 0.25)),
                    "docking": bool(item.get("docking", False)),
                }
            )
        return result

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        current = (float(p.x), float(p.y))
        self.previous_pose_xy = self.pose[:2] if self.pose is not None else current
        self.pose = (
            current[0],
            current[1],
            quaternion_to_yaw(msg.pose.pose.orientation),
        )
        self._update_gate_crossings(self.previous_pose_xy, current)

    def _on_scan(self, msg):
        self.scan = msg

    def _on_image(self, camera, msg):
        self.latest_images[camera] = msg
        self.image_widths[camera] = int(msg.width)

    def _on_detection(self, camera, msg):
        now = time.monotonic()
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            class_id = result.hypothesis.class_id
            self.latest_detections[class_id] = {
                "camera": camera,
                "time": now,
                "score": float(result.hypothesis.score),
                "pixel_x": float(detection.bbox.center.position.x),
            }

    def _on_landmarks(self, msg):
        self.semantic_classes = {
            detection.results[0].hypothesis.class_id
            for detection in msg.detections
            if detection.results
        }

    def _on_coverage(self, msg):
        self.coverage = float(msg.data)

    def _on_contact(self, msg):
        data = msg.data
        if data and data != "clear":
            self.latest_collision_status = data
            self.latest_collision_time = time.monotonic()
        for index, name in enumerate(self.config["dock_names"], start=1):
            if name in data or f"docking_blue_{[-10.0, -9.7, -9.4][index - 1]}" in data:
                if name not in self.contacts[self.lap]:
                    self.contacts[self.lap].add(name)
                    self.get_logger().info(
                        f"[LAP {self.lap}] Dock contact confirmed: {name} "
                        f"({len(self.contacts[self.lap])}/3)"
                    )

    def _on_nav_command(self, msg):
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.last_nav_activity_time = time.monotonic()
        if (
            self.lap == 1
            and msg.linear.x > 0.20
            and abs(msg.angular.z) < 0.45
        ):
            self.straight_yaw_commands.append(float(msg.angular.z))

    def _tick(self):
        now = time.monotonic()
        if now - self.last_status_time >= 1.0:
            self._publish_status()
            self.last_status_time = now
        if (
            self.phase != "CONTACT"
            and now - self.last_collision_enable_request >= 10.0
        ):
            self._toggle_collision_monitor(True)
            self.last_collision_enable_request = now

        if self.phase == "WAIT_NAV2":
            self._wait_for_nav2()
        elif self.phase == "NAVIGATE":
            if self._maybe_start_photo_search_near_hint(now):
                return
            if self.action_in_progress:
                if (
                    not self._maybe_start_dock_nudge(now)
                    and not self._maybe_accept_photo_exit_corridor(now)
                    and not self._maybe_accept_bottom_corridor_transit(now)
                    and not self._maybe_accept_near_gate_exit(now)
                    and not self._maybe_start_gate_nudge(now, "near gate center")
                    and not self._maybe_start_gate_approach_nudge(
                        now, "near gate approach corridor"
                    )
                    and not self._maybe_accept_near_optional_waypoint(now)
                    and not self._maybe_accept_near_gate_approach(now)
                    and not self._maybe_skip_passed_optional_waypoint(now)
                ):
                    self._check_navigation_watchdog(now)
            elif self._maybe_skip_passed_optional_waypoint(now):
                return
            elif self._maybe_accept_photo_exit_corridor(now):
                return
            elif self._maybe_accept_bottom_corridor_transit(now):
                return
            elif self._maybe_accept_near_optional_waypoint(now):
                return
            elif self._maybe_accept_near_gate_approach(now):
                return
            elif self._maybe_accept_near_gate_exit(now):
                return
            elif self._maybe_start_gate_nudge(now, "near gate center"):
                return
            elif self._maybe_start_dock_nudge(now):
                return
            elif now >= self.nav_retry_after:
                self._send_next_waypoint()
        elif self.phase == "SEARCH":
            self._run_search(now)
        elif self.phase == "PHOTO_ESCAPE":
            self._run_photo_escape(now)
        elif self.phase == "GATE_NUDGE":
            self._run_gate_nudge(now)
        elif self.phase == "DOCKING":
            if len(self.contacts[self.lap]) >= 3:
                self._complete_docking_from_contacts()
            elif now - self.task_started >= float(
                self.get_parameter("dock_action_timeout_s").value
            ):
                if self.dock_goal_handle is not None:
                    try:
                        self.dock_goal_handle.cancel_goal_async()
                    except Exception as exc:
                        self.get_logger().warn(
                            f"Failed to cancel dock action at handoff: {exc}"
                        )
                    self.dock_goal_handle = None
                self._start_contact_push("dock approach handoff timeout")
        elif self.phase == "CONTACT":
            self._run_contact(now)
        elif self.phase == "SAVE_MAP":
            self._wait_for_mapping_save(now)
        elif self.phase == "UNDOCK_FALLBACK":
            self._run_undock_fallback(now)
        elif self.phase == "OPTIMIZE":
            if now - self.task_started >= 0.5:
                self._start_second_lap()
        elif self.phase == "DONE":
            self.task_cmd_pub.publish(Twist())

    def _wait_for_nav2(self):
        if not self.bt_state_client.service_is_ready():
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            return
        request = GetState.Request()
        future = self.bt_state_client.call_async(request)
        future.add_done_callback(self._nav_state_response)
        self.phase = "WAIT_NAV2_RESPONSE"

    def _nav_state_response(self, future):
        try:
            state = future.result().current_state.label
        except Exception as exc:
            self.get_logger().warn(f"Nav2 state check failed: {exc}")
            self.phase = "WAIT_NAV2"
            return
        if state != "active":
            self.phase = "WAIT_NAV2"
            return
        self.phase = "NAVIGATE"
        self.source_pub.publish(String(data="nav"))

    def _send_next_waypoint(self):
        if self.waypoint_index >= len(self.navigation_waypoints):
            self.survey_success[self.lap] = True
            self._start_docking()
            return
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if (
            self.waypoint_index == 0
            and self.pose is not None
            and math.hypot(waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1])
            < 0.8
        ):
            self.waypoint_index += 1
            return

        self._publish_speed(waypoint["speed"])
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_for_waypoint(self.waypoint_index)
        self.action_in_progress = True
        self.nav_goal_token += 1
        token = self.nav_goal_token
        now = time.monotonic()
        self.nav_goal_sent_time = now
        self.last_nav_activity_time = now
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, token=token: self._nav_feedback(
                feedback, token
            ),
        )
        future.add_done_callback(
            lambda result, token=token: self._nav_goal_response(result, token)
        )
        self.get_logger().info(
            f"[LAP {self.lap}] Navigate {waypoint['name']} "
            f"({waypoint['x']:.2f}, {waypoint['y']:.2f})"
        )

    def _pose_for_waypoint(self, index):
        waypoint = self.navigation_waypoints[index]
        if waypoint.get("yaw") is not None:
            yaw = waypoint["yaw"]
        elif index + 1 < len(self.navigation_waypoints):
            following = self.navigation_waypoints[index + 1]
            yaw = math.atan2(
                following["y"] - waypoint["y"],
                following["x"] - waypoint["x"],
            )
        elif self.pose is not None:
            yaw = self.pose[2]
        else:
            yaw = 0.0
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def _nav_feedback(self, _feedback, token):
        if token != self.nav_goal_token:
            return
        # Feedback may continue while the controller is publishing zero velocity.
        # The stall watchdog is based on real navigation commands instead.

    def _nav_goal_response(self, future, token):
        if token != self.nav_goal_token:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self._navigation_failed(f"send failed: {exc}")
            return
        if not handle.accepted:
            self._navigation_failed("goal rejected")
            return
        self.nav_goal_handle = handle
        self.last_nav_activity_time = time.monotonic()
        result = handle.get_result_async()
        result.add_done_callback(
            lambda future, token=token: self._nav_result(future, token)
        )

    def _nav_result(self, future, token):
        if token != self.nav_goal_token:
            return
        self.action_in_progress = False
        self.nav_goal_handle = None
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            self._navigation_failed(f"result failed: {exc}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._navigation_failed(f"status={status}")
            return
        self.nav_retries = 0
        waypoint = self.navigation_waypoints[self.waypoint_index]
        self.waypoint_index += 1
        if waypoint["name"].endswith("_photo_approach"):
            self._begin_photo_target_nudge(
                time.monotonic(), "photo approach reached"
            )
            return
        if waypoint["name"] in SURFACE_WAYPOINTS:
            self._start_search("surface_box", "front")
        elif waypoint["name"] in UNDERWATER_WAYPOINTS:
            self._start_search("underwater_box", "down")
        else:
            self._start_search_for_next_photo()

    def _navigation_failed(self, reason):
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retry_after = time.monotonic() + float(
            self.get_parameter("nav_retry_backoff_s").value
        )
        self.nav_retries += 1
        if self.nav_retries <= int(self.get_parameter("max_nav_retries").value):
            self.get_logger().warn(f"Navigation retry {self.nav_retries}: {reason}")
            return
        now = time.monotonic()
        if self._maybe_accept_near_gate_approach(now, reason):
            return
        if self._maybe_start_gate_approach_nudge(now, reason):
            return
        if self._maybe_accept_near_gate_exit(now, reason):
            return
        if self._maybe_start_gate_nudge(now, reason):
            return
        if self._maybe_start_photo_search_after_nav_failure(now, reason):
            return
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if self._is_required_navigation_waypoint(waypoint["name"]):
            self.get_logger().error(
                f"Required waypoint {waypoint['name']} failed: {reason}; "
                "holding mission and retrying"
            )
            self.nav_retries = 0
            return
        if self._maybe_accept_near_optional_waypoint(now):
            return
        if self._maybe_skip_passed_optional_waypoint(now):
            return
        if self._maybe_skip_optional_near_following(now, reason):
            return
        if self._maybe_skip_optional_recovery_waypoint(now, reason):
            return
        if self._maybe_start_turn_waypoint_nudge(now, reason):
            return
        if self._maybe_start_pre_gate_nudge(now, reason):
            return
        self.get_logger().error(
            f"Optional waypoint {waypoint['name']} failed but is not safe to skip: "
            f"{reason}; holding mission and retrying"
        )
        self.nav_retries = 0

    def _maybe_start_photo_search_after_nav_failure(self, now, reason):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        target_name = name
        target_index = self.waypoint_index
        if name.endswith("_photo_approach"):
            if self.waypoint_index >= len(self.navigation_waypoints) - 1:
                return False
            following = self.navigation_waypoints[self.waypoint_index + 1]
            if following["name"] not in SURFACE_WAYPOINTS | UNDERWATER_WAYPOINTS:
                return False
            target_name = following["name"]
            target_index = self.waypoint_index + 1
        elif name not in SURFACE_WAYPOINTS | UNDERWATER_WAYPOINTS:
            return False

        target_waypoint = self.navigation_waypoints[target_index]
        target_distance = math.hypot(
            target_waypoint["x"] - self.pose[0],
            target_waypoint["y"] - self.pose[1],
        )
        approach_distance = math.hypot(
            waypoint["x"] - self.pose[0],
            waypoint["y"] - self.pose[1],
        )
        distance = target_distance if name.endswith("_photo_approach") else min(
            target_distance, approach_distance
        )
        radius = float(self.get_parameter("photo_search_start_radius_m").value)
        if target_name in UNDERWATER_WAYPOINTS:
            radius = min(radius, 3.0)
        if distance > radius:
            return False

        self.get_logger().warn(
            f"Starting HSV search from current photo area after {reason}; "
            f"{target_name} is {target_distance:.2f}m away, "
            f"{name} is {approach_distance:.2f}m away"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index = target_index + 1
        if target_name in SURFACE_WAYPOINTS:
            self._start_search("surface_box", "front")
        else:
            self._start_search("underwater_box", "down")
        return True

    def _maybe_start_photo_search_near_hint(self, now):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints) - 1:
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if not waypoint["name"].endswith("_photo_approach"):
            return False
        following = self.navigation_waypoints[self.waypoint_index + 1]
        if following["name"] in SURFACE_WAYPOINTS:
            target = "surface_box"
            camera = "front"
        elif following["name"] in UNDERWATER_WAYPOINTS:
            target = "underwater_box"
            camera = "down"
        else:
            return False
        hint = self.config["target_hints"][target]
        distance = math.hypot(hint[0] - self.pose[0], hint[1] - self.pose[1])
        radius = float(self.get_parameter("photo_search_start_radius_m").value)
        if target == "underwater_box":
            radius = min(radius, 3.0)
        if distance > radius:
            return False
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to cancel photo approach near target hint: {exc}"
                )
        self.get_logger().info(
            f"[LAP {self.lap}] Starting {target} HSV search {distance:.2f}m "
            "from mapped target hint"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 2
        self._start_search(target, camera)
        return True

    def _check_navigation_watchdog(self, now):
        timeout = float(self.get_parameter("nav_goal_stall_timeout_s").value)
        if timeout <= 0.0 or now - self.last_nav_activity_time <= timeout:
            return
        waypoint = self.navigation_waypoints[self.waypoint_index]
        reason = (
            f"stalled for {now - self.last_nav_activity_time:.1f}s without "
            f"Nav2 feedback/cmd_vel at {waypoint['name']}"
        )
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel stalled goal: {exc}")
        self._navigation_failed(reason)

    def _maybe_skip_passed_optional_waypoint(self, now):
        if (
            self.pose is None
            or self.waypoint_index >= len(self.navigation_waypoints) - 1
        ):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if not self._can_skip_passed_waypoint(waypoint["name"]):
            return False
        following = self.navigation_waypoints[self.waypoint_index + 1]
        current_distance = math.hypot(
            waypoint["x"] - self.pose[0],
            waypoint["y"] - self.pose[1],
        )
        following_distance = math.hypot(
            following["x"] - self.pose[0],
            following["y"] - self.pose[1],
        )
        if following_distance > 1.6 or current_distance <= following_distance + 0.5:
            return False
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel passed waypoint: {exc}")
        self.get_logger().warn(
            f"Skipping passed optional waypoint {waypoint['name']} "
            f"(distance={current_distance:.2f}m, next={following['name']} "
            f"{following_distance:.2f}m)"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        if waypoint["name"].endswith("_photo_approach"):
            self._begin_photo_target_nudge(now, "passed photo approach")
            return True
        self._start_search_for_next_photo()
        return True

    def _maybe_accept_near_optional_waypoint(self, now):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if self._is_required_navigation_waypoint(waypoint["name"]):
            return False
        distance = math.hypot(waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1])
        radius = self._optional_accept_radius(waypoint["name"])
        if distance > radius:
            return False
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel near optional waypoint: {exc}")
        self.get_logger().warn(
            f"Accepting near optional waypoint {waypoint['name']} at {distance:.2f}m"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        if waypoint["name"].endswith("_photo_approach"):
            self._begin_photo_target_nudge(now, "accepted photo approach")
            return True
        self._start_search_for_next_photo()
        return True

    def _optional_accept_radius(self, name):
        if name.endswith("_photo_approach"):
            return float(self.get_parameter("photo_approach_accept_radius_m").value)
        if any(token in name for token in ("turn", "corner", "entry")):
            return float(self.get_parameter("turn_accept_radius_m").value)
        return float(self.get_parameter("optional_accept_radius_m").value)

    def _maybe_accept_photo_exit_corridor(self, now):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if waypoint["name"] != "photo_exit_south":
            return False
        if not {"surface_box", "underwater_box"}.issubset(self.photos[self.lap]):
            return False
        x_min = float(self.get_parameter("photo_exit_corridor_x_min_m").value)
        x_max = float(self.get_parameter("photo_exit_corridor_x_max_m").value)
        if self.course == "b":
            x_min, x_max = -x_max, -x_min
        y_limit = float(self.get_parameter("photo_exit_corridor_y_m").value)
        if not (x_min <= self.pose[0] <= x_max and self.pose[1] <= y_limit):
            return False
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to cancel photo exit corridor waypoint: {exc}"
                )
        self.get_logger().warn(
            f"Accepting photo_exit_south from safe south corridor at "
            f"pose=({self.pose[0]:.2f}, {self.pose[1]:.2f})"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        return True

    def _maybe_accept_bottom_corridor_transit(self, now):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if waypoint["name"] != "path_13":
            return False
        if not {"surface_box", "underwater_box"}.issubset(self.photos[self.lap]):
            return False

        x_min = float(self.get_parameter("bottom_corridor_x_min_m").value)
        x_max = float(self.get_parameter("bottom_corridor_x_max_m").value)
        if self.course == "b":
            x_min, x_max = -x_max, -x_min
        y_limit = float(self.get_parameter("bottom_corridor_y_m").value)
        if not (x_min <= self.pose[0] <= x_max and self.pose[1] <= y_limit):
            return False

        direction = 1.0 if self.course == "a" else -1.0
        pass_margin = float(
            self.get_parameter("bottom_corridor_pass_margin_m").value
        )
        if (self.pose[0] - float(waypoint["x"])) * direction < pass_margin:
            return False

        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to cancel bottom corridor transit waypoint: {exc}"
                )
        self.get_logger().warn(
            f"Skipping {waypoint['name']} because the boat is already in the "
            f"bottom transit corridor at pose=({self.pose[0]:.2f}, {self.pose[1]:.2f})"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        return True

    def _maybe_accept_near_gate_exit(self, now, reason=""):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        if not (name.startswith("gate") and name.endswith("_exit")):
            return False
        gate_name = name.split("_", 1)[0]
        if gate_name not in self.gates_crossed[self.lap]:
            return False
        state = self._gate_line_state(gate_name)
        min_clearance = float(
            self.get_parameter("gate_exit_min_cross_clearance_m").value
        )
        if state is not None and abs(state["signed_distance"]) < min_clearance:
            return False
        distance = math.hypot(waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1])
        radius = float(self.get_parameter("gate_exit_accept_radius_m").value)
        if distance > radius:
            return False
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel near gate exit: {exc}")
        detail = f" after {reason}" if reason else ""
        self.get_logger().warn(
            f"Accepting near gate exit {name} at {distance:.2f}m{detail}; "
            f"{gate_name} already crossed"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        return True

    def _maybe_start_gate_nudge(self, now, reason=""):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        lap2_exit_target = (
            self.lap == 2
            and name.startswith("gate")
            and name.endswith("_exit")
        )
        if not (
            name.startswith("gate")
            and (name.endswith("_center") or lap2_exit_target)
        ):
            return False
        gate_name = name.split("_", 1)[0]
        if gate_name in self.gates_crossed[self.lap]:
            return False
        state = self._gate_line_state(gate_name)
        if state is None:
            return False
        trigger_distance = float(
            self.get_parameter("gate_nudge_trigger_distance_m").value
        )
        projection_margin = float(
            self.get_parameter("gate_nudge_projection_margin").value
        )
        if abs(state["signed_distance"]) > trigger_distance:
            return False
        if not (
            -projection_margin
            <= state["projection"]
            <= 1.0 + projection_margin
        ):
            return False
        self._begin_gate_nudge(now, gate_name, state, reason)
        return True

    def _begin_gate_nudge(self, now, gate_name, state, reason=""):
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel gate nudge goal: {exc}")
        detail = f" after {reason}" if reason else ""
        self.get_logger().warn(
            f"Starting gate nudge for {gate_name} at "
            f"{abs(state['signed_distance']):.2f}m from line{detail}"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.gate_nudge = {
            "gate": gate_name,
            "mode": "cross",
            "started": now,
            "direction": 1.0 if state["signed_distance"] >= 0.0 else -1.0,
        }
        self.phase = "GATE_NUDGE"
        self.source_pub.publish(String(data="task"))
        self._publish_speed(self._gate_nudge_speed())

    def _maybe_start_gate_approach_nudge(self, now, reason=""):
        if (
            self.pose is None
            or self.waypoint_index >= len(self.navigation_waypoints) - 1
        ):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        if not (name.startswith("gate") and name.endswith("_approach")):
            return False
        gate_name = name.split("_", 1)[0]
        if gate_name in self.gates_crossed[self.lap]:
            return False
        following = self.navigation_waypoints[self.waypoint_index + 1]
        if following["name"] != f"{gate_name}_center":
            return False
        state = self._gate_line_state(gate_name)
        projection_margin = float(
            self.get_parameter("gate_approach_projection_margin").value
        )
        if state is None or not (
            -projection_margin <= state["projection"] <= 1.0 + projection_margin
        ):
            return False
        distance = math.hypot(waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1])
        max_distance = float(
            self.get_parameter("gate_approach_nudge_distance_m").value
        )
        if distance > max_distance:
            return False
        self._begin_gate_approach_nudge(now, gate_name, waypoint, distance, reason)
        return True

    def _begin_gate_approach_nudge(self, now, gate_name, waypoint, distance, reason=""):
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to cancel gate approach nudge goal: {exc}"
                )
        detail = f" after {reason}" if reason else ""
        self.get_logger().warn(
            f"Starting gate approach nudge for {gate_name}: "
            f"{waypoint['name']} is {distance:.2f}m away{detail}"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.gate_nudge = {
            "gate": gate_name,
            "mode": "approach",
            "started": now,
            "target_x": float(waypoint["x"]),
            "target_y": float(waypoint["y"]),
            "target_label": waypoint["name"],
            "finish_radius": float(
                self.get_parameter("gate_approach_nudge_finish_radius_m").value
            ),
        }
        self.phase = "GATE_NUDGE"
        self.source_pub.publish(String(data="task"))
        self._publish_speed(self._gate_nudge_speed())

    def _maybe_start_pre_gate_nudge(self, now, reason=""):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if not (
            waypoint["name"].startswith("pre_gate")
            or waypoint["name"].startswith("post_gate")
        ):
            return False
        distance = math.hypot(waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1])
        max_distance = float(self.get_parameter("pre_gate_nudge_distance_m").value)
        if distance > max_distance:
            return False
        self._begin_waypoint_nudge(
            now,
            waypoint,
            distance,
            float(self.get_parameter("pre_gate_nudge_finish_radius_m").value),
            reason,
        )
        return True

    def _maybe_start_turn_waypoint_nudge(self, now, reason=""):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        if self._is_required_navigation_waypoint(name):
            return False
        if not any(token in name for token in ("turn", "corner", "entry")):
            return False
        distance = math.hypot(waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1])
        max_distance = float(self.get_parameter("turn_nudge_distance_m").value)
        if distance > max_distance:
            return False
        self._begin_waypoint_nudge(
            now,
            waypoint,
            distance,
            float(self.get_parameter("turn_nudge_finish_radius_m").value),
            reason,
        )
        return True

    def _maybe_start_dock_nudge(self, now, reason=""):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if waypoint["name"] not in {
            "dock_align_south",
            "dock_entry",
            "dock_side_align",
        }:
            return False
        distance = math.hypot(
            waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1]
        )
        if distance > float(self.get_parameter("dock_nudge_distance_m").value):
            return False
        self._begin_waypoint_nudge(
            now,
            waypoint,
            distance,
            float(self.get_parameter("dock_nudge_finish_radius_m").value),
            reason or "near docking staging pose",
            speed=float(self.get_parameter("dock_nudge_speed_mps").value),
            timeout=float(self.get_parameter("dock_nudge_timeout_s").value),
        )
        return True

    def _begin_waypoint_nudge(
        self,
        now,
        waypoint,
        distance,
        finish_radius,
        reason="",
        speed=None,
        timeout=None,
        search_after=None,
        advance_index=True,
    ):
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel waypoint nudge goal: {exc}")
        detail = f" after {reason}" if reason else ""
        self.get_logger().warn(
            f"Starting waypoint nudge for {waypoint['name']}: "
            f"target is {distance:.2f}m away{detail}"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.gate_nudge = {
            "gate": waypoint["name"],
            "mode": "waypoint",
            "started": now,
            "target_x": float(waypoint["x"]),
            "target_y": float(waypoint["y"]),
            "target_label": waypoint["name"],
            "finish_radius": float(finish_radius),
        }
        if speed is not None:
            self.gate_nudge["speed"] = float(speed)
        if timeout is not None:
            self.gate_nudge["timeout"] = float(timeout)
        if search_after is not None:
            self.gate_nudge["search_after"] = tuple(search_after)
        self.gate_nudge["advance_index"] = bool(advance_index)
        self.phase = "GATE_NUDGE"
        self.source_pub.publish(String(data="task"))
        self._publish_speed(
            float(speed) if speed is not None else self._gate_nudge_speed()
        )

    def _begin_photo_target_nudge(self, now, reason=""):
        if self.pose is None or self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        if name in SURFACE_WAYPOINTS:
            search_after = ("surface_box", "front")
            speed = 0.18
            finish_radius = 0.70
        elif name in UNDERWATER_WAYPOINTS:
            search_after = ("underwater_box", "down")
            speed = 0.16
            finish_radius = 0.50
        else:
            return False
        distance = math.hypot(
            waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1]
        )
        self._begin_waypoint_nudge(
            now,
            waypoint,
            distance,
            finish_radius=finish_radius,
            reason=reason,
            speed=speed,
            timeout=60.0,
            search_after=search_after,
        )
        return True

    def _gate_line_state(self, gate_name):
        if self.pose is None or gate_name not in GATES:
            return None
        green, red = GATES[gate_name]
        gate_x = red[0] - green[0]
        gate_y = red[1] - green[1]
        length_sq = gate_x * gate_x + gate_y * gate_y
        if length_sq <= 1e-9:
            return None
        length = math.sqrt(length_sq)
        normal_x = -gate_y / length
        normal_y = gate_x / length
        signed_distance = (
            (self.pose[0] - green[0]) * normal_x
            + (self.pose[1] - green[1]) * normal_y
        )
        projection = (
            (self.pose[0] - green[0]) * gate_x
            + (self.pose[1] - green[1]) * gate_y
        ) / length_sq
        clamped_projection = clamp(projection, 0.0, 1.0)
        foot_x = green[0] + clamped_projection * gate_x
        foot_y = green[1] + clamped_projection * gate_y
        return {
            "signed_distance": signed_distance,
            "projection": projection,
            "normal_x": normal_x,
            "normal_y": normal_y,
            "foot_x": foot_x,
            "foot_y": foot_y,
            "center_x": 0.5 * (green[0] + red[0]),
            "center_y": 0.5 * (green[1] + red[1]),
        }

    def _run_gate_nudge(self, now):
        if self.gate_nudge is None or self.pose is None:
            self._finish_gate_nudge(success=False)
            return
        if self.gate_nudge.get("mode") in ("approach", "waypoint"):
            self._run_target_nudge(now)
            return
        gate_name = self.gate_nudge["gate"]
        collision_recent = (
            now - self.latest_collision_time <= 0.8
            and "gate" in self.latest_collision_status
        )
        if collision_recent:
            green, red = GATES[gate_name]
            nearest = min(
                (green, red),
                key=lambda point: math.hypot(
                    point[0] - self.pose[0], point[1] - self.pose[1]
                ),
            )
            self.gate_nudge["collision_recovery_until"] = now + 4.0
            self.gate_nudge["collision_recovery_heading"] = math.atan2(
                nearest[1] - self.pose[1], nearest[0] - self.pose[0]
            )
            if not self.gate_nudge.get("collision_recenter", False):
                self.gate_nudge["collision_recovery_extra_s"] = (
                    float(self.gate_nudge.get("collision_recovery_extra_s", 0.0))
                    + 15.0
                )
            self.gate_nudge["collision_recenter"] = True
        recovery_until = float(
            self.gate_nudge.get("collision_recovery_until", 0.0)
        )
        if now < recovery_until:
            recovery_heading = float(
                self.gate_nudge["collision_recovery_heading"]
            )
            yaw_error = normalize_angle(recovery_heading - self.pose[2])
            command = Twist()
            command.angular.z = clamp(0.8 * yaw_error, -0.28, 0.28)
            if abs(yaw_error) < 0.9:
                command.linear.x = -0.12
            self.source_pub.publish(String(data="task"))
            self.task_cmd_pub.publish(command)
            return
        if self.gate_nudge.get("collision_recenter", False):
            state = self._gate_line_state(gate_name)
            if state is None:
                self._finish_gate_nudge(success=False)
                return
            direction = float(self.gate_nudge.get("direction", 1.0))
            recenter_distance = float(
                self.get_parameter("gate_collision_recenter_distance_m").value
            )
            target_x = (
                state["center_x"]
                + direction * state["normal_x"] * recenter_distance
            )
            target_y = (
                state["center_y"]
                + direction * state["normal_y"] * recenter_distance
            )
            distance = math.hypot(
                target_x - self.pose[0], target_y - self.pose[1]
            )
            recenter_radius = float(
                self.get_parameter("gate_collision_recenter_radius_m").value
            )
            if distance <= recenter_radius:
                self.gate_nudge["collision_recenter"] = False
                self.gate_nudge["started"] = now
                self.gate_nudge["collision_recovery_extra_s"] = 0.0
                self.get_logger().info(
                    f"Recovered from {gate_name} contact at the gate centerline"
                )
            else:
                desired_yaw = math.atan2(
                    target_y - self.pose[1], target_x - self.pose[0]
                )
                yaw_error = normalize_angle(desired_yaw - self.pose[2])
                command = Twist()
                command.angular.z = clamp(0.8 * yaw_error, -0.30, 0.30)
                if abs(yaw_error) < 0.9:
                    speed = float(
                        self.get_parameter(
                            "gate_collision_recenter_speed_mps"
                        ).value
                    )
                    command.linear.x = speed * max(
                        0.45, 1.0 - abs(yaw_error) / 0.9
                    )
                self.source_pub.publish(String(data="task"))
                self.task_cmd_pub.publish(command)
                return
        if gate_name in self.gates_crossed[self.lap]:
            state = self._gate_line_state(gate_name)
            if state is None:
                self._finish_gate_nudge(success=True)
                return
            direction = float(self.gate_nudge.get("direction", 1.0))
            finish_clearance = float(
                self.get_parameter("gate_nudge_finish_clearance_m").value
            )
            if state["signed_distance"] * direction <= -finish_clearance:
                self.get_logger().info(
                    f"[LAP {self.lap}] Gate nudge crossed {gate_name} "
                    f"with {abs(state['signed_distance']):.2f}m clearance"
                )
                self._finish_gate_nudge(success=True)
                return
        timeout = float(
            self.gate_nudge.get(
                "timeout", self.get_parameter("gate_nudge_timeout_s").value
            )
        ) + float(self.gate_nudge.get("collision_recovery_extra_s", 0.0))
        if now - self.gate_nudge["started"] >= timeout:
            if gate_name in self.gates_crossed[self.lap]:
                self.get_logger().warn(
                    f"Gate nudge timeout for {gate_name} after crossing; "
                    "continuing to gate exit"
                )
                self._finish_gate_nudge(success=True)
            else:
                self.get_logger().warn(
                    f"Gate nudge timeout for {gate_name}; returning to Nav2"
                )
                self._finish_gate_nudge(success=False)
            return
        state = self._gate_line_state(gate_name)
        if state is None:
            self._finish_gate_nudge(success=False)
            return
        direction = float(self.gate_nudge.get("direction", 1.0))
        exit_margin = float(self.get_parameter("gate_nudge_exit_margin_m").value)
        target_x = state["center_x"] - direction * state["normal_x"] * exit_margin
        target_y = state["center_y"] - direction * state["normal_y"] * exit_margin
        desired_yaw = math.atan2(target_y - self.pose[1], target_x - self.pose[0])
        yaw_error = normalize_angle(desired_yaw - self.pose[2])
        speed = self._gate_nudge_speed()
        command = Twist()
        command.angular.z = clamp(0.70 * yaw_error, -0.30, 0.30)
        if abs(yaw_error) < 0.9:
            command.linear.x = speed * max(0.45, 1.0 - abs(yaw_error) / 0.9)
        self.source_pub.publish(String(data="task"))
        self.task_cmd_pub.publish(command)

    def _run_target_nudge(self, now):
        if self.gate_nudge is None or self.pose is None:
            self._finish_gate_nudge(success=False)
            return
        target_label = self.gate_nudge.get("target_label", self.gate_nudge["gate"])
        target_x = float(self.gate_nudge["target_x"])
        target_y = float(self.gate_nudge["target_y"])
        distance = math.hypot(target_x - self.pose[0], target_y - self.pose[1])
        finish_radius = float(self.gate_nudge.get("finish_radius", 0.85))
        if distance <= finish_radius:
            self.get_logger().warn(
                f"Waypoint nudge reached {target_label} at {distance:.2f}m"
            )
            self._finish_gate_nudge(success=True)
            return
        timeout = float(
            self.gate_nudge.get(
                "timeout", self.get_parameter("gate_nudge_timeout_s").value
            )
        )
        if now - self.gate_nudge["started"] >= timeout:
            self.get_logger().warn(
                f"Waypoint nudge timeout for {target_label}; returning to Nav2"
            )
            self._finish_gate_nudge(success=False)
            return
        desired_yaw = math.atan2(target_y - self.pose[1], target_x - self.pose[0])
        yaw_error = normalize_angle(desired_yaw - self.pose[2])
        command = Twist()
        command.angular.z = clamp(0.70 * yaw_error, -0.30, 0.30)
        if abs(yaw_error) < 0.95:
            speed = float(
                self.gate_nudge.get("speed", self._gate_nudge_speed())
            )
            command.linear.x = speed * max(0.40, 1.0 - abs(yaw_error) / 0.95)
        self.source_pub.publish(String(data="task"))
        self.task_cmd_pub.publish(command)

    def _gate_nudge_speed(self):
        parameter = (
            "lap2_gate_nudge_speed_mps"
            if self.lap == 2
            else "gate_nudge_speed_mps"
        )
        return float(self.get_parameter(parameter).value)

    def _finish_gate_nudge(self, success):
        search_after = None
        advance_index = True
        if self.gate_nudge is not None:
            search_after = self.gate_nudge.get("search_after")
            advance_index = bool(self.gate_nudge.get("advance_index", True))
        self.task_cmd_pub.publish(Twist())
        self.source_pub.publish(String(data="nav"))
        if success and advance_index:
            self.waypoint_index += 1
        self.nav_retries = 0
        self.nav_retry_after = time.monotonic() + 0.1
        self.gate_nudge = None
        if search_after is not None:
            self._start_search(search_after[0], search_after[1])
            return
        self.phase = "NAVIGATE"

    def _maybe_accept_near_gate_approach(self, now, reason=""):
        if (
            self.pose is None
            or self.waypoint_index >= len(self.navigation_waypoints) - 1
        ):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        if not (name.startswith("gate") and name.endswith("_approach")):
            return False
        gate_name = name.split("_", 1)[0]
        following = self.navigation_waypoints[self.waypoint_index + 1]
        if following["name"] != f"{gate_name}_center":
            return False
        distance = math.hypot(waypoint["x"] - self.pose[0], waypoint["y"] - self.pose[1])
        radius = float(self.get_parameter("gate_approach_accept_radius_m").value)
        if reason:
            state = self._gate_line_state(gate_name)
            projection_margin = float(
                self.get_parameter("gate_nudge_projection_margin").value
            )
            if state is None or not (
                -projection_margin
                <= state["projection"]
                <= 1.0 + projection_margin
            ):
                return False
            radius = max(
                radius,
                float(
                    self.get_parameter(
                        "gate_approach_recovery_accept_radius_m"
                    ).value
                ),
            )
        if distance > radius:
            return False
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel near gate approach: {exc}")
        detail = f" after {reason}" if reason else ""
        self.get_logger().warn(
            f"Accepting near gate approach {name} at {distance:.2f}m{detail}; "
            f"continuing to {following['name']}"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        direct_nudge_distance = float(
            self.get_parameter("gate_approach_direct_nudge_distance_m").value
        )
        projection_margin = float(
            self.get_parameter("gate_nudge_projection_margin").value
        )
        state = self._gate_line_state(gate_name)
        if (
            state is not None
            and gate_name not in self.gates_crossed[self.lap]
            and abs(state["signed_distance"]) <= direct_nudge_distance
            and -projection_margin <= state["projection"] <= 1.0 + projection_margin
        ):
            self._begin_gate_nudge(now, gate_name, state, "accepted gate approach")
        return True

    def _maybe_skip_optional_near_following(self, now, reason):
        if (
            self.pose is None
            or self.waypoint_index >= len(self.navigation_waypoints) - 1
        ):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if self._is_required_navigation_waypoint(waypoint["name"]):
            return False
        following = self.navigation_waypoints[self.waypoint_index + 1]
        following_distance = math.hypot(
            following["x"] - self.pose[0],
            following["y"] - self.pose[1],
        )
        if following_distance > 2.0:
            return False
        current_distance = math.hypot(
            waypoint["x"] - self.pose[0],
            waypoint["y"] - self.pose[1],
        )
        self.get_logger().warn(
            f"Skipping optional waypoint {waypoint['name']} after {reason}; "
            f"next {following['name']} is {following_distance:.2f}m away "
            f"(current distance={current_distance:.2f}m)"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        return True

    def _maybe_skip_optional_recovery_waypoint(self, now, reason):
        if (
            self.pose is None
            or self.waypoint_index >= len(self.navigation_waypoints) - 1
        ):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        name = waypoint["name"]
        if self._is_required_navigation_waypoint(name):
            return False
        if not is_optional_recovery_waypoint(name):
            return False
        current_distance = math.hypot(
            waypoint["x"] - self.pose[0],
            waypoint["y"] - self.pose[1],
        )
        radius = float(self.get_parameter("optional_recovery_skip_radius_m").value)
        if current_distance > radius:
            return False
        following = self.navigation_waypoints[self.waypoint_index + 1]
        handle = self.nav_goal_handle
        self.nav_goal_token += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to cancel optional recovery waypoint: {exc}"
                )
        self.get_logger().warn(
            f"Skipping optional recovery waypoint {name} after {reason}; "
            f"distance={current_distance:.2f}m, next={following['name']}"
        )
        self.action_in_progress = False
        self.nav_goal_handle = None
        self.nav_retries = 0
        self.nav_retry_after = now + 0.1
        self.waypoint_index += 1
        return True

    def _can_skip_passed_waypoint(self, name):
        if not self._is_required_navigation_waypoint(name):
            return True
        if name.startswith("gate"):
            gate_name = name.split("_", 1)[0]
            return gate_name in self.gates_crossed[self.lap]
        return False

    def _is_required_navigation_waypoint(self, name):
        if name in SURFACE_WAYPOINTS or name in UNDERWATER_WAYPOINTS:
            return True
        if name.startswith("dock_") or name == "dock_contact_blue_buoys":
            return True
        return name.startswith("gate") and any(
            suffix in name for suffix in ("_approach", "_center", "_exit")
        )

    def _start_search(self, target, camera):
        self.phase = "SEARCH"
        self.search_target = target
        self.search_camera = camera
        self.search_started = time.monotonic()
        self.last_search_time = self.search_started
        self.last_search_log_time = 0.0
        self.search_lock_started = 0.0
        self.search_hint_aligned_time = 0.0
        self.search_last_yaw = self.pose[2] if self.pose is not None else None
        self.search_hint_yaw = self._photo_search_hint_yaw(target, camera)
        self.search_rotation = 0.0
        self.search_direction = 1.0
        self.source_pub.publish(String(data="task"))
        self._publish_speed(0.10)

    def _photo_search_hint_yaw(self, target, camera):
        if target in self.capture_poses:
            return float(self.capture_poses[target][2])
        if self.pose is None:
            return None
        hint = self.config["target_hints"].get(target)
        if hint is None:
            return None
        target_bearing = math.atan2(hint[1] - self.pose[1], hint[0] - self.pose[0])
        camera_offset = math.pi * 0.5 if camera == "down" else 0.0
        return normalize_angle(target_bearing - camera_offset)

    def _start_search_for_next_photo(self):
        if self.waypoint_index >= len(self.navigation_waypoints):
            return False
        waypoint = self.navigation_waypoints[self.waypoint_index]
        if waypoint["name"] in SURFACE_WAYPOINTS:
            self.waypoint_index += 1
            self._start_search("surface_box", "front")
            return True
        if waypoint["name"] in UNDERWATER_WAYPOINTS:
            self.waypoint_index += 1
            self._start_search("underwater_box", "down")
            return True
        return False

    def _run_search(self, now):
        known = self.search_target in self.capture_poses
        retry_count = self.photo_search_retries.get(
            (self.lap, self.search_target), 0
        )
        if self.lap == 2:
            timeout_parameter = "search_timeout_lap2_s"
            if not known or retry_count > 0:
                timeout_parameter = "search_timeout_lap2_retry_s"
        else:
            timeout_parameter = "search_timeout_lap1_s"
        timeout = float(self.get_parameter(timeout_parameter).value)
        elapsed = now - self.search_started
        tracking_timeout = float(
            self.get_parameter("search_tracking_timeout_s").value
        )
        search_timed_out = (
            self.search_lock_started <= 0.0 and elapsed >= timeout
        )
        tracking_timed_out = (
            self.search_lock_started > 0.0
            and now - self.search_lock_started >= tracking_timeout
        )
        if search_timed_out or tracking_timed_out:
            self.get_logger().warn(
                f"[LAP {self.lap}] HSV search timeout: {self.search_target}"
            )
            self._finish_search(captured=False)
            return

        detection = self.latest_detections.get(self.search_target)
        fresh = detection is not None and now - detection["time"] <= 0.7
        camera_detection = (
            fresh and detection.get("camera") == self.search_camera
        )
        range_m = (
            self._target_range(self.search_camera, detection)
            if camera_detection
            else None
        )
        correct_camera = (
            camera_detection
            and range_m is not None
            and self._detection_matches_target_hint(
                self.search_camera, detection, range_m
            )
        )
        if correct_camera and self.search_lock_started <= 0.0:
            self.search_lock_started = now
            self.get_logger().info(
                f"[LAP {self.lap}] Locked {self.search_target}; "
                f"tracking window={tracking_timeout:.1f}s"
            )
        semantic_confirmed = self.search_target in self.semantic_classes
        if correct_camera:
            width = max(1, self.image_widths[self.search_camera])
            normalized_error = (
                width * 0.5 - detection["pixel_x"]
            ) / (width * 0.5)
            target_range = float(
                self.get_parameter("photo_target_range_m").value
            )
            range_tolerance = float(
                self.get_parameter(
                    "photo_down_range_tolerance_m"
                    if self.search_camera == "down"
                    else "photo_range_tolerance_m"
                ).value
            )
            range_error = (
                range_m - target_range if range_m is not None else None
            )
            center_tolerance = float(
                self.get_parameter(
                    "photo_down_center_tolerance"
                    if self.search_camera == "down"
                    else "photo_front_center_tolerance"
                ).value
            )
            if photo_capture_ready(
                detection_camera=detection["camera"],
                expected_camera=self.search_camera,
                fresh=fresh,
                range_m=range_m,
                normalized_error=normalized_error,
                target_range=target_range,
                range_tolerance=range_tolerance,
                center_tolerance=center_tolerance,
            ):
                if self._save_photo():
                    self.photos[self.lap].add(self.search_target)
                    if self.pose is not None:
                        self.capture_poses[self.search_target] = self.pose
                    self.get_logger().info(
                        f"[LAP {self.lap}] Captured {self.search_target} "
                        f"range={range_m:.2f}m score={detection['score']:.2f}"
                    )
                    self._finish_search(captured=True)
                    return
                self.task_cmd_pub.publish(Twist())
                return
            command = Twist()
            yaw_gain = 0.20 if self.search_camera == "down" else 0.45
            yaw_limit = 0.12 if self.search_camera == "down" else 0.22
            command.angular.z = clamp(
                yaw_gain * normalized_error, -yaw_limit, yaw_limit
            )
            if (
                range_error is not None
                and self.search_camera == "front"
                and abs(normalized_error) <= 0.20
            ):
                max_adjust_speed = float(
                    self.get_parameter("photo_range_adjust_speed_mps").value
                )
                command.linear.x = clamp(
                    0.18 * range_error,
                    -max_adjust_speed,
                    max_adjust_speed,
                )
            self.task_cmd_pub.publish(command)
            if now - self.last_search_log_time >= 2.0:
                range_text = "none" if range_m is None else f"{range_m:.2f}m"
                self.get_logger().info(
                    f"[LAP {self.lap}] Tracking {self.search_target}: "
                    f"camera={detection['camera']} score={detection['score']:.2f} "
                    f"error={normalized_error:.2f} lidar={range_text} "
                    f"landmark={semantic_confirmed}"
                )
                self.last_search_log_time = now
            return

        hint_timeout = float(
            self.get_parameter("search_hint_timeout_s").value
        )
        if (
            self.search_hint_yaw is not None
            and retry_count <= 0
            and elapsed < hint_timeout
        ):
            if self.pose is None:
                self.task_cmd_pub.publish(Twist())
                return
            hint_error = normalize_angle(self.search_hint_yaw - self.pose[2])
            if abs(hint_error) > 0.10:
                command = Twist()
                command.angular.z = clamp(0.75 * hint_error, -0.32, 0.32)
                self.task_cmd_pub.publish(command)
                return
            if self.search_hint_aligned_time <= 0.0:
                self.search_hint_aligned_time = now
                self.get_logger().info(
                    f"[LAP {self.lap}] Camera aligned to mapped search hint "
                    f"for {self.search_target}; waiting for stable HSV"
                )
            self.task_cmd_pub.publish(Twist())
            return

        dt = max(0.0, min(0.2, now - self.last_search_time))
        self.last_search_time = now
        yaw_rate = float(self.get_parameter("search_yaw_rate_radps").value)
        if self.lap == 2 and known and retry_count <= 0:
            limit = abs(
                float(self.get_parameter("lap2_search_limit_rad").value)
            )
            self.search_rotation += self.search_direction * abs(yaw_rate) * dt
            if self.search_rotation >= limit:
                self.search_rotation = limit
                self.search_direction = -1.0
            elif self.search_rotation <= -limit:
                self.search_rotation = -limit
                self.search_direction = 1.0
            command_yaw = self.search_direction * abs(yaw_rate)
        else:
            if self.pose is not None and self.search_last_yaw is not None:
                self.search_rotation += abs(
                    normalize_angle(self.pose[2] - self.search_last_yaw)
                )
            self.search_last_yaw = self.pose[2] if self.pose is not None else None
            if self.search_rotation >= 2.0 * math.pi:
                self._finish_search(captured=False)
                return
            command_yaw = abs(yaw_rate)
        command = Twist()
        command.angular.z = command_yaw
        self.task_cmd_pub.publish(command)

    def _target_range(self, camera, detection):
        if self.scan is None or detection is None:
            return None
        bearing = self._detection_bearing(camera, detection)
        if bearing is None:
            return None
        center = int(
            round((bearing - self.scan.angle_min) / self.scan.angle_increment)
        )
        window = max(
            2, int(round(math.radians(5.0) / abs(self.scan.angle_increment)))
        )
        values = []
        for index in range(center - window, center + window + 1):
            value = float(self.scan.ranges[index % len(self.scan.ranges)])
            if (
                math.isfinite(value)
                and max(0.45, float(self.scan.range_min)) <= value
                <= float(self.get_parameter("photo_max_range_m").value)
            ):
                values.append(value)
        return statistics.median(values) if values else None

    def _detection_bearing(self, camera, detection):
        if detection is None:
            return None
        width = max(1, self.image_widths[camera])
        normalized = (width * 0.5 - detection["pixel_x"]) / width
        horizontal_fov = 1.85 if camera == "down" else 1.3962634
        bearing = normalized * horizontal_fov
        if camera == "down":
            bearing += math.pi * 0.5
        return normalize_angle(bearing)

    def _detection_matches_target_hint(self, camera, detection, range_m):
        hint = self.config["target_hints"].get(self.search_target)
        if hint is None:
            return True
        if self.pose is None or range_m is None:
            return False
        bearing = self._detection_bearing(camera, detection)
        if bearing is None:
            return False
        world_bearing = self.pose[2] + bearing
        detected_x = self.pose[0] + range_m * math.cos(world_bearing)
        detected_y = self.pose[1] + range_m * math.sin(world_bearing)
        association_error = math.hypot(
            detected_x - hint[0], detected_y - hint[1]
        )
        association_radius = float(
            self.get_parameter(
                "photo_down_hint_association_radius_m"
                if camera == "down"
                else "photo_hint_association_radius_m"
            ).value
        )
        return association_error <= association_radius

    def _save_photo(self):
        msg = self.latest_images[self.search_camera]
        if msg is None or not msg.data:
            return False
        try:
            import numpy as np
            enc = msg.encoding.lower()
            channels = 4 if enc in ("rgba8", "bgra8") else 3
            img_arr = np.frombuffer(msg.data, dtype=np.uint8)
            
            if msg.step == msg.width * channels:
                img_3d = img_arr.reshape((msg.height, msg.width, channels))
            else:
                img_3d = img_arr.reshape((msg.height, msg.step))[:, :msg.width * channels].reshape((msg.height, msg.width, channels))
            
            if enc in ("bgr8", "bgra8"):
                bgr_image = img_3d[:, :, :3]
            elif enc in ("rgb8", "rgba8"):
                bgr_image = img_3d[:, :, [2, 1, 0]]
            else:
                bgr_image = img_3d

            # Save standard file requested by the state machine
            path = self.capture_dir / f"{self.search_target}_lap{self.lap}_{self.course}.png"
            cv2.imwrite(str(path), bgr_image)

            # Determine the file names for the competition check (PPM format)
            if self.search_target == "surface_box":
                ppm_name = "surface_green_box_front.ppm"
                png_name = "surface_green_box_front.png"
            elif self.search_target == "underwater_box":
                ppm_name = "underwater_blue_box_down.ppm"
                png_name = "underwater_blue_box_down.png"
            else:
                ppm_name = f"{self.search_target}_lap{self.lap}_{self.course}.ppm"
                png_name = f"{self.search_target}_lap{self.lap}_{self.course}.png"

            path_ppm = self.capture_dir / ppm_name
            path_png = self.capture_dir / png_name

            # Write PNG using opencv
            cv2.imwrite(str(path_png), bgr_image)

            # Write PPM manually (bypassing cv2/cv_bridge)
            rgb_arr = bgr_image[:, :, [2, 1, 0]]
            payload = rgb_arr.tobytes()
            header = f"P6\n{msg.width} {msg.height}\n255\n".encode("ascii")
            path_ppm.write_bytes(header + payload)

            self.get_logger().info(f"[PHOTO] Saved files: {path}, {path_png}, {path_ppm}")
            return True
        except Exception as exc:
            self.get_logger().warn(f"Photo save failed: {exc}")
            return False

    def _finish_search(self, captured=False):
        self.task_cmd_pub.publish(Twist())
        if captured and self.search_target is not None:
            self.photo_search_retries[(self.lap, self.search_target)] = 0
        elif self._maybe_retry_photo_search_after_failure():
            return
        self._start_photo_escape(captured)

    def _maybe_retry_photo_search_after_failure(self):
        if self.lap != 2 or self.search_target is None or self.search_camera is None:
            return False
        key = (self.lap, self.search_target)
        retry_count = self.photo_search_retries.get(key, 0)
        max_retries = int(self.get_parameter("photo_lap2_max_retries").value)
        if retry_count >= max_retries:
            return False

        self.photo_search_retries[key] = retry_count + 1
        target = self.search_target
        camera = self.search_camera
        capture_pose = self.capture_poses.get(target)
        if capture_pose is not None and self.pose is not None:
            retry_waypoint = {
                "name": f"{target}_capture_retry",
                "x": float(capture_pose[0]),
                "y": float(capture_pose[1]),
            }
            distance = math.hypot(
                retry_waypoint["x"] - self.pose[0],
                retry_waypoint["y"] - self.pose[1],
            )
            self.get_logger().warn(
                f"[LAP 2] Retrying {target} from mapped capture pose "
                f"({retry_count + 1}/{max_retries}); distance={distance:.2f}m"
            )
            self._begin_waypoint_nudge(
                time.monotonic(),
                retry_waypoint,
                distance,
                finish_radius=0.45 if camera == "down" else 0.65,
                reason="HSV search timeout",
                speed=0.15 if camera == "down" else 0.18,
                timeout=60.0,
                search_after=(target, camera),
                advance_index=False,
            )
            return True

        self.get_logger().warn(
            f"[LAP 2] Retrying {target} with full sweep "
            f"({retry_count + 1}/{max_retries})"
        )
        self._start_search(target, camera)
        return True

    def _start_photo_escape(self, captured):
        now = time.monotonic()
        bearing = 0.0
        nearest = None
        if self.scan is not None and self.scan.ranges:
            minimum = max(0.45, float(self.scan.range_min))
            maximum = min(3.5, float(self.scan.range_max))
            for index, raw_value in enumerate(self.scan.ranges):
                value = float(raw_value)
                if not math.isfinite(value) or not minimum <= value <= maximum:
                    continue
                if nearest is None or value < nearest:
                    nearest = value
                    bearing = self.scan.angle_min + index * self.scan.angle_increment
        current_yaw = self.pose[2] if self.pose is not None else 0.0
        self.photo_escape = {
            "started": now,
            "start_xy": self.pose[:2] if self.pose is not None else None,
            "target_yaw": normalize_angle(current_yaw + bearing + math.pi),
            "nearest": nearest,
            "mode": (
                "reverse"
                if captured and self.search_camera == "front"
                else "turn_and_forward"
            ),
        }
        self.phase = "PHOTO_ESCAPE"
        self.source_pub.publish(String(data="task"))
        self._publish_speed(
            float(self.get_parameter("photo_escape_speed_mps").value)
        )
        nearest_text = "unknown" if nearest is None else f"{nearest:.2f}m"
        self.get_logger().info(
            f"LiDAR photo escape started: mode={self.photo_escape['mode']} "
            f"nearest={nearest_text}, "
            f"turning away by {math.degrees(normalize_angle(bearing + math.pi)):.1f}deg"
        )

    def _run_photo_escape(self, now):
        if self.photo_escape is None:
            self._finish_photo_escape()
            return
        timeout = float(self.get_parameter("photo_escape_timeout_s").value)
        if now - self.photo_escape["started"] >= timeout:
            self.get_logger().warn("LiDAR photo escape timeout; returning control to Nav2")
            self._finish_photo_escape()
            return
        if self.pose is None:
            self.task_cmd_pub.publish(Twist())
            return
        start_xy = self.photo_escape["start_xy"]
        distance = (
            math.hypot(self.pose[0] - start_xy[0], self.pose[1] - start_xy[1])
            if start_xy is not None
            else 0.0
        )
        target_distance = float(
            self.get_parameter("photo_escape_distance_m").value
        )
        if distance >= target_distance:
            self.get_logger().info(
                f"LiDAR photo escape complete after {distance:.2f}m"
            )
            self._finish_photo_escape()
            return
        if self.photo_escape["mode"] == "reverse":
            command = Twist()
            command.linear.x = -float(
                self.get_parameter("photo_escape_speed_mps").value
            )
            self.task_cmd_pub.publish(command)
            return
        yaw_error = normalize_angle(
            self.photo_escape["target_yaw"] - self.pose[2]
        )
        command = Twist()
        if abs(yaw_error) > 0.20:
            max_yaw = float(
                self.get_parameter("photo_escape_yaw_rate_radps").value
            )
            command.angular.z = clamp(0.65 * yaw_error, -max_yaw, max_yaw)
        else:
            command.linear.x = float(
                self.get_parameter("photo_escape_speed_mps").value
            )
            command.angular.z = clamp(0.35 * yaw_error, -0.10, 0.10)
        self.task_cmd_pub.publish(command)

    def _finish_photo_escape(self):
        self.task_cmd_pub.publish(Twist())
        self.source_pub.publish(String(data="nav"))
        self.photo_escape = None
        self.nav_retries = 0
        self.nav_retry_after = time.monotonic() + 0.2
        self.phase = "NAVIGATE"

    def _start_docking(self):
        self.phase = "DOCKING"
        self.action_in_progress = True
        self.task_started = time.monotonic()
        self.source_pub.publish(String(data="dock"))
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self._start_contact_push("docking server unavailable")
            return
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self.config["dock_id"]
        goal.dock_type = self.config["dock_type"]
        # The supervisor already completed the staging sequence. Enabling this
        # would make OpenNav request /cmd_vel_nav while the mux selects "dock".
        goal.navigate_to_staging_pose = False
        future = self.dock_client.send_goal_async(goal)
        future.add_done_callback(self._dock_goal_response)

    def _dock_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._start_contact_push(f"dock send failed: {exc}")
            return
        if not handle.accepted:
            self._start_contact_push("dock rejected")
            return
        self.dock_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self._dock_result)

    def _dock_result(self, future):
        self.dock_goal_handle = None
        if self.phase != "DOCKING":
            return
        try:
            result = future.result().result
            reason = "dock action complete" if result.success else result.error_msg
        except Exception as exc:
            reason = f"dock result failed: {exc}"
        self._start_contact_push(reason)

    def _start_contact_push(self, reason):
        dock_x = 12.35 if self.course == "a" else -12.35
        dock_y = -9.70
        close_to_dock = (
            self.pose is not None
            and math.hypot(self.pose[0] - dock_x, self.pose[1] - dock_y) <= 1.5
        )
        if not close_to_dock:
            self.get_logger().error(
                f"Refusing final contact push outside dock staging area: {reason}"
            )
            self.action_in_progress = False
            if self.lap == 1:
                self._begin_mapping_save()
            else:
                self.phase = "DONE"
            return
        self.get_logger().info(f"Final docking contact: {reason}")
        self.phase = "CONTACT"
        self.action_in_progress = False
        self.task_started = time.monotonic()
        self.dock_contact_stage = "stage"
        self.dock_contact_stage_started = self.task_started
        self.dock_contact_target_y = None
        self.source_pub.publish(String(data="task"))
        self._toggle_collision_monitor(False)

    def _run_contact(self, now):
        if len(self.contacts[self.lap]) >= 3:
            self._complete_docking_from_contacts()
            return
        if now - self.task_started > float(
            self.get_parameter("dock_contact_timeout_s").value
        ):
            retries = self.dock_contact_retries[self.lap]
            max_retries = int(self.get_parameter("dock_contact_max_retries").value)
            if retries < max_retries:
                self.dock_contact_retries[self.lap] = retries + 1
                self.task_started = now
                self.dock_contact_stage = "stage"
                self.dock_contact_stage_started = now
                self.dock_contact_target_y = None
                self.get_logger().warn(
                    f"Docking contact timeout at {len(self.contacts[self.lap])}/3; "
                    f"retrying contact sweep {retries + 1}/{max_retries}"
                )
                return
            self.task_cmd_pub.publish(Twist())
            self.get_logger().error(
                f"Docking contact timeout: {len(self.contacts[self.lap])}/3"
            )
            self._toggle_collision_monitor(True)
            if self.lap == 1:
                self._begin_mapping_save()
            else:
                self.phase = "DONE"
            return

        if self.pose is None:
            self.task_cmd_pub.publish(Twist())
            return

        direction = 1.0 if self.course == "a" else -1.0
        dock_x = direction * 12.35
        dock_y = self._dock_contact_target_y(now)
        dock_yaw = 0.0 if self.course == "a" else math.pi
        stage_x = dock_x - direction * float(
            self.get_parameter("dock_contact_stage_offset_m").value
        )
        stage_dx = stage_x - self.pose[0]
        stage_dy = dock_y - self.pose[1]
        distance_to_stage = math.hypot(stage_dx, stage_dy)
        yaw_error = normalize_angle(dock_yaw - self.pose[2])
        command = Twist()

        push_cycle_s = float(self.get_parameter("dock_contact_push_cycle_s").value)
        should_restage_for_missing = (
            self.dock_contact_stage == "push"
            and 2 <= len(self.contacts[self.lap]) < 3
            and abs(stage_dy) > 0.08
        )
        should_cycle_restage = (
            self.dock_contact_stage == "push"
            and len(self.contacts[self.lap]) < 3
            and now - self.dock_contact_stage_started > push_cycle_s
        )
        if should_restage_for_missing or should_cycle_restage:
            self.dock_contact_stage = "stage"
            self.dock_contact_stage_started = now
            self.get_logger().warn(
                f"[LAP {self.lap}] Re-staging dock contact for remaining buoy "
                f"contacts ({len(self.contacts[self.lap])}/3)"
            )

        if self.dock_contact_stage == "stage":
            too_close = direction * (self.pose[0] - stage_x) > 0.08
            if too_close:
                if abs(yaw_error) > 0.10:
                    command.angular.z = clamp(0.8 * yaw_error, -0.25, 0.25)
                else:
                    command.linear.x = -0.10
            elif abs(stage_dx) > 0.10 or abs(stage_dy) > 0.05:
                desired_yaw = math.atan2(
                    dock_y - self.pose[1], stage_x - self.pose[0]
                )
                approach_error = normalize_angle(desired_yaw - self.pose[2])
                command.angular.z = clamp(
                    0.8 * approach_error, -0.25, 0.25
                )
                if abs(approach_error) < 0.65:
                    command.linear.x = clamp(
                        0.5 * distance_to_stage, 0.04, 0.10
                    )
            elif abs(yaw_error) > 0.05:
                command.angular.z = clamp(0.8 * yaw_error, -0.22, 0.22)
            else:
                self.dock_contact_stage = "push"
                self.dock_contact_stage_started = now
                self.get_logger().info(
                    f"[LAP {self.lap}] Dock contact pose aligned; "
                    f"pose=({self.pose[0]:.2f}, {self.pose[1]:.2f}, "
                    f"{math.degrees(self.pose[2]):.1f}deg); "
                    "starting 0.08m/s final push"
                )

        if self.dock_contact_stage == "push":
            command.linear.x = float(
                self.get_parameter("dock_contact_speed_mps").value
            )
            dock_dx = dock_x - self.pose[0]
            lateral_error = dock_y - self.pose[1]
            lateral_yaw = math.atan2(lateral_error, max(0.25, abs(dock_dx)))
            command.angular.z = clamp(
                0.30 * yaw_error + 0.45 * lateral_yaw,
                -0.10,
                0.10,
            )
        self.task_cmd_pub.publish(command)

    def _dock_contact_target_y(self, now):
        positions = dict(zip(self.config["dock_names"], (-10.0, -9.7, -9.4)))
        missing = [
            name
            for name in self.config["dock_names"]
            if name not in self.contacts[self.lap]
        ]
        retry_after = float(
            self.get_parameter("dock_contact_lateral_retry_after_s").value
        )
        elapsed = now - self.task_started
        if not missing or (
            len(self.contacts[self.lap]) < 2 and elapsed < retry_after
        ):
            target_y = -9.7
        else:
            cycle = max(1, int(elapsed // max(1.0, retry_after)))
            target_y = positions[missing[cycle % len(missing)]]
        if self.dock_contact_target_y != target_y:
            self.dock_contact_target_y = target_y
            self.get_logger().info(
                f"[LAP {self.lap}] Dock contact target y={target_y:.2f}; "
                f"contacts={len(self.contacts[self.lap])}/3"
            )
        return target_y

    def _complete_docking_from_contacts(self):
        if self.dock_goal_handle is not None:
            try:
                self.dock_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel completed dock goal: {exc}")
            self.dock_goal_handle = None
        self.task_cmd_pub.publish(Twist())
        self.dock_contact_stage = None
        self.dock_success[self.lap] = True
        self._toggle_collision_monitor(True)
        self.get_logger().info(
            f"[LAP {self.lap}] Docking confirmed by all three buoy contacts"
        )
        if self.lap == 1:
            self._begin_mapping_save()
        else:
            self.phase = "DONE"

    def _begin_mapping_save(self):
        self.phase = "SAVE_MAP"
        self.task_started = time.monotonic()
        self.pending_service_calls = []
        for client in (self.map_save_client, self.landmark_save_client):
            if client.service_is_ready():
                self.pending_service_calls.append(
                    client.call_async(Trigger.Request())
                )
        mean_yaw = (
            statistics.mean(self.straight_yaw_commands)
            if self.straight_yaw_commands
            else 0.0
        )
        self.adaptive_trim = bounded_thruster_trim(mean_yaw)
        self.trim_pub.publish(Float64(data=self.adaptive_trim))

    def _wait_for_mapping_save(self, now):
        complete = all(call.done() for call in self.pending_service_calls)
        if not complete and now - self.task_started < 3.0:
            return
        for call in self.pending_service_calls:
            if not call.done():
                self.get_logger().warn("Map save service timed out")
                continue
            try:
                result = call.result()
                if not result.success:
                    self.get_logger().warn(result.message)
            except Exception as exc:
                self.get_logger().warn(f"Map save failed: {exc}")
        self.pending_service_calls = []
        self._start_undocking()

    def _start_undocking(self):
        self.phase = "UNDOCKING"
        self.source_pub.publish(String(data="dock"))
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self._start_undock_fallback()
            return
        goal = UndockRobot.Goal()
        goal.dock_type = self.config["dock_type"]
        goal.max_undocking_time = 30.0
        future = self.undock_client.send_goal_async(goal)
        future.add_done_callback(self._undock_goal_response)

    def _undock_goal_response(self, future):
        try:
            handle = future.result()
        except Exception:
            self._start_undock_fallback()
            return
        if not handle.accepted:
            self._start_undock_fallback()
            return
        result = handle.get_result_async()
        result.add_done_callback(self._undock_result)

    def _undock_result(self, future):
        try:
            success = bool(future.result().result.success)
        except Exception:
            success = False
        if success:
            self._start_optimization()
        else:
            self._start_undock_fallback()

    def _start_undock_fallback(self):
        self.phase = "UNDOCK_FALLBACK"
        self.task_started = time.monotonic()
        self.source_pub.publish(String(data="task"))

    def _run_undock_fallback(self, now):
        if now - self.task_started >= 6.0:
            self.task_cmd_pub.publish(Twist())
            self._start_optimization()
            return
        command = Twist()
        command.linear.x = -0.10
        self.task_cmd_pub.publish(command)

    def _start_optimization(self):
        self.phase = "OPTIMIZE"
        self.task_started = time.monotonic()
        optimized_waypoints = [dict(item) for item in self.base_navigation_waypoints]
        for target, pose in self.capture_poses.items():
            names = (
                SURFACE_WAYPOINTS
                if target == "surface_box"
                else UNDERWATER_WAYPOINTS
            )
            for index, waypoint in enumerate(optimized_waypoints):
                if waypoint["name"] in names:
                    optimized_waypoints[index] = optimized_capture_waypoint(
                        pose, waypoint
                    )
        optimized_waypoints, compressed = optimized_lap2_route(
            optimized_waypoints,
            coverage_percent=self.coverage,
            min_coverage_percent=float(
                self.get_parameter("lap2_min_coverage_percent").value
            ),
            compress_gate_waypoints=bool(
                self.get_parameter("lap2_compress_gate_waypoints").value
            ),
        )
        self.navigation_waypoints = optimized_waypoints
        self.lap2_gate_route_compressed = compressed
        self.trim_pub.publish(Float64(data=self.adaptive_trim))
        self.get_logger().info(
            f"Optimized lap 2 from survey map coverage={self.coverage:.1f}% "
            f"and {len(self.capture_poses)} capture poses; "
            f"gate_route_compressed={compressed} "
            f"waypoints={len(self.navigation_waypoints)}/"
            f"{len(self.base_navigation_waypoints)} "
            f"thruster trim={self.adaptive_trim:.3f}"
        )

    def _start_second_lap(self):
        self.lap = 2
        self.waypoint_index = 0
        self.nav_retries = 0
        self.contacts[2].clear()
        self.source_pub.publish(String(data="nav"))
        self.phase = "NAVIGATE"
        self.get_logger().info(
            "Starting optimized lap 2"
        )

    def _toggle_collision_monitor(self, enabled):
        if not self.collision_toggle_client.service_is_ready():
            return
        request = Toggle.Request()
        request.enable = bool(enabled)
        self.pending_service_calls.append(
            self.collision_toggle_client.call_async(request)
        )

    def _publish_speed(self, speed):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.percentage = False
        msg.speed_limit = clamp(
            float(speed),
            0.05,
            float(self.get_parameter("speed_limit_max_mps").value),
        )
        self.speed_limit_pub.publish(msg)

    def _update_gate_crossings(self, previous, current):
        if self.phase not in ("NAVIGATE", "WAIT_NAV2_RESPONSE", "GATE_NUDGE"):
            return
        for name, (green, red) in GATES.items():
            if name in self.gates_crossed[self.lap]:
                continue
            if crossed_gate(previous, current, green, red):
                self.gates_crossed[self.lap].add(name)
                self.get_logger().info(f"[LAP {self.lap}] Crossed {name}")

    def _publish_status(self):
        complete = (
            self.phase == "DONE"
            and all(self.survey_success.values())
            and all(self.dock_success.values())
            and all(
                target in self.photos[lap]
                for lap in (1, 2)
                for target in ("surface_box", "underwater_box")
            )
        )
        waypoint_name = "-"
        if 0 <= self.waypoint_index < len(self.navigation_waypoints):
            waypoint_name = self.navigation_waypoints[self.waypoint_index]["name"]
        data = (
            f"phase={self.phase} lap={self.lap}/2 course={self.course.upper()} "
            f"waypoint={waypoint_name} gates={len(self.gates_crossed[self.lap])}/10 "
            f"photos={len(self.photos[self.lap])}/2 "
            f"docking_touched={len(self.contacts[self.lap])}/3 "
            f"coverage={self.coverage:.1f}% trim={self.adaptive_trim:.3f} "
            f"lap2_compressed={self.lap2_gate_route_compressed} "
            f"mission_complete={complete}"
        )
        self.status_pub.publish(String(data=data))


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisorNav2V2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.task_cmd_pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
