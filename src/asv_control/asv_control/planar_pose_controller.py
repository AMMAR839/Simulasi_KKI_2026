#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
import tf2_ros


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PlanarPoseController(Node):
    """Drive the ASV from propeller thrust and servo vectoring on the water plane."""

    def __init__(self):
        super().__init__("planar_pose_controller")
        self.declare_parameter("world_name", "kki_2026_lintasan_a")
        self.declare_parameter("model_name", "asv_kki_2026")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "left_thrust_topic",
            "/model/asv_kki_2026/joint/left_propeller_joint/cmd_thrust",
        )
        self.declare_parameter(
            "right_thrust_topic",
            "/model/asv_kki_2026/joint/right_propeller_joint/cmd_thrust",
        )
        self.declare_parameter(
            "left_steering_topic", "/asv/thrusters/left_steering/cmd_pos"
        )
        self.declare_parameter(
            "right_steering_topic", "/asv/thrusters/right_steering/cmd_pos"
        )
        self.declare_parameter("pose_rate_hz", 10.0)
        self.declare_parameter("command_timeout_s", 0.7)
        self.declare_parameter("spawn_x", 10.8)
        self.declare_parameter("spawn_y", -8.7)
        self.declare_parameter("surface_z", 0.02)
        self.declare_parameter("minimum_visual_z", 0.01)
        self.declare_parameter("spawn_yaw", 1.2405)
        self.declare_parameter("odom_topic", "/asv/planar_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("mass_kg", 7.7)
        self.declare_parameter("yaw_inertia_kgm2", 0.56)
        # Physical response from propeller thrust. Raise slowly if acceleration
        # is too weak, but keep it below the point where waypoint overshoot grows.
        self.declare_parameter("effective_thrust_scale", 0.09)
        self.declare_parameter("thruster_x_m", -0.150)
        self.declare_parameter("thruster_y_m", 0.264)
        self.declare_parameter("linear_drag_n_per_mps", 6.5)
        self.declare_parameter("lateral_drag_n_per_mps", 12.0)
        self.declare_parameter("yaw_drag_nm_per_radps", 1.7)
        self.declare_parameter("max_linear_speed_mps", 0.45)
        self.declare_parameter("max_lateral_speed_mps", 0.12)
        self.declare_parameter("max_yaw_rate_radps", 0.65)
        self.declare_parameter("wave_effect_enabled", True)
        self.declare_parameter("wave_amplitude_m", 0.018)
        self.declare_parameter("wave_period_s", 3.5)
        self.declare_parameter("wave_roll_deg", 2.2)
        self.declare_parameter("wave_pitch_deg", 1.6)
        self.declare_parameter("service_timeout_ms", 1000)
        self.declare_parameter("startup_grace_s", 8.0)
        self.declare_parameter("kinematic_collision_enabled", True)
        self.declare_parameter("hull_collision_half_length_m", 0.28)
        self.declare_parameter("hull_collision_half_width_m", 0.18)
        self.declare_parameter("collision_margin_m", 0.01)
        self.declare_parameter("collision_rebound_factor", 0.12)

        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.service = f"/world/{self.world_name}/set_pose/blocking"
        self.gz_node = GzNode()
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        # TF broadcaster for odom → base_link (bypasses EKF startup delays)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = float(self.get_parameter("spawn_x").value)
        self.y = float(self.get_parameter("spawn_y").value)
        self.z = float(self.get_parameter("surface_z").value)
        self.minimum_visual_z = float(self.get_parameter("minimum_visual_z").value)
        self.yaw = normalize_angle(float(self.get_parameter("spawn_yaw").value))
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0
        self.mass = max(0.1, float(self.get_parameter("mass_kg").value))
        self.yaw_inertia = max(0.01, float(self.get_parameter("yaw_inertia_kgm2").value))
        self.effective_thrust_scale = max(
            0.0, float(self.get_parameter("effective_thrust_scale").value)
        )
        self.thruster_x = float(self.get_parameter("thruster_x_m").value)
        self.thruster_y = abs(float(self.get_parameter("thruster_y_m").value))
        self.linear_drag = max(0.0, float(self.get_parameter("linear_drag_n_per_mps").value))
        self.lateral_drag = max(
            0.0, float(self.get_parameter("lateral_drag_n_per_mps").value)
        )
        self.yaw_drag = max(0.0, float(self.get_parameter("yaw_drag_nm_per_radps").value))
        self.max_linear = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_lateral = float(self.get_parameter("max_lateral_speed_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_radps").value)
        self.wave_enabled = bool(self.get_parameter("wave_effect_enabled").value)
        self.wave_amplitude = max(0.0, float(self.get_parameter("wave_amplitude_m").value))
        self.wave_period = max(0.5, float(self.get_parameter("wave_period_s").value))
        self.wave_roll = math.radians(float(self.get_parameter("wave_roll_deg").value))
        self.wave_pitch = math.radians(float(self.get_parameter("wave_pitch_deg").value))
        self.timeout = float(self.get_parameter("command_timeout_s").value)
        self.service_timeout = int(self.get_parameter("service_timeout_ms").value)
        self.startup_grace = float(self.get_parameter("startup_grace_s").value)
        self.kinematic_collision_enabled = bool(
            self.get_parameter("kinematic_collision_enabled").value
        )
        self.hull_half_length = max(
            0.10, float(self.get_parameter("hull_collision_half_length_m").value)
        )
        self.hull_half_width = max(
            0.08, float(self.get_parameter("hull_collision_half_width_m").value)
        )
        self.collision_margin = max(0.0, float(self.get_parameter("collision_margin_m").value))
        self.collision_rebound = max(
            0.0, min(0.6, float(self.get_parameter("collision_rebound_factor").value))
        )
        self.collision_objects = self.build_course_collision_objects()

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.left_steer = 0.0
        self.right_steer = 0.0
        self.last_propulsion_time = 0.0
        self.last_update = time.monotonic()
        self.last_update_ros_ns = self.get_clock().now().nanoseconds
        self.startup_time = self.last_update
        self.last_warn = 0.0
        self.last_status = 0.0
        self.last_pose_result = False
        self.last_pose_response = False
        self.last_collision_time = 0.0
        self.last_collision_name = "clear"

        self.status_pub = self.create_publisher(
            String, "/asv/control/planar_pose_status", 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.collision_pub = self.create_publisher(
            Bool, "/asv/collision/kinematic_detected", 10
        )
        self.collision_status_pub = self.create_publisher(
            String, "/asv/collision/kinematic_status", 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self.on_cmd, 10
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("left_thrust_topic").value),
            self.on_left_thrust,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("right_thrust_topic").value),
            self.on_right_thrust,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("left_steering_topic").value),
            self.on_left_steer,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("right_steering_topic").value),
            self.on_right_steer,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/set_map_pose",
            self.on_set_map_pose,
            10,
        )

        rate = float(self.get_parameter("pose_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

    def next_update_time_and_dt(self):
        now = time.monotonic()
        ros_now_ns = self.get_clock().now().nanoseconds
        dt = now - self.last_update
        if (
            ros_now_ns > 0
            and self.last_update_ros_ns > 0
            and ros_now_ns >= self.last_update_ros_ns
        ):
            ros_dt = (ros_now_ns - self.last_update_ros_ns) * 1e-9
            if ros_dt > 0.0:
                dt = ros_dt
        self.last_update = now
        self.last_update_ros_ns = ros_now_ns
        return now, clamp(dt, 0.0, 0.10)

    def on_cmd(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_left_thrust(self, msg):
        self.left_thrust = float(msg.data)
        self.last_propulsion_time = time.monotonic()

    def on_right_thrust(self, msg):
        self.right_thrust = float(msg.data)
        self.last_propulsion_time = time.monotonic()

    def on_left_steer(self, msg):
        self.left_steer = float(msg.data)
        self.last_propulsion_time = time.monotonic()

    def on_right_steer(self, msg):
        self.right_steer = float(msg.data)
        self.last_propulsion_time = time.monotonic()

    def on_set_map_pose(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        # Convert orientation quaternion to yaw
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0
        self.get_logger().info(f"[TELEPORT] Internal pose reset to x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}deg")

    def on_timer(self):
        now, dt = self.next_update_time_and_dt()

        previous_x = self.x
        previous_y = self.y
        previous_yaw = self.yaw
        self.integrate_propulsion(now, dt)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        candidate_x = self.x + (self.u * cos_yaw - self.v * sin_yaw) * dt
        candidate_y = self.y + (self.u * sin_yaw + self.v * cos_yaw) * dt
        candidate_yaw = normalize_angle(self.yaw + self.r * dt)

        collision_name = self.first_kinematic_collision(
            candidate_x, candidate_y, candidate_yaw
        )
        if collision_name:
            # Check if we are moving away from the colliding object
            moving_away = False
            colliding_obj = None
            for obj in self.collision_objects:
                if obj["name"] == collision_name:
                    colliding_obj = obj
                    break
            if colliding_obj is not None:
                prev_dist = math.hypot(
                    previous_x - colliding_obj["x"],
                    previous_y - colliding_obj["y"],
                )
                cand_dist = math.hypot(
                    candidate_x - colliding_obj["x"],
                    candidate_y - colliding_obj["y"],
                )
                if cand_dist > prev_dist:
                    moving_away = True

            if moving_away:
                self.x = candidate_x
                self.y = candidate_y
                self.yaw = candidate_yaw
            else:
                self.x = previous_x
                self.y = previous_y
                self.yaw = candidate_yaw
                self.u *= -self.collision_rebound
                self.v *= 0.15
                self.r *= 0.20
                self.last_collision_time = now
                self.last_collision_name = collision_name
        else:
            self.x = candidate_x
            self.y = candidate_y
            self.yaw = candidate_yaw

        ok = self.set_model_pose()
        self.publish_odometry()
        if (
            not ok
            and now - self.startup_time > self.startup_grace
            and now - self.last_warn > 2.0
        ):
            self.get_logger().warn(
                f"Waiting for Gazebo pose service/model on {self.service}; "
                f"model={self.model_name} result={self.last_pose_result} "
                f"response={self.last_pose_response}"
            )
            self.last_warn = now

        if now - self.last_status > 1.0:
            roll, pitch, heave = self.wave_motion(now)
            collision_active = now - self.last_collision_time < 0.50
            self.status_pub.publish(
                String(
                    data=(
                        f"ok={ok} x={self.x:.2f} y={self.y:.2f} "
                        f"z={self.z + heave:.2f} yaw={math.degrees(self.yaw):.1f}deg "
                        f"model={self.model_name} "
                        f"pose_result={self.last_pose_result} "
                        f"pose_response={self.last_pose_response} "
                        f"u={self.u:.2f} r={self.r:.2f} "
                        f"left={self.left_thrust:.1f}N right={self.right_thrust:.1f}N "
                        f"servo_l={math.degrees(self.left_steer):.1f}deg "
                        f"servo_r={math.degrees(self.right_steer):.1f}deg "
                        f"collision={self.last_collision_name if collision_active else 'clear'} "
                        f"wave_roll={math.degrees(roll):.1f}deg "
                        f"wave_pitch={math.degrees(pitch):.1f}deg"
                    )
                )
            )
            self.collision_pub.publish(Bool(data=collision_active))
            self.collision_status_pub.publish(
                String(
                    data=(
                        f"kinematic_collision={self.last_collision_name}"
                        if collision_active
                        else "clear"
                    )
                )
            )
            self.last_status = now

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.u
        msg.twist.twist.linear.y = self.v
        msg.twist.twist.angular.z = self.r
        msg.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.20, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.20, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.20, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03,
        ]
        msg.twist.covariance = [
            0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.08, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.50, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.50, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.50, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        self.odom_pub.publish(msg)

        # Publish TF: odom → base_link (direct, no EKF dependency)
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

    def integrate_propulsion(self, now, dt):
        if now - self.last_propulsion_time > self.timeout:
            left_thrust = 0.0
            right_thrust = 0.0
            left_steer = 0.0
            right_steer = 0.0
        else:
            left_thrust = self.left_thrust
            right_thrust = self.right_thrust
            left_steer = self.left_steer
            right_steer = self.right_steer

        left_fx, left_fy = self.thruster_force(left_thrust, left_steer)
        right_fx, right_fy = self.thruster_force(right_thrust, right_steer)
        fx = left_fx + right_fx
        fy = left_fy + right_fy
        torque_z = (
            self.thruster_x * left_fy
            - self.thruster_y * left_fx
            + self.thruster_x * right_fy
            + self.thruster_y * right_fx
        )

        u_dot = (fx - self.linear_drag * self.u) / self.mass
        v_dot = (fy - self.lateral_drag * self.v) / self.mass
        r_dot = (torque_z - self.yaw_drag * self.r) / self.yaw_inertia

        self.u = clamp(self.u + u_dot * dt, -self.max_linear, self.max_linear)
        self.v = clamp(self.v + v_dot * dt, -self.max_lateral, self.max_lateral)
        self.r = clamp(self.r + r_dot * dt, -self.max_yaw_rate, self.max_yaw_rate)

        if abs(left_thrust) < 1e-3 and abs(right_thrust) < 1e-3:
            self.u *= max(0.0, 1.0 - 1.8 * dt)
            self.v *= max(0.0, 1.0 - 2.2 * dt)
            self.r *= max(0.0, 1.0 - 2.0 * dt)

    def thruster_force(self, thrust_cmd, steer_angle):
        thrust = self.effective_thrust_scale * thrust_cmd
        return thrust * math.cos(steer_angle), thrust * math.sin(steer_angle)

    def build_course_collision_objects(self):
        course_b = "lintasan_b" in self.world_name.lower()
        mirror = -1.0 if course_b else 1.0

        objects = []
        for y in (0.0, 4.8, 8.8):
            objects.append({"type": "circle", "name": f"left_gate_green_{y}", "x": -13.0, "y": y, "r": 0.10})
            objects.append({"type": "circle", "name": f"left_gate_red_{y}", "x": -11.0, "y": y, "r": 0.10})
            objects.append({"type": "circle", "name": f"right_gate_red_{y}", "x": 11.0, "y": y, "r": 0.10})
            objects.append({"type": "circle", "name": f"right_gate_green_{y}", "x": 13.0, "y": y, "r": 0.10})
        for x in (-5.2, -1.4, 2.4, 6.2):
            objects.append({"type": "circle", "name": f"top_gate_green_{x}", "x": x, "y": 13.0, "r": 0.10})
            objects.append({"type": "circle", "name": f"top_gate_red_{x}", "x": x, "y": 11.0, "r": 0.10})

        for y in (-10.0, -9.7, -9.4):
            objects.append({"type": "circle", "name": f"docking_blue_{y}", "x": mirror * 12.35, "y": y, "r": 0.10})

        # Keep target boxes and docking buoys collision-active. The dock deck is
        # already represented in Gazebo; adding it here can trap the kinematic
        # controller before the mission reaches docking mode.
        objects.extend(
            [
                {
                    "type": "circle",
                    "name": "floating_obstacle_1",
                    "x": mirror * -3.0,
                    "y": -2.0,
                    "r": 0.22,
                },
                {
                    "type": "circle",
                    "name": "floating_obstacle_2",
                    "x": mirror * 6.8,
                    "y": 2.4,
                    "r": 0.22,
                },
                {
                    "type": "box",
                    "name": "surface_target",
                    "x": mirror * -10.2,
                    "y": -5.4,
                    "hx": 0.30,
                    "hy": 0.30,
                    "yaw": 0.0,
                },
                {
                    "type": "box",
                    "name": "underwater_target",
                    "x": mirror * -11.0,
                    "y": -7.7,
                    "hx": 0.30,
                    "hy": 0.30,
                    "yaw": 0.0,
                },
            ]
        )
        return objects

    def first_kinematic_collision(self, x, y, yaw):
        if not self.kinematic_collision_enabled:
            return ""
        for obj in self.collision_objects:
            if obj["type"] == "circle" and self.circle_hits_hull(x, y, yaw, obj):
                return obj["name"]
            if obj["type"] == "box" and self.box_hits_hull(x, y, yaw, obj):
                return obj["name"]
        return ""

    def circle_hits_hull(self, x, y, yaw, obj):
        # Use an oriented hull footprint instead of one large radius. The gate
        # buoys are only 2 m apart, so a circular proxy can stop the boat even
        # when it is correctly passing through the center.
        local_x, local_y = self.world_to_hull_local(obj["x"], obj["y"], x, y, yaw)
        closest_x = clamp(local_x, -self.hull_half_length, self.hull_half_length)
        closest_y = clamp(local_y, -self.hull_half_width, self.hull_half_width)
        distance = math.hypot(local_x - closest_x, local_y - closest_y)
        return distance <= obj["r"] + self.collision_margin

    def box_hits_hull(self, x, y, yaw, obj):
        return self.oriented_boxes_overlap(
            x,
            y,
            yaw,
            self.hull_half_length,
            self.hull_half_width,
            obj["x"],
            obj["y"],
            obj["yaw"],
            obj["hx"],
            obj["hy"],
            self.collision_margin,
        )

    def world_to_hull_local(self, point_x, point_y, hull_x, hull_y, hull_yaw):
        dx = point_x - hull_x
        dy = point_y - hull_y
        cy = math.cos(-hull_yaw)
        sy = math.sin(-hull_yaw)
        return dx * cy - dy * sy, dx * sy + dy * cy

    def oriented_boxes_overlap(
        self,
        ax,
        ay,
        ayaw,
        ahx,
        ahy,
        bx,
        by,
        byaw,
        bhx,
        bhy,
        margin,
    ):
        a_axes = self.box_axes(ayaw)
        b_axes = self.box_axes(byaw)
        dx = bx - ax
        dy = by - ay

        for axis_x, axis_y in (*a_axes, *b_axes):
            distance = abs(dx * axis_x + dy * axis_y)
            a_radius = self.projected_radius(axis_x, axis_y, a_axes, ahx, ahy)
            b_radius = self.projected_radius(axis_x, axis_y, b_axes, bhx, bhy)
            if distance > a_radius + b_radius + margin:
                return False
        return True

    def box_axes(self, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return ((cos_yaw, sin_yaw), (-sin_yaw, cos_yaw))

    def projected_radius(self, axis_x, axis_y, box_axes, half_x, half_y):
        x_axis, y_axis = box_axes
        return (
            half_x * abs(axis_x * x_axis[0] + axis_y * x_axis[1])
            + half_y * abs(axis_x * y_axis[0] + axis_y * y_axis[1])
        )

    def wave_motion(self, now):
        if not self.wave_enabled:
            return 0.0, 0.0, 0.0
        phase = 2.0 * math.pi * now / self.wave_period
        travel = 0.35 * self.x - 0.22 * self.y
        roll = self.wave_roll * math.sin(phase + travel)
        pitch = self.wave_pitch * math.sin(phase * 0.82 + 0.55 * travel + 1.1)
        heave = self.wave_amplitude * math.sin(phase + travel)
        return roll, pitch, heave

    def set_model_pose(self):
        # Use the model name only. Subscribing to the full /pose/info stream
        # while issuing high-rate blocking pose requests can make Gazebo reject
        # intermittent requests and show flicker.
        roll, pitch, heave = self.wave_motion(time.monotonic())
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, self.yaw)
        pose = Pose()
        pose.name = self.model_name
        pose.position.x = self.x
        pose.position.y = self.y
        # Keep the visual waterline above the wave mesh to avoid z-fighting flicker.
        pose.position.z = max(self.minimum_visual_z, self.z + heave)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        result, response = self.gz_node.request(
            self.service, pose, Pose, Boolean, self.service_timeout
        )
        self.last_pose_result = bool(result)
        self.last_pose_response = bool(response.data) if response is not None else False
        return bool(self.last_pose_result and self.last_pose_response)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarPoseController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
