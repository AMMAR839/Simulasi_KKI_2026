#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import Twist
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


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
        self.declare_parameter("surface_z", 0.03)
        self.declare_parameter("spawn_yaw", 1.2405)
        self.declare_parameter("mass_kg", 7.7)
        self.declare_parameter("yaw_inertia_kgm2", 0.56)
        # Physical response from propeller thrust. Raise slowly if acceleration
        # is too weak, but keep it below the point where waypoint overshoot grows.
        self.declare_parameter("effective_thrust_scale", 0.13)
        self.declare_parameter("thruster_x_m", -0.150)
        self.declare_parameter("thruster_y_m", 0.264)
        self.declare_parameter("linear_drag_n_per_mps", 4.5)
        self.declare_parameter("lateral_drag_n_per_mps", 12.0)
        self.declare_parameter("yaw_drag_nm_per_radps", 1.7)
        self.declare_parameter("max_linear_speed_mps", 0.90)
        self.declare_parameter("max_lateral_speed_mps", 0.18)
        self.declare_parameter("max_yaw_rate_radps", 0.80)
        self.declare_parameter("wave_effect_enabled", True)
        self.declare_parameter("wave_amplitude_m", 0.025)
        self.declare_parameter("wave_period_s", 3.5)
        self.declare_parameter("wave_roll_deg", 2.2)
        self.declare_parameter("wave_pitch_deg", 1.6)
        self.declare_parameter("service_timeout_ms", 1000)

        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.service = f"/world/{self.world_name}/set_pose/blocking"
        self.gz_node = GzNode()

        self.x = float(self.get_parameter("spawn_x").value)
        self.y = float(self.get_parameter("spawn_y").value)
        self.z = float(self.get_parameter("surface_z").value)
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

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.left_steer = 0.0
        self.right_steer = 0.0
        self.last_propulsion_time = 0.0
        self.last_update = time.monotonic()
        self.last_warn = 0.0
        self.last_status = 0.0

        self.status_pub = self.create_publisher(
            String, "/asv/control/planar_pose_status", 10
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

        rate = float(self.get_parameter("pose_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

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

    def on_timer(self):
        now = time.monotonic()
        dt = clamp(now - self.last_update, 0.0, 0.10)
        self.last_update = now

        self.integrate_propulsion(now, dt)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (self.u * cos_yaw - self.v * sin_yaw) * dt
        self.y += (self.u * sin_yaw + self.v * cos_yaw) * dt
        self.yaw = normalize_angle(self.yaw + self.r * dt)

        ok = self.set_model_pose()
        if not ok and now - self.last_warn > 2.0:
            self.get_logger().warn(
                f"Waiting for Gazebo pose service/model on {self.service}"
            )
            self.last_warn = now

        if now - self.last_status > 1.0:
            roll, pitch, heave = self.wave_motion(now)
            self.status_pub.publish(
                String(
                    data=(
                        f"ok={ok} x={self.x:.2f} y={self.y:.2f} "
                        f"z={self.z + heave:.2f} yaw={math.degrees(self.yaw):.1f}deg "
                        f"model={self.model_name} "
                        f"u={self.u:.2f} r={self.r:.2f} "
                        f"left={self.left_thrust:.1f}N right={self.right_thrust:.1f}N "
                        f"servo_l={math.degrees(self.left_steer):.1f}deg "
                        f"servo_r={math.degrees(self.right_steer):.1f}deg "
                        f"wave_roll={math.degrees(roll):.1f}deg "
                        f"wave_pitch={math.degrees(pitch):.1f}deg"
                    )
                )
            )
            self.last_status = now

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
        roll, pitch, heave = self.wave_motion(time.monotonic())
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, self.yaw)
        pose = Pose()
        pose.name = self.model_name
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z + heave
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        result, response = self.gz_node.request(
            self.service, pose, Pose, Boolean, self.service_timeout
        )
        return bool(result and response.data)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarPoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
