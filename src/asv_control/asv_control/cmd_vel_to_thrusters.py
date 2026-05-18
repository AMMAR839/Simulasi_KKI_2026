#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String


class CmdVelToThrusters(Node):
    """Map body velocity commands to steerable twin-thruster commands."""

    def __init__(self):
        super().__init__("cmd_vel_to_thrusters")
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
            "left_steering_topic",
            "/asv/thrusters/left_steering/cmd_pos",
        )
        self.declare_parameter(
            "right_steering_topic",
            "/asv/thrusters/right_steering/cmd_pos",
        )
        self.declare_parameter(
            "left_propeller_velocity_topic",
            "/model/asv_kki_2026/joint/left_propeller_joint/cmd_vel",
        )
        self.declare_parameter(
            "right_propeller_velocity_topic",
            "/model/asv_kki_2026/joint/right_propeller_joint/cmd_vel",
        )
        # These values decide how hard the propellers push for a requested speed.
        self.declare_parameter("max_forward_thrust_n", 48.0)
        self.declare_parameter("max_reverse_thrust_n", 28.0)
        self.declare_parameter("max_speed_cmd_mps", 0.9)
        self.declare_parameter("yaw_to_thrust_n_per_radps", 5.0)
        self.declare_parameter("pivot_yaw_to_thrust_n_per_radps", 12.0)
        self.declare_parameter("max_yaw_rate_cmd_radps", 0.9)
        self.declare_parameter("max_steering_angle_rad", 0.60)
        self.declare_parameter("steering_gain", 0.85)
        self.declare_parameter("min_vectoring_thrust_n", 2.5)
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("propeller_spin_radps_per_n", 11.0)
        self.declare_parameter("max_propeller_spin_radps", 520.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.apply_parameters(self.parameter_values())
        self.add_on_set_parameters_callback(self.on_parameter_update)

        left_topic = self.get_parameter("left_thrust_topic").value
        right_topic = self.get_parameter("right_thrust_topic").value
        left_steering_topic = self.get_parameter("left_steering_topic").value
        right_steering_topic = self.get_parameter("right_steering_topic").value
        self.left_pub = self.create_publisher(Float64, left_topic, 10)
        self.right_pub = self.create_publisher(Float64, right_topic, 10)
        self.left_steer_pub = self.create_publisher(Float64, left_steering_topic, 10)
        self.right_steer_pub = self.create_publisher(Float64, right_steering_topic, 10)
        self.left_prop_spin_pub = self.create_publisher(
            Float64, self.get_parameter("left_propeller_velocity_topic").value, 10
        )
        self.right_prop_spin_pub = self.create_publisher(
            Float64, self.get_parameter("right_propeller_velocity_topic").value, 10
        )
        self.status_pub = self.create_publisher(String, "/asv/control/thruster_status", 10)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.create_subscription(Twist, cmd_topic, self.on_cmd_vel, 10)

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_nonzero_log = 0.0
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

    def parameter_values(self):
        return {
            "max_forward_thrust_n": float(
                self.get_parameter("max_forward_thrust_n").value
            ),
            "max_reverse_thrust_n": float(
                self.get_parameter("max_reverse_thrust_n").value
            ),
            "max_speed_cmd_mps": float(self.get_parameter("max_speed_cmd_mps").value),
            "yaw_to_thrust_n_per_radps": float(
                self.get_parameter("yaw_to_thrust_n_per_radps").value
            ),
            "pivot_yaw_to_thrust_n_per_radps": float(
                self.get_parameter("pivot_yaw_to_thrust_n_per_radps").value
            ),
            "max_yaw_rate_cmd_radps": float(
                self.get_parameter("max_yaw_rate_cmd_radps").value
            ),
            "max_steering_angle_rad": float(
                self.get_parameter("max_steering_angle_rad").value
            ),
            "steering_gain": float(self.get_parameter("steering_gain").value),
            "min_vectoring_thrust_n": float(
                self.get_parameter("min_vectoring_thrust_n").value
            ),
            "steering_sign": float(self.get_parameter("steering_sign").value),
            "propeller_spin_radps_per_n": float(
                self.get_parameter("propeller_spin_radps_per_n").value
            ),
            "max_propeller_spin_radps": float(
                self.get_parameter("max_propeller_spin_radps").value
            ),
            "cmd_timeout_s": float(self.get_parameter("cmd_timeout_s").value),
        }

    def apply_parameters(self, values):
        self.max_forward = max(0.0, values["max_forward_thrust_n"])
        self.max_reverse = max(0.0, values["max_reverse_thrust_n"])
        self.max_speed = max(0.05, values["max_speed_cmd_mps"])
        self.yaw_scale = max(0.0, values["yaw_to_thrust_n_per_radps"])
        self.pivot_yaw_scale = max(0.0, values["pivot_yaw_to_thrust_n_per_radps"])
        self.max_yaw_rate = max(0.05, values["max_yaw_rate_cmd_radps"])
        self.max_steering_angle = max(0.0, values["max_steering_angle_rad"])
        self.steering_gain = max(0.0, values["steering_gain"])
        self.min_vectoring_thrust = max(0.0, values["min_vectoring_thrust_n"])
        self.steering_sign = -1.0 if values["steering_sign"] < 0.0 else 1.0
        self.spin_scale = max(0.0, values["propeller_spin_radps_per_n"])
        self.max_spin = max(1.0, values["max_propeller_spin_radps"])
        self.timeout = max(0.05, values["cmd_timeout_s"])

    def on_parameter_update(self, parameters):
        values = self.parameter_values()
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value
        try:
            self.apply_parameters(values)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()
        if abs(msg.linear.x) > 0.02 or abs(msg.angular.z) > 0.02:
            now = time.monotonic()
            if now - self.last_nonzero_log > 1.0:
                self.get_logger().info(
                    f"cmd_vel received: vx={msg.linear.x:.2f}, "
                    f"wz={msg.angular.z:.2f}"
                )
                self.last_nonzero_log = now

    def on_timer(self):
        if time.monotonic() - self.last_cmd_time > self.timeout:
            left = 0.0
            right = 0.0
            steer = 0.0
            state = "timeout_stop"
        else:
            left, right, steer = self._mix_command(self.last_cmd)
            state = "active"

        self.left_pub.publish(Float64(data=left))
        self.right_pub.publish(Float64(data=right))
        self.left_steer_pub.publish(Float64(data=steer))
        self.right_steer_pub.publish(Float64(data=steer))
        left_spin = self._thrust_to_spin(left)
        right_spin = -self._thrust_to_spin(right)
        self.left_prop_spin_pub.publish(Float64(data=left_spin))
        self.right_prop_spin_pub.publish(Float64(data=right_spin))
        self.status_pub.publish(
            String(
                data=(
                    f"{state}: left={left:.2f}N right={right:.2f}N "
                    f"servo={math.degrees(steer):.1f}deg "
                    f"prop_l={left_spin:.1f}rad/s prop_r={right_spin:.1f}rad/s"
                )
            )
        )

    def _mix_command(self, cmd: Twist) -> tuple[float, float, float]:
        throttle = self._scale_throttle(float(cmd.linear.x))
        yaw_cmd = float(cmd.angular.z)
        if not math.isfinite(yaw_cmd):
            yaw_cmd = 0.0
        yaw_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_cmd))

        normalized_yaw = yaw_cmd / max(self.max_yaw_rate, 1e-6)
        steer = (
            self.steering_sign
            * self.steering_gain
            * normalized_yaw
            * self.max_steering_angle
        )
        steer = max(-self.max_steering_angle, min(self.max_steering_angle, steer))

        if abs(throttle) < self.min_vectoring_thrust and abs(yaw_cmd) > 0.03:
            pivot = self.pivot_yaw_scale * yaw_cmd
            return self._clamp(-pivot), self._clamp(pivot), 0.0

        turn = self.yaw_scale * yaw_cmd
        left = self._clamp(throttle - turn)
        right = self._clamp(throttle + turn)
        return left, right, steer

    def _scale_throttle(self, speed_cmd: float) -> float:
        if not math.isfinite(speed_cmd):
            return 0.0
        normalized = max(-1.0, min(1.0, speed_cmd / max(self.max_speed, 1e-6)))
        if normalized >= 0.0:
            return normalized * self.max_forward
        return normalized * self.max_reverse

    def _clamp(self, value: float) -> float:
        return max(-self.max_reverse, min(self.max_forward, value))

    def _thrust_to_spin(self, thrust: float) -> float:
        spin = thrust * self.spin_scale
        return max(-self.max_spin, min(self.max_spin, spin))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrusters()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
