#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ManualAutonomyMux(Node):
    """Select manual joystick or autonomous waypoint command stream."""

    def __init__(self):
        super().__init__("manual_autonomy_mux")
        self.declare_parameter("auto_mode", False)
        # Legacy override. Leave empty so auto_mode decides startup mode.
        self.declare_parameter("default_mode", "")
        self.declare_parameter("manual_timeout_s", 0.5)
        self.declare_parameter("auto_timeout_s", 1.0)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.mode = self.startup_mode()
        self.estop = False
        self.manual_cmd = Twist()
        self.auto_cmd = Twist()
        self.last_manual_time = 0.0
        self.last_auto_time = 0.0
        self.manual_timeout = float(self.get_parameter("manual_timeout_s").value)
        self.auto_timeout = float(self.get_parameter("auto_timeout_s").value)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/asv/control/mode_status", 10)

        self.create_subscription(Twist, "/cmd_vel_manual", self.on_manual, 10)
        self.create_subscription(Twist, "/cmd_vel_auto", self.on_auto, 10)
        self.create_subscription(String, "/asv/mode", self.on_mode, 10)
        self.create_subscription(Bool, "/asv/emergency_stop", self.on_estop, 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

    def on_manual(self, msg: Twist):
        self.manual_cmd = msg
        self.last_manual_time = time.monotonic()

    def on_auto(self, msg: Twist):
        self.auto_cmd = msg
        self.last_auto_time = time.monotonic()

    def on_mode(self, msg: String):
        requested = msg.data.strip().lower()
        if requested in ("manual", "auto", "autonomous"):
            # Joystick buttons or terminal commands can switch modes at runtime.
            self.mode = "auto" if requested == "autonomous" else requested
        elif requested in ("stop", "estop", "emergency_stop"):
            self.estop = True
        else:
            self.get_logger().warn(f"Ignoring unknown ASV mode: {msg.data}")

    def on_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def on_timer(self):
        now = time.monotonic()
        cmd = Twist()
        reason = "estop" if self.estop else "selected"

        if not self.estop:
            if self.mode == "manual":
                if now - self.last_manual_time <= self.manual_timeout:
                    cmd = self.manual_cmd
                else:
                    reason = "manual_timeout"
            elif self.mode == "auto":
                if now - self.last_auto_time <= self.auto_timeout:
                    cmd = self.auto_cmd
                else:
                    reason = "auto_timeout"
            else:
                reason = "unknown_mode"

        self.cmd_pub.publish(cmd)
        self.status_pub.publish(
            String(
                data=(
                    f"mode={self.mode} estop={self.estop} reason={reason} "
                    f"vx={cmd.linear.x:.2f} wz={cmd.angular.z:.2f}"
                )
            )
        )

    def startup_mode(self) -> str:
        default_mode = str(self.get_parameter("default_mode").value).strip().lower()
        if default_mode in ("auto", "autonomous"):
            return "auto"
        if default_mode == "manual":
            return "manual"
        # Main startup switch: true starts waypoint following, false starts joystick mode.
        return "auto" if bool(self.get_parameter("auto_mode").value) else "manual"


def main(args=None):
    rclpy.init(args=args)
    node = ManualAutonomyMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
