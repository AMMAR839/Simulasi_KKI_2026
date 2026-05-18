#!/usr/bin/env python3
"""Small Linux joystick reader for manual ASV teleop.

This avoids a hard runtime dependency on ros-jazzy-joy / teleop_twist_joy.
It reads /dev/input/js0, publishes /joy for mode buttons, and publishes
/cmd_vel_manual for throttle + steering.
"""

import os
import struct
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class SimpleJoystickTeleop(Node):
    def __init__(self):
        super().__init__("simple_joystick_teleop")
        self.declare_parameter("device", "/dev/input/js0")
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("autorepeat_rate", 20.0)
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("scale_linear_x", 1.2)
        self.declare_parameter("scale_linear_turbo_x", 1.8)
        self.declare_parameter("axis_angular_yaw", 0)
        self.declare_parameter("scale_angular_yaw", 0.9)
        self.declare_parameter("scale_angular_turbo_yaw", 1.4)
        self.declare_parameter("enable_button", 2)
        self.declare_parameter("enable_turbo_button", 3)
        self.declare_parameter("require_enable_button", True)

        self.device = str(self.get_parameter("device").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.axis_linear = int(self.get_parameter("axis_linear_x").value)
        self.scale_linear = float(self.get_parameter("scale_linear_x").value)
        self.scale_linear_turbo = float(
            self.get_parameter("scale_linear_turbo_x").value
        )
        self.axis_yaw = int(self.get_parameter("axis_angular_yaw").value)
        self.scale_yaw = float(self.get_parameter("scale_angular_yaw").value)
        self.scale_yaw_turbo = float(
            self.get_parameter("scale_angular_turbo_yaw").value
        )
        self.enable_button = int(self.get_parameter("enable_button").value)
        self.turbo_button = int(self.get_parameter("enable_turbo_button").value)
        self.require_enable = bool(self.get_parameter("require_enable_button").value)

        self.axes = [0.0] * 8
        self.buttons = [0] * 12
        self.joy_fd = None
        self.last_open_attempt = 0.0
        self.last_warn = 0.0

        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_manual", 10)

        rate = float(self.get_parameter("autorepeat_rate").value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

    def open_device(self):
        now = time.monotonic()
        if self.joy_fd is not None or now - self.last_open_attempt < 1.0:
            return
        self.last_open_attempt = now
        try:
            self.joy_fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            self.get_logger().info(f"Joystick opened: {self.device}")
        except OSError as exc:
            if now - self.last_warn > 5.0:
                self.get_logger().warn(
                    f"Joystick {self.device} not available yet: {exc}"
                )
                self.last_warn = now

    def on_timer(self):
        self.open_device()
        self.read_events()
        self.publish_joy()
        self.publish_cmd()

    def read_events(self):
        if self.joy_fd is None:
            return
        while True:
            try:
                data = os.read(self.joy_fd, 8)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warn(f"Joystick disconnected: {exc}")
                os.close(self.joy_fd)
                self.joy_fd = None
                return
            if len(data) != 8:
                return
            _, raw_value, event_type, number = struct.unpack("IhBB", data)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS:
                self.ensure_axis(number)
                self.axes[number] = self.apply_deadzone(raw_value / 32767.0)
            elif event_type == JS_EVENT_BUTTON:
                self.ensure_button(number)
                self.buttons[number] = 1 if raw_value else 0

    def publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self.axes)
        msg.buttons = list(self.buttons)
        self.joy_pub.publish(msg)

    def publish_cmd(self):
        enabled = not self.require_enable or self.button(self.enable_button)
        turbo = self.button(self.turbo_button)
        cmd = Twist()
        if enabled:
            linear_scale = self.scale_linear_turbo if turbo else self.scale_linear
            yaw_scale = self.scale_yaw_turbo if turbo else self.scale_yaw
            # Axis 1 is throttle, axis 0 is steering by default.
            cmd.linear.x = self.axis(self.axis_linear) * linear_scale
            cmd.angular.z = self.axis(self.axis_yaw) * yaw_scale
        self.cmd_pub.publish(cmd)

    def axis(self, index):
        return self.axes[index] if 0 <= index < len(self.axes) else 0.0

    def button(self, index):
        return 0 <= index < len(self.buttons) and self.buttons[index] == 1

    def ensure_axis(self, index):
        while index >= len(self.axes):
            self.axes.append(0.0)

    def ensure_button(self, index):
        while index >= len(self.buttons):
            self.buttons.append(0)

    def apply_deadzone(self, value):
        value = clamp(value, -1.0, 1.0)
        return 0.0 if abs(value) < self.deadzone else value

    def destroy_node(self):
        if self.joy_fd is not None:
            os.close(self.joy_fd)
            self.joy_fd = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleJoystickTeleop()
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


if __name__ == "__main__":
    main()
