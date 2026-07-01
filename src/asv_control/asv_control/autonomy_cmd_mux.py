#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class AutonomyCmdMux(Node):
    """Select exactly one autonomous command source with timeout-to-stop."""

    SOURCES = ("nav", "task", "dock")

    def __init__(self):
        super().__init__("autonomy_cmd_mux")
        self.declare_parameter("default_source", "nav")
        self.declare_parameter("source_timeout_s", 0.6)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("nav_topic", "/cmd_vel_nav")
        self.declare_parameter("task_topic", "/cmd_vel_task")
        self.declare_parameter("dock_topic", "/cmd_vel_dock")
        self.declare_parameter("output_topic", "/cmd_vel_auto_raw")

        source = str(self.get_parameter("default_source").value).strip().lower()
        self.source = source if source in self.SOURCES else "nav"
        self.timeout = max(
            0.05, float(self.get_parameter("source_timeout_s").value)
        )
        self.commands = {name: Twist() for name in self.SOURCES}
        self.command_times = {name: 0.0 for name in self.SOURCES}
        self.output = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.status = self.create_publisher(
            String, "/asv/control/autonomy_mux_status", 10
        )
        for name in self.SOURCES:
            topic = str(self.get_parameter(f"{name}_topic").value)
            self.create_subscription(
                Twist,
                topic,
                lambda msg, selected=name: self._on_command(selected, msg),
                10,
            )
        self.create_subscription(
            String, "/asv/autonomy/source", self._on_source, 10
        )
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._on_timer)

    def _on_command(self, source, msg):
        self.commands[source] = msg
        self.command_times[source] = time.monotonic()

    def _on_source(self, msg):
        requested = msg.data.strip().lower()
        if requested not in self.SOURCES:
            self.get_logger().warn(f"Ignoring unknown autonomy source: {requested}")
            return
        if requested != self.source:
            self.source = requested
            self.get_logger().info(f"Autonomy source changed to {requested}")

    def _on_timer(self):
        age = time.monotonic() - self.command_times[self.source]
        timed_out = age > self.timeout
        command = Twist() if timed_out else self.commands[self.source]
        self.output.publish(command)
        self.status.publish(
            String(
                data=(
                    f"source={self.source} timeout={timed_out} age={age:.2f}s "
                    f"vx={command.linear.x:.2f} wz={command.angular.z:.2f}"
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyCmdMux()
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
