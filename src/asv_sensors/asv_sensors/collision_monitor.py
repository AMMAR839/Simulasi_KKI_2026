#!/usr/bin/env python3
"""Summarize Gazebo contact sensor output for ASV collision detection."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, String


CONTACT_TOPICS = [
    "/asv/collisions/hull",
    "/asv/collisions/left_thruster",
    "/asv/collisions/right_thruster",
    "/asv/collisions/left_propeller",
    "/asv/collisions/right_propeller",
    "/asv/collisions/lidar",
    "/asv/collisions/front_camera",
    "/asv/collisions/down_camera",
    "/kki/collisions/objects",
]


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__("contact_monitor")
        self.declare_parameter("contact_hold_s", 0.35)
        self.contact_hold = float(self.get_parameter("contact_hold_s").value)
        self.last_contact_time = 0.0
        self.last_summary = "clear"
        self.kinematic_summary = "kinematic_collision"

        self.detected_pub = self.create_publisher(
            Bool, "/asv/collision/detected", 10
        )
        self.status_pub = self.create_publisher(
            String, "/asv/collision/status", 10
        )

        for topic in CONTACT_TOPICS:
            self.create_subscription(
                Contacts,
                topic,
                lambda msg, source=topic: self.on_contacts(msg, source),
                10,
            )
        self.create_subscription(
            Bool,
            "/asv/collision/kinematic_detected",
            self.on_kinematic_detected,
            10,
        )
        self.create_subscription(
            String,
            "/asv/collision/kinematic_status",
            self.on_kinematic_status,
            10,
        )

        self.create_timer(0.10, self.on_timer)

    def on_contacts(self, msg: Contacts, source: str):
        if not msg.contacts:
            return

        details = []
        for contact in msg.contacts[:3]:
            left = contact.collision1.name or "collision_1"
            right = contact.collision2.name or "collision_2"
            depth = max(contact.depths) if contact.depths else 0.0
            details.append(f"{left} <-> {right} depth={depth:.3f}m")

        # Keep the latest contact active briefly because contact sensors can
        # publish intermittently when a pose-driven model just touches an object.
        self.last_contact_time = time.monotonic()
        self.last_summary = f"{source}: " + " | ".join(details)

    def on_kinematic_detected(self, msg: Bool):
        if msg.data:
            self.last_contact_time = time.monotonic()
            self.last_summary = self.kinematic_summary

    def on_kinematic_status(self, msg: String):
        data = msg.data.strip()
        if not data or data == "clear":
            return

        # The pose-driven ASV cannot rely on Gazebo to push it back, so the
        # planar controller publishes its own collision stop/rebound status.
        self.kinematic_summary = data
        self.last_contact_time = time.monotonic()
        self.last_summary = data

    def on_timer(self):
        active = (time.monotonic() - self.last_contact_time) <= self.contact_hold
        self.detected_pub.publish(Bool(data=active))
        self.status_pub.publish(
            String(data=self.last_summary if active else "clear")
        )


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
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
