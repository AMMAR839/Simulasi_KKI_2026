#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def point_segment_distance(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def is_turn_waypoint(name):
    name = str(name)
    return any(token in name for token in ("turn", "corner", "entry"))


def is_gate_waypoint(name):
    name = str(name)
    return name.startswith("gate") and name.endswith(("_approach", "_center", "_exit"))


def segment_lane_half_width(first_name, second_name, half_width, turn_half_width):
    if is_gate_waypoint(first_name) or is_gate_waypoint(second_name):
        return half_width
    if is_turn_waypoint(first_name) or is_turn_waypoint(second_name):
        return turn_half_width
    return half_width


def waypoint_lane_half_width(name, half_width, turn_half_width):
    if is_gate_waypoint(name):
        return half_width
    if is_turn_waypoint(name):
        return turn_half_width
    return half_width


class PreferredLaneServer(Node):
    def __init__(self):
        super().__init__("preferred_lane_server")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("resolution_m", 0.10)
        self.declare_parameter("width_m", 60.0)
        self.declare_parameter("height_m", 60.0)
        self.declare_parameter("origin_x", -30.0)
        self.declare_parameter("origin_y", -30.0)
        self.declare_parameter("lane_half_width_m", 1.40)
        self.declare_parameter("turn_lane_half_width_m", 2.00)
        self.declare_parameter("outside_cost", 100)
        self.mask = self._build_mask()
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mask_pub = self.create_publisher(
            OccupancyGrid, "/asv/navigation/lane_mask", qos
        )
        self.info_pub = self.create_publisher(
            CostmapFilterInfo, "/asv/navigation/keepout_info", qos
        )
        self._published = False
        self.create_timer(1.0, self._publish)

    def _build_mask(self):
        path = Path(str(self.get_parameter("waypoint_file").value))
        data = yaml.safe_load(path.read_text()) if path.exists() else {}
        waypoints = [
            {
                "name": str(item.get("name", "")),
                "x": float(item["x"]),
                "y": float(item["y"]),
            }
            for item in data.get("waypoints", [])
        ]
        points = [(item["x"], item["y"]) for item in waypoints]
        resolution = float(self.get_parameter("resolution_m").value)
        width = int(round(float(self.get_parameter("width_m").value) / resolution))
        height = int(round(float(self.get_parameter("height_m").value) / resolution))
        origin_x = float(self.get_parameter("origin_x").value)
        origin_y = float(self.get_parameter("origin_y").value)
        half_width = float(self.get_parameter("lane_half_width_m").value)
        turn_half_width = max(
            half_width,
            float(self.get_parameter("turn_lane_half_width_m").value),
        )
        outside = int(self.get_parameter("outside_cost").value)
        mask = np.full((height, width), outside, dtype=np.int8)
        if len(points) < 2:
            return mask
        for first, second in zip(waypoints[:-1], waypoints[1:]):
            ax, ay = first["x"], first["y"]
            bx, by = second["x"], second["y"]
            segment_width = segment_lane_half_width(
                first["name"], second["name"], half_width, turn_half_width
            )
            min_x = max(0, int((min(ax, bx) - segment_width - origin_x) / resolution))
            max_x = min(
                width - 1,
                int((max(ax, bx) + segment_width - origin_x) / resolution),
            )
            min_y = max(0, int((min(ay, by) - segment_width - origin_y) / resolution))
            max_y = min(
                height - 1,
                int((max(ay, by) + segment_width - origin_y) / resolution),
            )
            for cell_y in range(min_y, max_y + 1):
                py = origin_y + (cell_y + 0.5) * resolution
                for cell_x in range(min_x, max_x + 1):
                    px = origin_x + (cell_x + 0.5) * resolution
                    if (
                        point_segment_distance(px, py, ax, ay, bx, by)
                        <= segment_width
                    ):
                        mask[cell_y, cell_x] = 0
        for item in waypoints:
            px, py = item["x"], item["y"]
            padding = int(
                math.ceil(
                    waypoint_lane_half_width(
                        item["name"], half_width, turn_half_width
                    )
                    / resolution
                )
            )
            cx = int((px - origin_x) / resolution)
            cy = int((py - origin_y) / resolution)
            mask[
                max(0, cy - padding) : min(height, cy + padding + 1),
                max(0, cx - padding) : min(width, cx + padding + 1),
            ] = 0
        return mask

    def _publish(self):
        if self._published:
            return
        now = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.info.map_load_time = now
        msg.info.resolution = float(self.get_parameter("resolution_m").value)
        msg.info.width = self.mask.shape[1]
        msg.info.height = self.mask.shape[0]
        msg.info.origin.position.x = float(self.get_parameter("origin_x").value)
        msg.info.origin.position.y = float(self.get_parameter("origin_y").value)
        msg.info.origin.orientation.w = 1.0
        msg.data = self.mask.reshape(-1).tolist()
        self.mask_pub.publish(msg)

        info = CostmapFilterInfo()
        info.header.stamp = now
        info.header.frame_id = "map"
        info.type = 0
        info.filter_mask_topic = "/asv/navigation/lane_mask"
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)
        self._published = True


def main(args=None):
    rclpy.init(args=args)
    node = PreferredLaneServer()
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
