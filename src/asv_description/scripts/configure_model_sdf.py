#!/usr/bin/env python3
"""Render a temporary ASV model SDF with launch-time visualization options."""

import argparse
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET


def bool_text(value: str) -> str:
    normalized = str(value).strip().lower()
    return "true" if normalized in {"1", "true", "yes", "on"} else "false"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Source model.sdf path")
    parser.add_argument(
        "--lidar-visualize",
        "--show-lidar-visual",
        dest="lidar_visualize",
        default="true",
        help="true shows Gazebo LiDAR rays; false keeps the sensor running but hides rays",
    )
    args = parser.parse_args()

    source = Path(args.input)
    if not source.exists():
        raise FileNotFoundError(source)

    tree = ET.parse(source)
    root = tree.getroot()
    desired = bool_text(args.lidar_visualize)

    updated = False
    for sensor in root.findall(".//sensor"):
        if sensor.get("name") == "lidar_sensor":
            visualize = sensor.find("visualize")
            if visualize is None:
                visualize = ET.SubElement(sensor, "visualize")
            # This only controls Gazebo ray display. Topic /asv/lidar/scan remains active.
            visualize.text = desired
            updated = True

    if not updated:
        raise RuntimeError("lidar_sensor not found in model SDF")

    output = Path(tempfile.gettempdir()) / f"asv_kki_2026_model_lidar_{desired}.sdf"
    tree.write(output, encoding="utf-8", xml_declaration=True)
    sys.stdout.write(str(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
