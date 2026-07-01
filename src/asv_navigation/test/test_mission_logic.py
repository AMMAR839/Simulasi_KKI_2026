from asv_navigation.mission_logic import (
    bounded_thruster_trim,
    crossed_gate,
    is_optional_recovery_waypoint,
    optimized_lap2_route,
    optimized_capture_waypoint,
    photo_capture_ready,
)


def test_crossed_gate_requires_crossing_inside_buoys():
    green = (13.0, 0.0)
    red = (11.0, 0.0)
    assert crossed_gate((12.0, -0.5), (12.0, 0.5), green, red)
    assert not crossed_gate((13.5, -0.5), (13.5, 0.5), green, red)
    assert not crossed_gate((12.0, -0.5), (12.0, -0.1), green, red)


def test_thruster_trim_is_bounded():
    assert bounded_thruster_trim(10.0) == -0.10
    assert bounded_thruster_trim(-10.0) == 0.10
    assert abs(bounded_thruster_trim(0.2) + 0.03) < 1e-9


def test_capture_pose_replaces_only_pose_fields():
    fallback = {"name": "surface_photo_green_box", "x": 1.0, "y": 2.0, "speed": 0.2}
    result = optimized_capture_waypoint((3.0, 4.0, 0.5), fallback)
    assert result["x"] == 3.0
    assert result["y"] == 4.0
    assert result["yaw"] == 0.5
    assert result["speed"] == 0.2
    assert fallback["x"] == 1.0


def test_photo_requires_expected_camera_fresh_hsv_and_lidar_range():
    args = {
        "expected_camera": "down",
        "fresh": True,
        "range_m": 2.35,
        "normalized_error": 0.06,
        "target_range": 2.2,
        "range_tolerance": 0.45,
    }
    assert photo_capture_ready(detection_camera="down", **args)
    assert not photo_capture_ready(detection_camera="front", **args)
    assert not photo_capture_ready(detection_camera="down", **{**args, "fresh": False})
    assert not photo_capture_ready(detection_camera="down", **{**args, "range_m": None})
    assert not photo_capture_ready(
        detection_camera="down", **{**args, "normalized_error": 0.20}
    )


def test_photo_supports_camera_specific_center_tolerance():
    args = {
        "detection_camera": "down",
        "expected_camera": "down",
        "fresh": True,
        "range_m": 3.3,
        "target_range": 1.8,
        "range_tolerance": 1.6,
        "center_tolerance": 0.35,
    }
    assert photo_capture_ready(normalized_error=0.30, **args)
    assert not photo_capture_ready(normalized_error=0.40, **args)


def test_lap2_route_compresses_gate_micro_waypoints_when_map_is_available():
    waypoints = [
        {"name": "pre_gate8", "x": 12.0, "y": -4.0, "speed": 0.30},
        {"name": "gate8_approach", "x": 12.0, "y": -1.0, "speed": 0.34},
        {"name": "gate8_center", "x": 12.0, "y": 0.0, "speed": 0.30},
        {"name": "gate8_exit", "x": 12.0, "y": 1.0, "speed": 0.34},
        {"name": "surface_photo_green_box", "x": -8.2, "y": -5.4, "speed": 0.22},
    ]
    route, compressed = optimized_lap2_route(waypoints, coverage_percent=12.0)
    assert compressed
    assert [item["name"] for item in route] == [
        "pre_gate8",
        "gate8_exit",
        "surface_photo_green_box",
    ]
    assert route[1]["speed"] >= 0.38
    assert route[-1]["speed"] <= 0.36


def test_lap2_route_keeps_full_gate_waypoints_when_map_coverage_is_low():
    waypoints = [
        {"name": "gate8_approach", "x": 12.0, "y": -1.0, "speed": 0.34},
        {"name": "gate8_center", "x": 12.0, "y": 0.0, "speed": 0.30},
        {"name": "gate8_exit", "x": 12.0, "y": 1.0, "speed": 0.34},
    ]
    route, compressed = optimized_lap2_route(
        waypoints, coverage_percent=4.0, min_coverage_percent=8.0
    )
    assert not compressed
    assert [item["name"] for item in route] == [
        "gate8_approach",
        "gate8_center",
        "gate8_exit",
    ]


def test_optional_recovery_waypoint_names_are_limited_to_safe_exits():
    assert is_optional_recovery_waypoint("photo_exit_south")
    assert is_optional_recovery_waypoint("dock_retreat")
    assert is_optional_recovery_waypoint("undock_staging")
    assert not is_optional_recovery_waypoint("gate8_exit")
    assert not is_optional_recovery_waypoint("dock_contact_blue_buoys")
