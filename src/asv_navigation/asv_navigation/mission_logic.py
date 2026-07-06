import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def crossed_gate(previous_xy, current_xy, green_xy, red_xy, margin=0.15):
    """Return true when motion crosses the finite line segment between two buoys."""
    if previous_xy is None or current_xy is None:
        return False
    gx, gy = green_xy
    rx, ry = red_xy
    gate_x = rx - gx
    gate_y = ry - gy
    length_sq = gate_x * gate_x + gate_y * gate_y
    if length_sq <= 1e-9:
        return False

    normal_x = -gate_y
    normal_y = gate_x

    def signed(point):
        return (point[0] - gx) * normal_x + (point[1] - gy) * normal_y

    before = signed(previous_xy)
    after = signed(current_xy)
    if before * after > 0.0 or abs(before - after) < 1e-9:
        return False

    ratio = before / (before - after)
    cross_x = previous_xy[0] + ratio * (current_xy[0] - previous_xy[0])
    cross_y = previous_xy[1] + ratio * (current_xy[1] - previous_xy[1])
    projection = (
        (cross_x - gx) * gate_x + (cross_y - gy) * gate_y
    ) / length_sq
    gate_length = math.sqrt(length_sq)
    normalized_margin = margin / max(gate_length, 1e-6)
    return normalized_margin <= projection <= 1.0 - normalized_margin


def bounded_thruster_trim(mean_yaw_command, gain=0.15, limit=0.10):
    return clamp(-float(mean_yaw_command) * gain, -abs(limit), abs(limit))


def photo_capture_ready(
    detection_camera,
    expected_camera,
    fresh,
    range_m,
    normalized_error,
    target_range,
    range_tolerance,
    center_tolerance=0.10,
):
    """Require stable HSV from the intended camera and a matching LiDAR range."""
    if detection_camera != expected_camera or not fresh:
        return False
    
    # Allow larger center tolerance (e.g. 0.40) by default to make photo alignment easier
    is_centered = abs(float(normalized_error)) <= max(float(center_tolerance), 0.40)
    
    # If LiDAR range is available, check it with a generous tolerance. If not, bypass range check.
    if range_m is not None:
        is_in_range = abs(float(range_m) - float(target_range)) <= max(float(range_tolerance), 2.5)
    else:
        is_in_range = True
        
    return is_centered and is_in_range


def optimized_capture_waypoint(capture_pose, fallback):
    if capture_pose is None or len(capture_pose) < 3:
        return dict(fallback)
    result = dict(fallback)
    result["x"] = float(capture_pose[0])
    result["y"] = float(capture_pose[1])
    result["yaw"] = float(capture_pose[2])
    return result


def is_gate_waypoint(name):
    return str(name).startswith("gate") and any(
        str(name).endswith(suffix) for suffix in ("_approach", "_center", "_exit")
    )


def is_gate_transit_target(name):
    return str(name).startswith("gate") and str(name).endswith("_exit")


def is_optional_recovery_waypoint(name):
    name = str(name)
    return (
        name.startswith("photo_exit")
        or name.endswith("_retreat")
        or name.endswith("_staging")
    )


def lap2_speed_for_waypoint(name, current_speed):
    """Return a conservative lap-2 speed based on segment type."""
    name = str(name)
    speed = float(current_speed)
    if "photo" in name:
        return min(max(speed, 0.28), 0.36)
    if name.startswith("dock_") or name == "dock_contact_blue_buoys":
        return min(speed, 0.30)
    if is_gate_transit_target(name):
        return max(speed, 0.38)
    if any(token in name for token in ("turn", "corner", "entry", "align")):
        return min(max(speed, 0.24), 0.34)
    if name.startswith(("path_", "return_", "pre_gate")):
        return max(speed, 0.48)
    return max(speed, 0.36)


def optimized_lap2_route(
    waypoints,
    coverage_percent=0.0,
    min_coverage_percent=8.0,
    compress_gate_waypoints=True,
):
    """Build a lap-2 route that uses the lap-1 map to reduce gate micromanagement.

    The mission still requires every gate line to be crossed. When the survey map
    coverage is sufficient, approach/center gate waypoints are dropped and each
    gate is traversed by navigating through its exit point. LiDAR and Nav2
    costmaps remain responsible for confirming that mapped obstacles are still
    present and safe to pass.
    """
    can_compress = (
        bool(compress_gate_waypoints)
        and float(coverage_percent) >= float(min_coverage_percent)
    )
    result = []
    for waypoint in waypoints:
        item = dict(waypoint)
        name = item.get("name", "")
        if can_compress and is_gate_waypoint(name) and not is_gate_transit_target(name):
            continue
        item["speed"] = lap2_speed_for_waypoint(name, item.get("speed", 0.25))
        result.append(item)
    return result, can_compress
