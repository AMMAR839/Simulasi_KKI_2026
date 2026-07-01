from asv_perception.preferred_lane_server import (
    segment_lane_half_width,
    waypoint_lane_half_width,
)


def test_gate_segment_uses_gate_width_even_when_connected_to_turn():
    assert segment_lane_half_width("top_left_turn", "gate3_approach", 1.4, 2.0) == 1.4
    assert segment_lane_half_width("gate3_exit", "path_1", 1.4, 2.0) == 1.4


def test_non_gate_turn_segment_uses_turn_width():
    assert segment_lane_half_width("corner_top_left", "top_left_turn", 1.4, 2.0) == 2.0
    assert waypoint_lane_half_width("gate3_approach", 1.4, 2.0) == 1.4
    assert waypoint_lane_half_width("corner_top_left", 1.4, 2.0) == 2.0
