import numpy as np

from asv_perception.mapping_core import OccupancyGridMapper, bresenham


def test_bresenham_includes_both_endpoints():
    points = bresenham(0, 0, 4, 2)
    assert points[0] == (0, 0)
    assert points[-1] == (4, 2)


def test_ray_marks_free_and_occupied_cells():
    mapper = OccupancyGridMapper(
        width_m=10.0,
        height_m=10.0,
        resolution=1.0,
        origin_x=-5.0,
        origin_y=-5.0,
    )
    mapper.integrate_ray(0.0, 0.0, 3.0, 0.0, hit=True)
    values = mapper.occupancy_values()

    start_x, start_y = mapper.world_to_cell(0.0, 0.0)
    end_x, end_y = mapper.world_to_cell(3.0, 0.0)
    assert 0 <= values[start_y, start_x] < 50
    assert values[end_y, end_x] > 50
    assert values[0, 0] == -1
    assert mapper.coverage_percent > 0.0


def test_non_hit_ray_does_not_create_obstacle():
    mapper = OccupancyGridMapper(
        width_m=20.0,
        height_m=20.0,
        resolution=1.0,
        origin_x=-10.0,
        origin_y=-10.0,
    )
    mapper.integrate_ray(0.0, 0.0, 6.0, 0.0, hit=False)
    values = mapper.occupancy_values()
    observed = values[values >= 0]
    assert observed.size > 0
    assert np.all(observed < 50)
