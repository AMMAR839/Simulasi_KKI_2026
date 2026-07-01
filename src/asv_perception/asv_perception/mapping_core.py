import math

import numpy as np


def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return points
        twice = 2 * error
        if twice > -dy:
            error -= dy
            x0 += sx
        if twice < dx:
            error += dx
            y0 += sy


class OccupancyGridMapper:
    def __init__(
        self,
        width_m=60.0,
        height_m=60.0,
        resolution=0.10,
        origin_x=-30.0,
        origin_y=-30.0,
    ):
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.observed = np.zeros((self.height, self.width), dtype=np.bool_)

    def world_to_cell(self, x, y):
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def inside(self, cell_x, cell_y):
        return 0 <= cell_x < self.width and 0 <= cell_y < self.height

    def integrate_ray(self, sensor_x, sensor_y, end_x, end_y, hit=True):
        start = self.world_to_cell(sensor_x, sensor_y)
        end = self.world_to_cell(end_x, end_y)
        points = bresenham(start[0], start[1], end[0], end[1])
        if not points:
            return
        free_points = points[:-1] if hit else points
        for cell_x, cell_y in free_points:
            if not self.inside(cell_x, cell_y):
                continue
            self.observed[cell_y, cell_x] = True
            self.log_odds[cell_y, cell_x] = max(
                -4.0, self.log_odds[cell_y, cell_x] - 0.35
            )
        if hit:
            cell_x, cell_y = points[-1]
            if self.inside(cell_x, cell_y):
                self.observed[cell_y, cell_x] = True
                self.log_odds[cell_y, cell_x] = min(
                    4.0, self.log_odds[cell_y, cell_x] + 0.85
                )

    def occupancy_values(self):
        probability = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        result = np.full(self.log_odds.shape, -1, dtype=np.int8)
        result[self.observed] = np.clip(
            np.rint(probability[self.observed] * 100.0), 0, 100
        ).astype(np.int8)
        return result

    @property
    def coverage_percent(self):
        return 100.0 * float(np.count_nonzero(self.observed)) / float(
            self.observed.size
        )
