import math
import re
from pathlib import Path

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

    def load_pgm(self, pgm_path: str) -> int:
        """Pre-seed the mapper from a saved PGM file.

        The PGM must use the same resolution and origin as this mapper.
        Pixel convention (ROS2 / nav2_map_server trinary mode):
          254  → free     (log_odds = -2.0, observed = True)
            0  → occupied (log_odds = +3.0, observed = True)
          205  → unknown  (log_odds =  0.0, observed = False)

        Returns the number of observed cells loaded (0 on failure).
        """
        path = Path(pgm_path)
        if not path.exists():
            return 0
        try:
            with path.open("rb") as f:
                raw = f.read()
            # Parse PGM header (P5 format, skip comment lines)
            header_end = 0
            tokens = []
            i = 0
            while len(tokens) < 4:
                if raw[i:i+1] == b"#":
                    while raw[i:i+1] not in (b"\n", b""):
                        i += 1
                    i += 1
                    continue
                match = re.match(rb"[^\S\n]*(\S+)", raw[i:])
                if match:
                    tokens.append(match.group(1).decode("ascii"))
                    i += match.end()
                else:
                    i += 1
            magic, width_s, height_s, maxval_s = tokens
            if magic != "P5":
                return 0
            pgm_w, pgm_h = int(width_s), int(height_s)
            # Pixel data starts after the header whitespace byte
            pixel_data = raw[i:]
            if i < len(raw) and raw[i:i+1] in (b" ", b"\t", b"\n", b"\r"):
                pixel_data = raw[i + 1:]
            img = np.frombuffer(pixel_data, dtype=np.uint8).reshape(pgm_h, pgm_w)
            if pgm_w != self.width or pgm_h != self.height:
                return 0
            # PGM was saved with np.flipud — restore to internal row-order
            img = np.flipud(img)
            free_mask = img == 254
            occ_mask  = img == 0
            self.log_odds[free_mask] = -2.0
            self.observed[free_mask] = True
            self.log_odds[occ_mask]  =  3.0
            self.observed[occ_mask]  = True
            return int(np.count_nonzero(self.observed))
        except Exception:
            return 0

    @property
    def coverage_percent(self):
        return 100.0 * float(np.count_nonzero(self.observed)) / float(
            self.observed.size
        )
