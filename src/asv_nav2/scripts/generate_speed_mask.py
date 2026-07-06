#!/usr/bin/env python3
"""generate_speed_mask.py — Generate a Nav2 SpeedFilter mask for the KKI track.

Speed encoding (percentage-based, multiplier=-1.0, base=100.0):
  speed_limit_percent = (mask_value * MULTIPLIER) + BASE
  mask_value = (BASE - speed_limit_percent)

Zones (value → speed limit %):
  0  → 100% (no limit, full speed ~0.62 m/s)
  20 → 80%  (~0.50 m/s) general navigation
  40 → 60%  (~0.37 m/s) corners/turns
  45 → 55%  (~0.34 m/s) gate outer approach
  55 → 45%  (~0.28 m/s) gate center
  70 → 30%  (~0.19 m/s) docking

Usage:
    python3 generate_speed_mask.py --course a
    python3 generate_speed_mask.py --course b
"""

import argparse
from pathlib import Path

import numpy as np
import yaml

MULTIPLIER = -1.0
BASE = 100.0

GATE_ZONES = [
    (45, 3.0),
    (50, 1.5),
    (55, 0.8),
]
CORNER_SPEED_PCT = 40
CORNER_RADIUS_M  = 2.5
PHOTO_SPEED_PCT  = 45
PHOTO_RADIUS_M   = 2.0
DOCK_SPEED_PCT   = 70
DOCK_RADIUS_M    = 2.5


def pct_to_mask(pct: int) -> int:
    return int(round((BASE - pct) / (-MULTIPLIER)))


def save_pgm(img: np.ndarray, path: Path):
    h, w = img.shape
    with path.open("wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(np.flipud(img).tobytes())


def save_yaml(path: Path, pgm_name: str, resolution: float, origin: list):
    meta = {
        "image": pgm_name,
        "mode": "scale",
        "resolution": resolution,
        "origin": origin,
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.35,
    }
    path.write_text(yaml.safe_dump(meta, sort_keys=False))


def generate(course: str, ws_root: Path):
    map_dir     = ws_root / "mission_maps" / course
    survey_yaml = map_dir / "lap1_map.yaml"
    wp_yaml     = ws_root / "src/asv_navigation/config" / f"kki_waypoints_{course}.yaml"
    out_pgm     = map_dir / "speed_mask.pgm"
    out_yaml    = map_dir / "speed_mask.yaml"

    if not survey_yaml.exists():
        raise FileNotFoundError(f"Survey map not found: {survey_yaml}")
    if not wp_yaml.exists():
        raise FileNotFoundError(f"Waypoints not found: {wp_yaml}")

    with survey_yaml.open() as f:
        sm = yaml.safe_load(f)

    resolution = float(sm.get("resolution", 0.10))
    origin     = sm.get("origin", [-30.0, -30.0, 0.0])
    ox, oy     = float(origin[0]), float(origin[1])

    pgm_path = map_dir / sm["image"]
    with pgm_path.open("rb") as f:
        raw = f.read()
    tokens = []
    i = 0
    while len(tokens) < 4:
        import re
        m = re.match(rb"[^\S\n]*(\S+)", raw[i:])
        if m:
            tokens.append(m.group(1).decode("ascii"))
            i += m.end()
        else:
            i += 1
    pgm_w, pgm_h = int(tokens[1]), int(tokens[2])
    print(f"[speed_mask] Map {pgm_w}x{pgm_h} px, res={resolution} m/px, origin=({ox},{oy})")

    mask = np.zeros((pgm_h, pgm_w), dtype=np.uint8)

    with wp_yaml.open() as f:
        data = yaml.safe_load(f)
    waypoints = data.get("waypoints", [])

    def paint(cx, cy, radius_m, value):
        radius_px = max(1, int(radius_m / resolution))
        col_c = int((cx - ox) / resolution)
        row_c = int((cy - oy) / resolution)
        for dr in range(-radius_px - 1, radius_px + 2):
            for dc in range(-radius_px - 1, radius_px + 2):
                if dc*dc + dr*dr <= radius_px*radius_px:
                    r, c = row_c + dr, col_c + dc
                    if 0 <= r < pgm_h and 0 <= c < pgm_w:
                        mask[r, c] = max(mask[r, c], value)

    for wp in waypoints:
        name = wp.get("name", "")
        x, y = float(wp["x"]), float(wp["y"])

        if "gate" in name and "center" in name:
            for speed_pct, radius in GATE_ZONES:
                paint(x, y, radius, pct_to_mask(speed_pct))
        elif any(k in name for k in ("corner", "turn", "top_right", "top_left")):
            paint(x, y, CORNER_RADIUS_M, pct_to_mask(CORNER_SPEED_PCT))
        elif any(k in name for k in ("photo", "green_box", "blue_box")):
            paint(x, y, PHOTO_RADIUS_M, pct_to_mask(PHOTO_SPEED_PCT))
        elif "dock" in name:
            paint(x, y, DOCK_RADIUS_M, pct_to_mask(DOCK_SPEED_PCT))

    save_pgm(mask, out_pgm)
    save_yaml(out_yaml, out_pgm.name, resolution, origin)

    constrained = int(np.count_nonzero(mask))
    total = pgm_w * pgm_h
    print(f"[speed_mask] Speed-limited area: {constrained}/{total} px ({100.0*constrained/total:.1f}%)")
    for v in np.unique(mask[mask > 0]):
        pct = BASE + v * MULTIPLIER
        count = int(np.sum(mask == v))
        print(f"  mask={v:3d} -> speed<={pct:.0f}% ({pct*0.62/100:.2f} m/s) [{count} px]")

    print(f"[speed_mask] -> {out_pgm}")
    print(f"[speed_mask] -> {out_yaml}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--course", default="a", choices=["a","b"])
    ap.add_argument("--ws", default=str(Path(__file__).resolve().parents[4]))
    args = ap.parse_args()
    generate(args.course, Path(args.ws))


if __name__ == "__main__":
    main()
