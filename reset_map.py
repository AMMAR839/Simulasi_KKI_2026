#!/usr/bin/env python3
import os
from pathlib import Path
import yaml
import numpy as np

def main():
    out_dir = Path('mission_maps/a')
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Create blank lap1_map
    lap1_yaml = {
        'image': 'lap1_map.pgm',
        'mode': 'trinary',
        'resolution': 0.1,
        'origin': [-30.0, -30.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }
    with open(out_dir / 'lap1_map.yaml', 'w') as f:
        yaml.safe_dump(lap1_yaml, f, sort_keys=False)

    # 600x600 map filled with 205 (unknown)
    lap1_img = np.full((600, 600), 205, dtype=np.uint8)
    with open(out_dir / 'lap1_map.pgm', 'wb') as f:
        f.write(b'P5\n600 600\n255\n')
        f.write(lap1_img.tobytes())

    # 2. Create blank speed_mask
    speed_yaml = {
        'image': 'speed_mask.pgm',
        'mode': 'scale',
        'resolution': 0.1,
        'origin': [-30.0, -30.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.35
    }
    with open(out_dir / 'speed_mask.yaml', 'w') as f:
        yaml.safe_dump(speed_yaml, f, sort_keys=False)

    # 600x600 speed mask filled with 0 (no limit)
    speed_img = np.zeros((600, 600), dtype=np.uint8)
    with open(out_dir / 'speed_mask.pgm', 'wb') as f:
        f.write(b'P5\n600 600\n255\n')
        f.write(speed_img.tobytes())

    # Delete old landmarks list
    landmarks_file = out_dir / 'landmarks.yaml'
    if landmarks_file.exists():
        landmarks_file.unlink()

    print('Peta dan Speed Mask telah direset ke KOSONG secara aman!')

if __name__ == '__main__':
    main()
