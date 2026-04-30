import argparse
import csv
import math
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Export projected GT target metadata from the minimal experiment config.'
    )
    parser.add_argument('config_file', type=Path, help='Path to experiment_targets.yaml')
    parser.add_argument('output_csv', type=Path, help='Output CSV path')
    return parser.parse_args()


def project_target(target: dict, camera: dict) -> dict:
    x, y, z = target['center_xyz_m']
    camera_x, camera_y, camera_z = camera.get('position_xyz_m', [0.0, 0.0, 1.2])
    rel_x = x - float(camera_x)
    rel_y = y - float(camera_y)
    rel_z = z - float(camera_z)
    width = float(camera['image_width'])
    height = float(camera['image_height'])
    hfov = float(camera['horizontal_fov_rad'])
    focal_x = width / (2.0 * math.tan(hfov / 2.0))
    focal_y = focal_x
    cx = width / 2.0
    cy = height / 2.0

    u = cx - focal_x * (rel_y / rel_x)
    v = cy - focal_y * (rel_z / rel_x)

    size_x, _, size_z = target['size_xyz_m']
    roi_w = focal_x * (size_x / rel_x)
    roi_h = focal_y * (size_z / rel_x)
    gt_depth_reference = str(camera.get('gt_depth_reference', 'center')).strip().lower()
    center_depth_m = float(rel_x)
    front_surface_depth_m = float(rel_x - (size_x / 2.0))
    if gt_depth_reference == 'front_surface':
        gt_depth_m = front_surface_depth_m
    else:
        gt_depth_reference = 'center'
        gt_depth_m = center_depth_m

    return {
        'name': target['name'],
        'expected_range_group': target['expected_range_group'],
        'gt_depth_reference': gt_depth_reference,
        'gt_center_depth_m': round(center_depth_m, 4),
        'gt_front_surface_depth_m': round(front_surface_depth_m, 4),
        'gt_depth_m': round(gt_depth_m, 4),
        'center_u_px': round(u, 2),
        'center_v_px': round(v, 2),
        'roi_xmin_px': round(u - roi_w / 2.0, 2),
        'roi_ymin_px': round(v - roi_h / 2.0, 2),
        'roi_xmax_px': round(u + roi_w / 2.0, 2),
        'roi_ymax_px': round(v + roi_h / 2.0, 2),
    }


def main() -> None:
    args = parse_args()
    with args.config_file.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    rows = [project_target(target, config['camera']) for target in config['targets']]
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(f'Wrote {len(rows)} targets to {args.output_csv}')


if __name__ == '__main__':
    main()
