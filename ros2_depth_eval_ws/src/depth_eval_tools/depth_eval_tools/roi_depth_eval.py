import argparse
import csv
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Compute a minimal depth ROI quality summary from a CSV table.'
    )
    parser.add_argument(
        'csv_file',
        type=Path,
        help='CSV file containing ROI depth samples and GT columns.',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = []
    with args.csv_file.open('r', encoding='utf-8', newline='') as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)

    if not rows:
        raise SystemExit('Input CSV is empty.')

    roi_depth = np.array([float(row['roi_depth_m']) for row in rows], dtype=float)
    gt_depth = np.array([float(row['gt_depth_m']) for row in rows], dtype=float)
    center_valid = np.array([float(row.get('center_valid', '1')) for row in rows], dtype=float)
    foreground_support = np.array([float(row.get('foreground_support', '1')) for row in rows], dtype=float)
    foreground_purity = np.array([float(row.get('foreground_purity', '1')) for row in rows], dtype=float)
    valid_mask = np.isfinite(roi_depth) & (roi_depth > 0.0)
    error = np.abs(roi_depth[valid_mask] - gt_depth[valid_mask]) if np.any(valid_mask) else np.array([])

    summary = {
        'samples': int(len(rows)),
        'roi_valid_ratio': float(valid_mask.mean()),
        'center_valid_ratio': float(center_valid.mean()),
        'foreground_support_ratio': float(foreground_support.mean()),
        'foreground_purity_ratio': float(foreground_purity.mean()),
        'abs_error_m': float(error.mean()) if error.size else float('nan'),
        'saturation_ratio': float((roi_depth >= 11.99).mean()),
    }

    for key, value in summary.items():
        print(f'{key}: {value}')


if __name__ == '__main__':
    main()
