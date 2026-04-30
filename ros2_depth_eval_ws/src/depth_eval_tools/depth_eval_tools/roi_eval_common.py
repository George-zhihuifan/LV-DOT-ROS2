import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class RoiTarget:
    name: str
    expected_range_group: str
    gt_depth_reference: str
    gt_center_depth_m: float
    gt_front_surface_depth_m: float
    gt_depth_m: float
    roi_xmin_px: int
    roi_ymin_px: int
    roi_xmax_px: int
    roi_ymax_px: int


def load_targets(gt_csv: Path) -> list[RoiTarget]:
    targets: list[RoiTarget] = []
    with gt_csv.open('r', encoding='utf-8', newline='') as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            targets.append(
                RoiTarget(
                    name=row['name'],
                    expected_range_group=row['expected_range_group'],
                    gt_depth_reference=row.get('gt_depth_reference', 'center'),
                    gt_center_depth_m=float(row.get('gt_center_depth_m', row['gt_depth_m'])),
                    gt_front_surface_depth_m=float(row.get('gt_front_surface_depth_m', row['gt_depth_m'])),
                    gt_depth_m=float(row['gt_depth_m']),
                    roi_xmin_px=int(round(float(row['roi_xmin_px']))),
                    roi_ymin_px=int(round(float(row['roi_ymin_px']))),
                    roi_xmax_px=int(round(float(row['roi_xmax_px']))),
                    roi_ymax_px=int(round(float(row['roi_ymax_px']))),
                )
            )
    if not targets:
        raise SystemExit('GT CSV contains no targets.')
    return targets


def depth_image_to_array(msg) -> np.ndarray:
    if msg.encoding == '32FC1':
        array = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return array.astype(np.float32, copy=False)
    if msg.encoding == '16UC1':
        array = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return array.astype(np.float32) / 1000.0
    raise SystemExit(f'Unsupported depth encoding: {msg.encoding}')


def clamp_roi(xmin: int, ymin: int, xmax: int, ymax: int, width: int, height: int) -> tuple[int, int, int, int]:
    x0 = max(0, min(width - 1, xmin))
    y0 = max(0, min(height - 1, ymin))
    x1 = max(x0 + 1, min(width, xmax))
    y1 = max(y0 + 1, min(height, ymax))
    return x0, y0, x1, y1


def evaluate_roi(
    depth_m: np.ndarray,
    gt_depth_m: float,
    xmin: int,
    ymin: int,
    xmax: int,
    ymax: int,
    far_clip_m: float,
    tolerance_m: float,
) -> dict:
    height, width = depth_m.shape
    x0, y0, x1, y1 = clamp_roi(xmin, ymin, xmax, ymax, width, height)
    roi = depth_m[y0:y1, x0:x1]
    valid = np.isfinite(roi) & (roi > 0.0) & (roi < far_clip_m)
    support = valid & (np.abs(roi - gt_depth_m) <= tolerance_m)
    saturated = np.isfinite(roi) & (roi >= far_clip_m - 1e-3)

    cx = min(width - 1, max(0, int(round((x0 + x1 - 1) / 2.0))))
    cy = min(height - 1, max(0, int(round((y0 + y1 - 1) / 2.0))))
    center_depth = float(depth_m[cy, cx])
    center_valid = float(np.isfinite(center_depth) and 0.0 < center_depth < far_clip_m)

    valid_depths = roi[valid]
    abs_error_m = float(np.mean(np.abs(valid_depths - gt_depth_m))) if valid_depths.size else float('nan')
    roi_area = roi.size if roi.size else 1
    valid_count = int(valid.sum())
    if valid_depths.size:
        min_depth_m = float(np.min(valid_depths))
        p10_depth_m = float(np.percentile(valid_depths, 10))
        p25_depth_m = float(np.percentile(valid_depths, 25))
        p50_depth_m = float(np.percentile(valid_depths, 50))
        p75_depth_m = float(np.percentile(valid_depths, 75))
        median_depth_m = float(np.median(valid_depths))
    else:
        min_depth_m = float('nan')
        p10_depth_m = float('nan')
        p25_depth_m = float('nan')
        p50_depth_m = float('nan')
        p75_depth_m = float('nan')
        median_depth_m = float('nan')

    return {
        'roi_valid_ratio': float(valid_count / roi_area),
        'center_valid_ratio': center_valid,
        'foreground_support_ratio': float(support.sum() / roi_area),
        'foreground_purity_ratio': float(support.sum() / valid_count) if valid_count else 0.0,
        'abs_error_m': abs_error_m,
        'saturation_ratio': float(saturated.sum() / roi_area),
        'min_depth_m': min_depth_m,
        'p10_depth_m': p10_depth_m,
        'p25_depth_m': p25_depth_m,
        'p50_depth_m': p50_depth_m,
        'p75_depth_m': p75_depth_m,
        'median_depth_m': median_depth_m,
    }


def summarize(rows: list[dict]) -> dict:
    keys = [
        'roi_valid_ratio',
        'center_valid_ratio',
        'foreground_support_ratio',
        'foreground_purity_ratio',
        'abs_error_m',
        'saturation_ratio',
        'min_depth_m',
        'p10_depth_m',
        'p25_depth_m',
        'p50_depth_m',
        'p75_depth_m',
        'median_depth_m',
    ]
    summary = {'target_name': '__summary__', 'range_group': 'all', 'gt_depth_m': float('nan')}
    for key in keys:
        values = np.array([float(row[key]) for row in rows], dtype=float)
        summary[key] = float(np.nanmean(values))
    return summary
