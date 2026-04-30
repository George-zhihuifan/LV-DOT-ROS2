import numpy as np


def valid_mask(depth_m: np.ndarray, far_clip_m: float) -> np.ndarray:
    finite = np.isfinite(depth_m)
    positive = finite & (depth_m > 0.0)
    return positive & (depth_m < far_clip_m)


def crop_center(depth_m: np.ndarray, fraction: float) -> np.ndarray:
    height, width = depth_m.shape
    crop_h = max(1, int(round(height * fraction)))
    crop_w = max(1, int(round(width * fraction)))
    y0 = max(0, (height - crop_h) // 2)
    x0 = max(0, (width - crop_w) // 2)
    y1 = min(height, y0 + crop_h)
    x1 = min(width, x0 + crop_w)
    return depth_m[y0:y1, x0:x1]


def ratio_valid(depth_m: np.ndarray, far_clip_m: float) -> float:
    mask = valid_mask(depth_m, far_clip_m)
    return float(mask.mean())


def summarize_depth_distribution(depth_m: np.ndarray, far_clip_m: float) -> dict:
    finite = np.isfinite(depth_m)
    positive = finite & (depth_m > 0.0)
    valid = positive & (depth_m < far_clip_m)
    saturated = finite & (depth_m >= far_clip_m - 1e-3)
    valid_depths = depth_m[valid]

    if valid_depths.size:
        min_depth_m = float(np.min(valid_depths))
        p10_depth_m = float(np.percentile(valid_depths, 10))
        p50_depth_m = float(np.percentile(valid_depths, 50))
        p90_depth_m = float(np.percentile(valid_depths, 90))
        max_depth_m = float(np.max(valid_depths))
    else:
        min_depth_m = float('nan')
        p10_depth_m = float('nan')
        p50_depth_m = float('nan')
        p90_depth_m = float('nan')
        max_depth_m = float('nan')

    center50 = crop_center(depth_m, 0.5)
    center25 = crop_center(depth_m, 0.25)

    return {
        'finite_ratio': float(finite.mean()),
        'positive_ratio': float(positive.mean()),
        'valid_ratio': float(valid.mean()),
        'center50_valid_ratio': ratio_valid(center50, far_clip_m),
        'center25_valid_ratio': ratio_valid(center25, far_clip_m),
        'saturation_ratio': float(saturated.mean()),
        'min_depth_m': min_depth_m,
        'p10_depth_m': p10_depth_m,
        'p50_depth_m': p50_depth_m,
        'p90_depth_m': p90_depth_m,
        'max_depth_m': max_depth_m,
    }
