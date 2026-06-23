#!/usr/bin/env python3
"""Analyze z-anchor oracle upper bounds from advanced evaluator matched-pairs CSV."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def std(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def load_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                gt_x = float(row["gt_x"])
                gt_y = float(row["gt_y"])
                gt_z = float(row["gt_z"])
                det_x = float(row["det_x"])
                det_y = float(row["det_y"])
                det_z = float(row["det_z"])
                z_centroid = float(row["z_centroid"])
                z_foot_lift = float(row["z_foot_lift"])
                z_head_drop = float(row["z_head_drop"])
            except (KeyError, ValueError):
                continue
            rows.append(
                {
                    "gt_x": gt_x,
                    "gt_y": gt_y,
                    "gt_z": gt_z,
                    "det_x": det_x,
                    "det_y": det_y,
                    "det_z": det_z,
                    "z_centroid": z_centroid,
                    "z_foot_lift": z_foot_lift,
                    "z_head_drop": z_head_drop,
                    "dxy": math.hypot(gt_x - det_x, gt_y - det_y),
                    "range_m": float(row.get("gt_range_m", "nan")),
                    "visible": float(row.get("gt_visible", "0")),
                }
            )
    return rows


def candidate_error(row: dict[str, float], z_value: float) -> tuple[float, float]:
    dz = row["gt_z"] - z_value
    dist3d = math.sqrt(row["dxy"] ** 2 + dz ** 2)
    return dz, dist3d


def summarize(name: str, rows: list[dict[str, float]], z_selector) -> dict[str, float]:
    dzs: list[float] = []
    abs_dzs: list[float] = []
    dists: list[float] = []
    f1_like_hits = 0
    for row in rows:
        z_value = z_selector(row)
        dz, dist3d = candidate_error(row, z_value)
        dzs.append(dz)
        abs_dzs.append(abs(dz))
        dists.append(dist3d)
        if dist3d <= 1.0:
            f1_like_hits += 1
    n = len(rows)
    return {
        "name": name,
        "n": n,
        "mean_dz": mean(dzs),
        "mean_abs_dz": mean(abs_dzs),
        "mean_dist3d": mean(dists),
        "std_dist3d": std(dists),
        "hit_rate_1m_if_xy_fixed": f1_like_hits / n if n else float("nan"),
    }


def format_summary(summary: dict[str, float]) -> str:
    return (
        f"{summary['name']}: "
        f"n={int(summary['n'])}, "
        f"mean_dz={summary['mean_dz']:.3f}, "
        f"mean_abs_dz={summary['mean_abs_dz']:.3f}, "
        f"mean_dist3d={summary['mean_dist3d']:.3f}, "
        f"std_dist3d={summary['std_dist3d']:.3f}, "
        f"hit@1m_if_xy_fixed={summary['hit_rate_1m_if_xy_fixed']:.3f}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path, help="matched_pairs CSV exported by advanced_evaluator")
    args = ap.parse_args()

    rows = load_rows(args.csv)
    if not rows:
        raise SystemExit(f"No valid rows found in {args.csv}")

    summaries = [
        summarize("centroid", rows, lambda r: r["z_centroid"]),
        summarize("foot_lift", rows, lambda r: r["z_foot_lift"]),
        summarize("head_drop", rows, lambda r: r["z_head_drop"]),
        summarize(
            "oracle_best_single_anchor",
            rows,
            lambda r: min(
                [r["z_centroid"], r["z_foot_lift"], r["z_head_drop"]],
                key=lambda z: abs(r["gt_z"] - z),
            ),
        ),
        summarize(
            "oracle_avg_foot_head",
            rows,
            lambda r: 0.5 * (r["z_foot_lift"] + r["z_head_drop"]),
        ),
    ]

    print(f"rows={len(rows)} source={args.csv}")
    for summary in summaries:
        print(format_summary(summary))


if __name__ == "__main__":
    main()
