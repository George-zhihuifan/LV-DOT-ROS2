#!/usr/bin/env python3
"""Summarize LV-DOT ablation JSON outputs into markdown tables.

Supports multi-run aggregation: directories named run{N}_{scenario}_{group}
are grouped by (scenario, group) and reported as mean ± std across runs.
Also emits per-run raw tables when --per-run is given.
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


def fmt(value: object, precision: int = 3) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.{precision}f}"
    return str(value)


def fmt_mean_std(values: list[float], precision: int = 3) -> str:
    if not values:
        return ""
    m = statistics.mean(values)
    if len(values) >= 2:
        s = statistics.stdev(values)
        return f"{m:.{precision}f} ± {s:.{precision}f}"
    return f"{m:.{precision}f}"


def deep_get(d: dict, *keys: str) -> Any:
    for k in keys:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


_RUN_RE = re.compile(r"^run(\d+)_(.+)$")


def parse_run_dir(name: str) -> tuple[int | None, str]:
    """Return (run_index, scenario_group) or (None, name) if no run prefix."""
    m = _RUN_RE.match(name)
    if m:
        return int(m.group(1)), m.group(2)
    return None, name


def load_summaries(root: Path) -> list[tuple[str, dict]]:
    rows = []
    for summary_path in sorted(root.glob("*/summary.json")):
        try:
            rows.append((summary_path.parent.name, json.loads(summary_path.read_text(encoding="utf-8"))))
        except json.JSONDecodeError:
            continue
    return rows


def is_valid_run(summary: dict, min_frames: int = 10) -> bool:
    """Filter out degenerate runs (empty or crashed before collecting data)."""
    return summary.get("total_frames", 0) >= min_frames


def is_failed_run(
    summary: dict,
    scenario_group: str,
    *,
    min_frames: int,
    f1_threshold: float,
    recall_threshold: float,
) -> bool:
    """Heuristic for simulator-failed runs.

    Failed if:
    - F1@1m == 0
    - best_iou_max == 0
    - total_frames > 1000
    Exception:
    - A1_lidar_only is not filtered by this rule (legitimate near-zero baseline).
    """
    if "A1_lidar_only" in scenario_group:
        return False
    total_frames = deep_get(summary, "total_frames")
    f1_1m = deep_get(summary, "center_distance", "1.0m", "f1")
    recall_1m = deep_get(summary, "center_distance", "1.0m", "recall")
    best_iou_max = deep_get(summary, "iou_diagnostic", "best_iou_diagnostic", "max_best_iou")
    if not isinstance(total_frames, (int, float)):
        return False
    if not isinstance(f1_1m, (int, float)):
        return False
    if not isinstance(best_iou_max, (int, float)):
        return False
    if not isinstance(recall_1m, (int, float)):
        return False

    tf = float(total_frames)
    f1 = float(f1_1m)
    rec = float(recall_1m)
    iou = float(best_iou_max)

    # Strict zero-output failure.
    zero_fail = tf > min_frames and f1 == 0.0 and iou == 0.0
    # Practical collapsed-run failure: almost-zero detection quality on long runs.
    collapse_fail = tf > min_frames and f1 <= f1_threshold and rec <= recall_threshold

    return zero_fail or collapse_fail


def group_by_scenario_group(
    rows: list[tuple[str, dict]],
    filter_failed: bool = False,
    *,
    failed_min_frames: int = 1000,
    failed_f1_threshold: float = 0.05,
    failed_recall_threshold: float = 0.03,
) -> tuple[dict[str, list[dict]], int, list[tuple[str, str, float, float, float, float]]]:
    """Group summaries by (scenario_group) across run indices.

    Returns:
      (grouped_summaries, filtered_failed_count)
    """
    groups: dict[str, list[dict]] = defaultdict(list)
    filtered_failed = 0
    filtered_runs: list[tuple[str, str, float, float, float, float]] = []
    for name, summary in rows:
        if not is_valid_run(summary):
            continue
        _, sg = parse_run_dir(name)
        if filter_failed and is_failed_run(
            summary,
            sg,
            min_frames=failed_min_frames,
            f1_threshold=failed_f1_threshold,
            recall_threshold=failed_recall_threshold,
        ):
            filtered_failed += 1
            filtered_runs.append((
                name,
                sg,
                float(deep_get(summary, "total_frames") or 0.0),
                float(deep_get(summary, "center_distance", "1.0m", "f1") or 0.0),
                float(deep_get(summary, "center_distance", "1.0m", "recall") or 0.0),
                float(deep_get(summary, "iou_diagnostic", "best_iou_diagnostic", "max_best_iou") or 0.0),
            ))
            continue
        groups[sg].append(summary)
    return dict(groups), filtered_failed, filtered_runs


def collect_values(summaries: list[dict], *keys: str) -> list[float]:
    vals = []
    for s in summaries:
        v = deep_get(s, *keys)
        if isinstance(v, (int, float)) and v is not None:
            vals.append(float(v))
    return vals


def scenario_group_sort_key(sg: str) -> tuple[int, str]:
    """Sort by scenario first, then ablation group (A1 < A3 < A4 < A5 < A6)."""
    parts = sg.split("_", 1)
    if len(parts) == 2:
        scenario, group = parts[0], parts[1]
    else:
        scenario, group = "", sg
    group_order = {"A1": 0, "A3": 1, "A4": 2, "A5": 3, "A6": 4}
    prefix = group.split("_")[0] if group else ""
    return (0 if "dense" in scenario else 1, group_order.get(prefix, 99), sg)


# ---------------------------------------------------------------------------
# Table generators
# ---------------------------------------------------------------------------

def aggregated_tables(grouped: dict[str, list[dict]]) -> list[str]:
    lines = ["# Ablation Summary (Aggregated: mean ± std)", ""]
    sorted_keys = sorted(grouped.keys(), key=scenario_group_sort_key)

    # --- Center-Distance Detection ---
    lines.append("## Center-Distance Detection Metrics (Primary)")
    lines.append("")
    lines.append("| Config | N | P@0.5m | R@0.5m | F1@0.5m | P@1.0m | R@1.0m | F1@1.0m | Err@1m | P@1.5m | R@1.5m | F1@1.5m | P@2.0m | R@2.0m | F1@2.0m |")
    lines.append("|---|---:|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for sg in sorted_keys:
        sums = grouped[sg]
        n = len(sums)
        cells = [sg, str(n)]
        for thresh in ["0.5m", "1.0m", "1.5m", "2.0m"]:
            for metric in ["precision", "recall", "f1"]:
                cells.append(fmt_mean_std(collect_values(sums, "center_distance", thresh, metric)))
            if thresh == "1.0m":
                cells.append(fmt_mean_std(collect_values(sums, "center_distance", thresh, "mean_error_m")))
        lines.append("| " + " | ".join(cells) + " |")

    lines.extend(["", "## Visible-GT Detection Metrics", ""])
    lines.append("| Config | N | visGT/frame | visR@1.0m | visF1@1.0m | visR@1.5m | visF1@1.5m | visR@2.0m | visF1@2.0m |")
    lines.append("|---|---:|---|---|---|---|---|---|---|")
    for sg in sorted_keys:
        sums = grouped[sg]
        n = len(sums)
        cells = [sg, str(n)]
        cells.append(fmt_mean_std(collect_values(sums, "center_distance", "1.0m", "visible_gt_per_frame")))
        for thresh in ["1.0m", "1.5m", "2.0m"]:
            cells.append(fmt_mean_std(collect_values(sums, "center_distance", thresh, "visible_recall")))
            cells.append(fmt_mean_std(collect_values(sums, "center_distance", thresh, "visible_f1")))
        lines.append("| " + " | ".join(cells) + " |")

    # --- Tracking ---
    lines.extend(["", "## Tracking Metrics (center-distance@1.0m)", ""])
    lines.append("| Config | N | MOTA | IDF1 | IDSW | Frag | MT | ML |")
    lines.append("|---|---:|---|---|---|---|---|---|")
    for sg in sorted_keys:
        sums = grouped[sg]
        n = len(sums)
        cells = [sg, str(n)]
        for metric in ["mota", "idf1", "idsw", "frag", "mt", "ml"]:
            vals = collect_values(sums, "tracking", metric)
            if metric in ("idsw", "frag", "mt", "ml"):
                cells.append(fmt_mean_std(vals, precision=1))
            else:
                cells.append(fmt_mean_std(vals))
        lines.append("| " + " | ".join(cells) + " |")

    # --- GRU Prediction ---
    lines.extend(["", "## GRU Prediction Metrics", ""])
    lines.append("| Config | N | ADE@1s | ADE@2.5s | FDE | Samples |")
    lines.append("|---|---:|---|---|---|---|")
    for sg in sorted_keys:
        sums = grouped[sg]
        pred_vals = collect_values(sums, "prediction", "ade_1s")
        if not pred_vals:
            continue
        n = len(sums)
        cells = [sg, str(n)]
        for metric in ["ade_1s", "ade_2_5s", "fde"]:
            cells.append(fmt_mean_std(collect_values(sums, "prediction", metric)))
        cells.append(fmt_mean_std(collect_values(sums, "prediction", "samples"), precision=0))
        lines.append("| " + " | ".join(cells) + " |")

    # --- IoU Diagnostic ---
    lines.extend(["", "## 3D IoU Diagnostic", ""])
    lines.append("| Config | N | best_iou_max | mean_best_iou | IoU@0.3 F1 | IoU@0.5 F1 |")
    lines.append("|---|---:|---|---|---|---|")
    for sg in sorted_keys:
        sums = grouped[sg]
        n = len(sums)
        cells = [sg, str(n)]
        cells.append(fmt_mean_std(collect_values(sums, "iou_diagnostic", "best_iou_diagnostic", "max_best_iou")))
        cells.append(fmt_mean_std(collect_values(sums, "iou_diagnostic", "best_iou_diagnostic", "mean_best_iou_per_gt")))
        cells.append(fmt_mean_std(collect_values(sums, "iou_diagnostic", "iou_0.3", "f1")))
        cells.append(fmt_mean_std(collect_values(sums, "iou_diagnostic", "iou_0.5", "f1")))
        lines.append("| " + " | ".join(cells) + " |")

    return lines


def per_run_tables(rows: list[tuple[str, dict]]) -> list[str]:
    """Original per-run tables (no aggregation)."""
    lines = ["# Ablation Summary (Per-Run)", ""]

    lines.append("## Center-Distance Detection Metrics (Primary)")
    lines.append("")
    lines.append("| Run | Frames | P@0.5m | R@0.5m | F1@0.5m | P@1.0m | R@1.0m | F1@1.0m | Err@1m | P@1.5m | R@1.5m | F1@1.5m | P@2.0m | R@2.0m | F1@2.0m |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for name, summary in rows:
        cd = summary.get("center_distance", {})
        cd05 = cd.get("0.5m", {})
        cd10 = cd.get("1.0m", {})
        cd15 = cd.get("1.5m", {})
        cd20 = cd.get("2.0m", {})
        lines.append(
            "| "
            + " | ".join([
                name,
                fmt(summary.get("total_frames")),
                fmt(cd05.get("precision")), fmt(cd05.get("recall")), fmt(cd05.get("f1")),
                fmt(cd10.get("precision")), fmt(cd10.get("recall")), fmt(cd10.get("f1")),
                fmt(cd10.get("mean_error_m")),
                fmt(cd15.get("precision")), fmt(cd15.get("recall")), fmt(cd15.get("f1")),
                fmt(cd20.get("precision")), fmt(cd20.get("recall")), fmt(cd20.get("f1")),
            ])
            + " |"
        )

    lines.extend(["", "## Visible-GT Detection Metrics", ""])
    lines.append("| Run | visGT/frame | visR@1.0m | visF1@1.0m | visR@1.5m | visF1@1.5m | visR@2.0m | visF1@2.0m |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for name, summary in rows:
        cd = summary.get("center_distance", {})
        cd10 = cd.get("1.0m", {})
        cd15 = cd.get("1.5m", {})
        cd20 = cd.get("2.0m", {})
        lines.append(
            "| "
            + " | ".join([
                name,
                fmt(cd10.get("visible_gt_per_frame")),
                fmt(cd10.get("visible_recall")), fmt(cd10.get("visible_f1")),
                fmt(cd15.get("visible_recall")), fmt(cd15.get("visible_f1")),
                fmt(cd20.get("visible_recall")), fmt(cd20.get("visible_f1")),
            ])
            + " |"
        )

    lines.extend(["", "## Tracking Metrics (center-distance@1.0m)", ""])
    lines.append("| Run | MOTA | IDF1 | IDSW | Frag | MT | ML |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|")
    for name, summary in rows:
        t = summary.get("tracking", {})
        lines.append(
            "| "
            + " | ".join([
                name,
                fmt(t.get("mota")), fmt(t.get("idf1")), fmt(t.get("idsw")),
                fmt(t.get("frag")), fmt(t.get("mt")), fmt(t.get("ml")),
            ])
            + " |"
        )

    lines.extend(["", "## GRU Prediction Metrics", ""])
    lines.append("| Run | ADE@1s | ADE@2.5s | FDE | Samples |")
    lines.append("|---|---:|---:|---:|---:|")
    for name, summary in rows:
        p = summary.get("prediction", {})
        if not p:
            continue
        lines.append(
            "| "
            + " | ".join([
                name,
                fmt(p.get("ade_1s")), fmt(p.get("ade_2_5s")), fmt(p.get("fde")),
                fmt(p.get("samples")),
            ])
            + " |"
        )

    lines.extend(["", "## 3D IoU Diagnostic", ""])
    lines.append("| Run | best_iou_max | mean_best_iou | IoU@0.3 F1 | IoU@0.5 F1 |")
    lines.append("|---|---:|---:|---:|---:|")
    for name, summary in rows:
        iou = summary.get("iou_diagnostic", {})
        diag = iou.get("best_iou_diagnostic", {})
        i03 = iou.get("iou_0.3", {})
        i05 = iou.get("iou_0.5", {})
        lines.append(
            "| "
            + " | ".join([
                name,
                fmt(diag.get("max_best_iou")),
                fmt(diag.get("mean_best_iou_per_gt")),
                fmt(i03.get("f1")),
                fmt(i05.get("f1")),
            ])
            + " |"
        )

    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize LV-DOT ablation JSON outputs.")
    parser.add_argument("--root", required=True, help="Directory containing per-run summary.json files.")
    parser.add_argument("--output", default="", help="Markdown output path. Defaults to <root>/SUMMARY.md.")
    parser.add_argument("--per-run", action="store_true", help="Also emit per-run (non-aggregated) tables.")
    parser.add_argument("--filter-failed", action="store_true", help="Filter failed runs by heuristic (F1@1m=0, best_iou_max=0, total_frames>1000; except A1).")
    parser.add_argument("--failed-min-frames", type=int, default=1000, help="Min total_frames to consider a run for failed filtering.")
    parser.add_argument("--failed-f1-threshold", type=float, default=0.05, help="F1@1m threshold for collapsed-run filtering.")
    parser.add_argument("--failed-recall-threshold", type=float, default=0.03, help="Recall@1m threshold for collapsed-run filtering.")
    args = parser.parse_args()

    root = Path(args.root)
    output = Path(args.output) if args.output else root / "SUMMARY.md"
    rows = load_summaries(root)

    grouped, filtered_failed, filtered_runs = group_by_scenario_group(
        rows,
        filter_failed=args.filter_failed,
        failed_min_frames=args.failed_min_frames,
        failed_f1_threshold=args.failed_f1_threshold,
        failed_recall_threshold=args.failed_recall_threshold,
    )

    all_lines: list[str] = []

    all_lines.extend(aggregated_tables(grouped))
    all_lines.append("")
    all_lines.append(f"Aggregated {len(grouped)} configs from {len(rows)} runs in `{root}`.")
    if args.filter_failed:
        all_lines.append(
            "Filtered failed runs: "
            f"{filtered_failed} "
            f"(heuristic: total_frames>{args.failed_min_frames} and "
            f"((F1@1m=0 and best_iou_max=0) or (F1@1m<={args.failed_f1_threshold} and Recall@1m<={args.failed_recall_threshold})); "
            "A1 excluded)."
        )
        if filtered_runs:
            all_lines.extend(["", "Filtered run list:", "", "| Run | Config | Frames | F1@1m | R@1m | best_iou_max |", "|---|---|---:|---:|---:|---:|"])
            for run_name, sg, frames, f1, rec, iou in sorted(filtered_runs):
                all_lines.append(f"| {run_name} | {sg} | {frames:.0f} | {f1:.3f} | {rec:.3f} | {iou:.3f} |")

    if args.per_run:
        all_lines.extend(["", "---", ""])
        all_lines.extend(per_run_tables(rows))
        all_lines.append("")
        all_lines.append(f"Listed {len(rows)} individual runs.")

    output.write_text("\n".join(all_lines) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
