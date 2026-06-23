#!/usr/bin/env python3
import argparse
import csv
from collections import defaultdict
from pathlib import Path


def load_pairs(path: Path):
    rows = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "stamp_sec": float(row["stamp_sec"]),
                    "gt_id": row["gt_id"],
                    "dist3d": float(row["dist3d"]),
                    "visible": int(row["gt_visible"]),
                }
            )
    return rows


def group_by_stamp(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["stamp_sec"]].append(row)
    ordered = sorted(grouped.items())
    return ordered


def summarize_offset(stamps, offset_sec, threshold_m):
    tp = 0
    fp = 0
    fn = 0
    for stamp, rows in stamps:
        target = stamp - offset_sec
        nearest_stamp, nearest_rows = min(stamps, key=lambda item: abs(item[0] - target))
        gt_ids = {r["gt_id"] for r in rows}
        matched_gt = set()
        det_count = len(nearest_rows)
        for pair in nearest_rows:
            if pair["dist3d"] <= threshold_m and pair["gt_id"] in gt_ids:
                matched_gt.add(pair["gt_id"])
        frame_tp = len(matched_gt)
        tp += frame_tp
        fp += max(0, det_count - frame_tp)
        fn += max(0, len(gt_ids) - frame_tp)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = (2 * precision * recall / (precision + recall)) if (precision + recall) else 0.0
    return precision, recall, f1, tp, fp, fn


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pairs", required=True)
    parser.add_argument("--threshold-m", type=float, default=1.0)
    parser.add_argument("--offset-start", type=float, default=0.0)
    parser.add_argument("--offset-stop", type=float, default=0.5)
    parser.add_argument("--offset-step", type=float, default=0.02)
    args = parser.parse_args()

    rows = load_pairs(Path(args.pairs))
    stamps = group_by_stamp(rows)

    best = None
    offset = args.offset_start
    print("offset_sec,precision,recall,f1,tp,fp,fn")
    while offset <= args.offset_stop + 1e-9:
        precision, recall, f1, tp, fp, fn = summarize_offset(stamps, offset, args.threshold_m)
        print(f"{offset:.3f},{precision:.6f},{recall:.6f},{f1:.6f},{tp},{fp},{fn}")
        if best is None or f1 > best[2]:
            best = (offset, precision, f1, recall, tp, fp, fn)
        offset += args.offset_step

    if best is not None:
        print(
            f"\nBEST offset={best[0]:.3f}s precision={best[1]:.6f} "
            f"recall={best[3]:.6f} f1={best[2]:.6f} tp={best[4]} fp={best[5]} fn={best[6]}"
        )


if __name__ == "__main__":
    main()
