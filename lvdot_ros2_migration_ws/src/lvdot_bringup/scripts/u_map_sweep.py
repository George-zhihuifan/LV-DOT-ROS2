#!/usr/bin/env python3
"""u_map parameter sweep.

For each u_map (threshold_point, threshold_line, min_length_line) combo, launches
the full detector + scene + evaluator stack, lets it run for `run_seconds`,
parses the evaluator CSV, and records cumulative TP/FP/FN/precision/recall/F1.

Result CSV columns: tp_pt, tp_ln, min_len, frames, tp, fp, fn, prec, rec, f1, mean_err_m.

Picks the (tp_pt, tp_ln, min_len) with highest F1 (ties broken by recall then
precision then lowest mean_err) and prints it.

Usage:
  python3 u_map_sweep.py \
    --grid 1,2,3 1,2,3 1,2 \
    --warmup 20 --collect 30 \
    --output /tmp/u_map_sweep.csv
"""
from __future__ import annotations

import argparse
import csv
import itertools
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def parse_csv(path: Path) -> tuple[int, int, int, int, float]:
    """Return (frames, tp, fp, fn, sum_err)."""
    if not path.exists():
        return 0, 0, 0, 0, 0.0
    frames = tp = fp = fn = 0
    err_sum = 0.0
    err_n = 0
    with path.open() as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            frames += 1
            tp += int(row["tp"])
            fp += int(row["fp"])
            fn += int(row["fn"])
            mean_err = row["mean_err_m"]
            if mean_err:
                err_sum += float(mean_err)
                err_n += 1
    avg_err = err_sum / err_n if err_n else float("nan")
    return frames, tp, fp, fn, avg_err


def run_one(args, tp_pt: int, tp_ln: int, min_len: int) -> dict:
    csv_out = Path(f"/tmp/u_map_sweep_{tp_pt}_{tp_ln}_{min_len}.csv")
    csv_out.unlink(missing_ok=True)
    log_out = Path(f"/tmp/u_map_sweep_{tp_pt}_{tp_ln}_{min_len}.log")

    cmd = [
        "ros2", "launch", "lvdot_bringup", "run_detector_with_scene.launch.py",
        f"enable_yolo:={'true' if args.enable_yolo else 'false'}",
        f"launch_yolo_node:={'true' if args.enable_yolo else 'false'}",
        "launch_pose_stub:=true",
        "pose_stub_orbit_enabled:=true",
        "launch_evaluator:=true",
        f"evaluator_csv_path:={csv_out}",
        f"evaluator_match_threshold_m:={args.match_threshold}",
        f"u_map_threshold_point:={tp_pt}",
        f"u_map_threshold_line:={tp_ln}",
        f"u_map_min_length_line:={min_len}",
        "executor_threads:=4",
        "gazebo_gui:=false",
        "detector_rviz:=false",
        "rviz:=false",
        "use_realistic_sensors:=false",
    ]

    print(f"[sweep] tp_pt={tp_pt} tp_ln={tp_ln} min_len={min_len}: launching")
    sys.stdout.flush()
    with log_out.open("w") as logfh:
        proc = subprocess.Popen(cmd, stdout=logfh, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    try:
        time.sleep(args.warmup + args.collect)
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()

    subprocess.run(
        "pkill -f 'ros2 launch|ign gazebo|lvdot_detector_main|pose_stub|"
        "parameter_bridge|pedestrian_state_publisher|detection_evaluator' || true",
        shell=True,
    )
    time.sleep(3)

    frames, tp, fp, fn, mean_err = parse_csv(csv_out)
    prec = tp / max(1, tp + fp)
    rec = tp / max(1, tp + fn)
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0.0
    res = dict(
        tp_pt=tp_pt, tp_ln=tp_ln, min_len=min_len,
        frames=frames, tp=tp, fp=fp, fn=fn,
        prec=prec, rec=rec, f1=f1, mean_err=mean_err,
    )
    print(f"  -> frames={frames} tp={tp} fp={fp} fn={fn} prec={prec:.3f} rec={rec:.3f} F1={f1:.3f} err={mean_err:.2f}m")
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", nargs=3, default=["1,2,3", "1,2,3", "1,2"],
                    metavar=("THRESH_POINT", "THRESH_LINE", "MIN_LEN"))
    ap.add_argument("--warmup", type=float, default=20.0)
    ap.add_argument("--collect", type=float, default=30.0)
    ap.add_argument("--match-threshold", type=float, default=2.5)
    ap.add_argument("--enable-yolo", action="store_true")
    ap.add_argument("--output", default="/tmp/u_map_sweep.csv")
    args = ap.parse_args()

    point_vals = [int(v) for v in args.grid[0].split(",")]
    line_vals = [int(v) for v in args.grid[1].split(",")]
    len_vals = [int(v) for v in args.grid[2].split(",")]

    results = []
    for tp_pt, tp_ln, min_len in itertools.product(point_vals, line_vals, len_vals):
        results.append(run_one(args, tp_pt, tp_ln, min_len))

    with open(args.output, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=[
            "tp_pt", "tp_ln", "min_len", "frames", "tp", "fp", "fn",
            "prec", "rec", "f1", "mean_err",
        ])
        w.writeheader()
        for r in results:
            w.writerow(r)

    results.sort(key=lambda r: (-r["f1"], -r["rec"], -r["prec"], r["mean_err"]))
    best = results[0]
    print("\n=== SWEEP RESULTS (sorted by F1 desc) ===")
    print(f"{'tp_pt':>5} {'tp_ln':>5} {'min_len':>7} {'frames':>6} {'TP':>5} {'FP':>5} {'FN':>5} {'prec':>6} {'rec':>6} {'F1':>6} {'err':>5}")
    for r in results:
        print(f"{r['tp_pt']:>5} {r['tp_ln']:>5} {r['min_len']:>7} {r['frames']:>6} "
              f"{r['tp']:>5} {r['fp']:>5} {r['fn']:>5} "
              f"{r['prec']:>6.3f} {r['rec']:>6.3f} {r['f1']:>6.3f} {r['mean_err']:>5.2f}")
    print(f"\n=== BEST: tp_pt={best['tp_pt']} tp_ln={best['tp_ln']} min_len={best['min_len']} "
          f"F1={best['f1']:.3f} rec={best['rec']:.3f} ===")
    print(f"CSV saved to {args.output}")


if __name__ == "__main__":
    main()
