#!/usr/bin/env bash
set -euo pipefail

# A/B compare for GT eval:
# A = baseline detector_param.yaml
# B = tuned_v1 (detector_param_tuned_v1.yaml)
#
# Usage:
#   run_gt_eval_ab_compare.sh [rounds] [duration_sec] [gt_source]
#
# Example:
#   run_gt_eval_ab_compare.sh 5 60 agents

ROUNDS="${1:-5}"
DURATION_SEC="${2:-60}"
GT_SOURCE="${3:-agents}"

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
CFG_DIR="$WS_DIR/src/lvdot_bringup/config"
BASE_CFG="$CFG_DIR/detector_param.yaml"
TUNED_CFG="$CFG_DIR/detector_param_tuned_v1.yaml"
RUN_SUITE="$WS_DIR/src/lvdot_bringup/scripts/run_gt_eval_suite.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
AB_DIR="$WS_DIR/reports/gt_eval_ab_${STAMP}"
mkdir -p "$AB_DIR"

BASE_BAK="$AB_DIR/detector_param.baseline.bak.yaml"
cp "$BASE_CFG" "$BASE_BAK"

restore_base() {
  cp "$BASE_BAK" "$BASE_CFG"
}
trap restore_base EXIT

echo "[AB] running baseline..."
"$RUN_SUITE" "$ROUNDS" "$DURATION_SEC" "$GT_SOURCE"
BASE_SUITE_DIR="$(ls -dt "$WS_DIR"/reports/gt_eval_suite_* | head -n 1)"
cp -r "$BASE_SUITE_DIR" "$AB_DIR/baseline"

echo "[AB] switching to tuned_v1..."
cp "$TUNED_CFG" "$BASE_CFG"

echo "[AB] running tuned_v1..."
"$RUN_SUITE" "$ROUNDS" "$DURATION_SEC" "$GT_SOURCE"
TUNED_SUITE_DIR="$(ls -dt "$WS_DIR"/reports/gt_eval_suite_* | head -n 1)"
cp -r "$TUNED_SUITE_DIR" "$AB_DIR/tuned_v1"

echo "[AB] restoring baseline config..."
restore_base

python3 - "$AB_DIR" <<'PY'
import os
import re
import sys

ab_dir = sys.argv[1]
base = os.path.join(ab_dir, "baseline", "summary.txt")
tuned = os.path.join(ab_dir, "tuned_v1", "summary.txt")
out_md = os.path.join(ab_dir, "ab_compare.md")

def load(path):
    d = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if "=" not in line:
                continue
            k, v = line.split("=", 1)
            d[k] = v
    return d

def f(d, key, default=0.0):
    try:
        v = d.get(key, str(default))
        if " std=" in v:
            return float(v.split(" std=")[0])
        return float(v)
    except Exception:
        return default

b = load(base)
t = load(tuned)

metrics = [
    ("tracked_recall_mean", "Tracked Recall"),
    ("tracked_err_m_mean", "Tracked Center Err (m)"),
    ("fused_recall_mean", "Fused Recall"),
    ("fused_err_m_mean", "Fused Center Err (m)"),
    ("lidar_recall_mean", "LiDAR Recall"),
    ("yolo2d_recall_mean", "YOLO 2D Recall"),
    ("skew_warn_avg", "Skew Warn Avg"),
]

lines = []
lines.append("# GT Eval A/B Compare")
lines.append("")
lines.append(f"- baseline: `{base}`")
lines.append(f"- tuned_v1: `{tuned}`")
lines.append("")
lines.append("| Metric | Baseline | Tuned V1 | Delta (Tuned-Baseline) |")
lines.append("|---|---:|---:|---:|")
for k, name in metrics:
    bv = f(b, k)
    tv = f(t, k)
    dv = tv - bv
    lines.append(f"| {name} | {bv:.4f} | {tv:.4f} | {dv:+.4f} |")

lines.append("")
lines.append("## Rule-of-Thumb Verdict")
tr_b = f(b, "tracked_recall_mean")
tr_t = f(t, "tracked_recall_mean")
se_b = f(b, "skew_warn_avg")
se_t = f(t, "skew_warn_avg")
if tr_t > tr_b and se_t <= se_b:
    lines.append("- tuned_v1 improves tracked recall while not increasing skew warnings.")
else:
    lines.append("- tuned_v1 does not clearly dominate baseline; inspect per-run logs before adopting.")

with open(out_md, "w", encoding="utf-8") as fobj:
    fobj.write("\n".join(lines) + "\n")

print(out_md)
PY

echo "[AB] done. output: $AB_DIR"

