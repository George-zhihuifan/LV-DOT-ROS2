#!/usr/bin/env bash
set -euo pipefail

# Run paper-oriented tracking experiments:
# 1) Adaptive similarity A/B (adaptive=false vs adaptive=true), each N rounds
# 2) Final main run on selected config, M rounds
#
# Usage:
#   run_tracking_paper_suite.sh [ab_rounds] [ab_duration] [main_rounds] [main_duration] [gt_source]
#
# Example:
#   run_tracking_paper_suite.sh 5 60 10 60 agents

AB_ROUNDS="${1:-5}"
AB_DURATION="${2:-60}"
MAIN_ROUNDS="${3:-10}"
MAIN_DURATION="${4:-60}"
GT_SOURCE="${5:-agents}"

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
CFG_DIR="$WS_DIR/src/lvdot_bringup/config"
BASE_CFG="$CFG_DIR/detector_param.yaml"
TUNED_CFG="$CFG_DIR/detector_param_tuned_v1.yaml"
RUN_SUITE="$WS_DIR/src/lvdot_bringup/scripts/run_gt_eval_suite.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/reports/tracking_paper_suite_${STAMP}"
mkdir -p "$OUT_DIR"

BACKUP_CFG="$OUT_DIR/detector_param.backup.yaml"
cp "$BASE_CFG" "$BACKUP_CFG"

ADAPT_OFF_CFG="$OUT_DIR/detector_param_tuned_v1_adapt_off.yaml"
ADAPT_ON_CFG="$OUT_DIR/detector_param_tuned_v1_adapt_on.yaml"
cp "$TUNED_CFG" "$ADAPT_OFF_CFG"
cp "$TUNED_CFG" "$ADAPT_ON_CFG"

python3 - "$ADAPT_OFF_CFG" "$ADAPT_ON_CFG" <<'PY'
import sys

def rewrite(path, target):
    lines = []
    found = False
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.strip().startswith('adaptive_similarity_weight:'):
                indent = line[:len(line)-len(line.lstrip())]
                lines.append(f"{indent}adaptive_similarity_weight: {'true' if target else 'false'}\n")
                found = True
            else:
                lines.append(line)
    if not found:
        lines.append(f"    adaptive_similarity_weight: {'true' if target else 'false'}\n")
    with open(path, 'w', encoding='utf-8') as f:
        f.writelines(lines)

rewrite(sys.argv[1], False)
rewrite(sys.argv[2], True)
PY

restore_cfg() {
  cp "$BACKUP_CFG" "$BASE_CFG"
}
trap restore_cfg EXIT

cleanup_all() {
  pkill -9 -f 'run_detector_with_scene.launch.py|run_full_stack_qcgaf_gru.launch.py|uav_pedestrian_prototype.launch.py|qcgaf_fusion_node|gru_prediction_node|lvdot_detector_main|lvdot_yolo_node|parameter_bridge|rviz2|gz sim|ros2 launch|gt_detection_publisher|image_pointcloud_relay' >/dev/null 2>&1 || true
  sleep 2
}

copy_latest_suite() {
  local tag="$1"
  local latest
  latest="$(ls -dt "$WS_DIR"/reports/gt_eval_suite_* | head -n 1)"
  cp -r "$latest" "$OUT_DIR/$tag"
}

echo "[paper_suite] strict cleanup"
cleanup_all

echo "[paper_suite] A/B part A: adaptive=false"
cp "$ADAPT_OFF_CFG" "$BASE_CFG"
"$RUN_SUITE" "$AB_ROUNDS" "$AB_DURATION" "$GT_SOURCE"
copy_latest_suite "adaptive_off"

echo "[paper_suite] strict cleanup"
cleanup_all

echo "[paper_suite] A/B part B: adaptive=true"
cp "$ADAPT_ON_CFG" "$BASE_CFG"
"$RUN_SUITE" "$AB_ROUNDS" "$AB_DURATION" "$GT_SOURCE"
copy_latest_suite "adaptive_on"

echo "[paper_suite] strict cleanup"
cleanup_all

echo "[paper_suite] Main run: tuned_v1 adaptive=true"
cp "$ADAPT_ON_CFG" "$BASE_CFG"
"$RUN_SUITE" "$MAIN_ROUNDS" "$MAIN_DURATION" "$GT_SOURCE"
copy_latest_suite "main_10round"

python3 - "$OUT_DIR" <<'PY'
import os, re, sys
root = sys.argv[1]

def load_summary_txt(dirpath):
    p = os.path.join(dirpath, 'summary.txt')
    d = {}
    with open(p, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if '=' in line:
                k, v = line.split('=', 1)
                d[k] = v
    return d, p

def f(d, k, default=0.0):
    v = d.get(k, str(default))
    if ' std=' in v:
        v = v.split(' std=')[0]
    try:
        return float(v)
    except Exception:
        return default

off_dir = os.path.join(root, 'adaptive_off')
on_dir = os.path.join(root, 'adaptive_on')
main_dir = os.path.join(root, 'main_10round')

off, offp = load_summary_txt(off_dir)
on, onp = load_summary_txt(on_dir)
main, mainp = load_summary_txt(main_dir)

md = []
md.append('# Tracking Paper Suite Report')
md.append('')
md.append('## Configs')
md.append(f'- adaptive_off: `{offp}`')
md.append(f'- adaptive_on: `{onp}`')
md.append(f'- main_10round: `{mainp}`')
md.append('')
md.append('## A/B (adaptive true - false)')
md.append('')
md.append('| Metric | adaptive=false | adaptive=true | Delta |')
md.append('|---|---:|---:|---:|')
for k, name in [
    ('tracked_recall_mean', 'Tracked Recall'),
    ('tracked_err_m_mean', 'Tracked Center Err (m)'),
    ('fused_recall_mean', 'Fused Recall'),
    ('fused_err_m_mean', 'Fused Center Err (m)'),
    ('lidar_recall_mean', 'LiDAR Recall'),
    ('yolo2d_recall_mean', 'YOLO 2D Recall'),
    ('skew_warn_avg', 'Skew Warn Avg'),
]:
    a = f(off, k)
    b = f(on, k)
    md.append(f'| {name} | {a:.4f} | {b:.4f} | {b-a:+.4f} |')

md.append('')
md.append('## Main 10-round (adaptive=true)')
md.append('')
for k in ['uv','db','lidar','fused','tracked']:
    md.append(f"- {k}: recall={f(main, k+'_recall_mean'):.4f}, err_m={f(main, k+'_err_m_mean'):.4f}")
md.append(f"- yolo_2d recall={f(main, 'yolo2d_recall_mean'):.4f}")
md.append(f"- skew_warn_avg={f(main, 'skew_warn_avg'):.4f}")

out = os.path.join(root, 'paper_suite_report.md')
with open(out, 'w', encoding='utf-8') as fp:
    fp.write('\n'.join(md) + '\n')
print(out)
PY

restore_cfg
cleanup_all

echo "[paper_suite] done: $OUT_DIR"
