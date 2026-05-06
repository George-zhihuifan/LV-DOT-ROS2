#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   run_gt_eval_suite.sh [rounds] [duration_sec] [gt_source]
#
# Example:
#   run_gt_eval_suite.sh 5 60 agents

ROUNDS="${1:-5}"
DURATION_SEC="${2:-60}"
GT_SOURCE="${3:-agents}"   # agents | pose_info

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
REPORT_ROOT="$WS_DIR/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$REPORT_ROOT/gt_eval_suite_${STAMP}"
mkdir -p "$OUT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

cleanup_all() {
  killall -9 rviz2 gz parameter_bridge pedestrian_state_publisher \
    lvdot_detector_main lvdot_yolo_node lvdot_pose_stub python3 2>/dev/null || true
  sleep 1
}

for ((i=1; i<=ROUNDS; i++)); do
  echo "[suite] run ${i}/${ROUNDS}: cleanup"
  cleanup_all

  LAUNCH_LOG="$OUT_DIR/launch_run${i}.log"
  EVAL_LOG="$OUT_DIR/eval_run${i}.txt"

  echo "[suite] run ${i}/${ROUNDS}: launch scene+detector"
  nohup ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
    gazebo_gui:=false rviz:=false detector_rviz:=false \
    enable_uav_controller:=false launch_pose_stub:=true \
    enable_yolo:=true launch_yolo_node:=true fusion_mode:=dual \
    enable_stage_timers:=true enable_vis_stage:=true executor_threads:=4 \
    > "$LAUNCH_LOG" 2>&1 &
  LPID=$!

  sleep 14

  echo "[suite] run ${i}/${ROUNDS}: eval duration=${DURATION_SEC}s gt_source=${GT_SOURCE}"
  timeout "$((DURATION_SEC + 35))" \
    python3 "$WS_DIR/src/lvdot_bringup/scripts/eval_detection_vs_gazebo_gt.py" \
      --duration "$DURATION_SEC" \
      --gt-source "$GT_SOURCE" \
      > "$EVAL_LOG" 2>&1 || true

  kill "$LPID" 2>/dev/null || true
  sleep 1
  kill -9 "$LPID" 2>/dev/null || true
  cleanup_all
done

echo "[suite] building summary..."
python3 - "$OUT_DIR" "$ROUNDS" "$DURATION_SEC" "$GT_SOURCE" <<'PY'
import glob
import os
import re
import statistics
import sys

out_dir = sys.argv[1]
rounds = int(sys.argv[2])
duration = int(sys.argv[3])
gt_source = sys.argv[4]

eval_files = sorted(glob.glob(os.path.join(out_dir, "eval_run*.txt")))
launch_files = sorted(glob.glob(os.path.join(out_dir, "launch_run*.log")))

metric_pat = re.compile(
    r'^(uv|db|lidar|fused|tracked)\s+recall=([0-9.]+)\s+matched=([0-9]+)/([0-9]+)\s+mean_center_err=([0-9.]+)m$',
    re.M
)
yolo_pat = re.compile(r'^yolo_2d recall=([0-9.]+)\s+matched=([0-9]+)/([0-9]+)', re.M)
gt_pat = re.compile(r'^Current GT objects:\s+([0-9]+)', re.M)

keys = ["uv", "db", "lidar", "fused", "tracked"]
stats = {k: {"recall": [], "err": [], "matched": [], "gt": []} for k in keys}
yolo_recall = []
gt_objects = []

for f in eval_files:
    text = open(f, "r", encoding="utf-8", errors="ignore").read()
    gtm = gt_pat.search(text)
    if gtm:
        gt_objects.append(int(gtm.group(1)))
    for k, r, m, g, e in metric_pat.findall(text):
        stats[k]["recall"].append(float(r))
        stats[k]["err"].append(float(e))
        stats[k]["matched"].append(int(m))
        stats[k]["gt"].append(int(g))
    ym = yolo_pat.search(text)
    if ym:
        yolo_recall.append(float(ym.group(1)))

skew_counts = []
for f in launch_files:
    text = open(f, "r", encoding="utf-8", errors="ignore").read()
    skew_counts.append(text.count("skew"))

def mean_std(arr):
    if not arr:
        return (0.0, 0.0)
    if len(arr) == 1:
        return (arr[0], 0.0)
    return (statistics.mean(arr), statistics.pstdev(arr))

summary_txt = os.path.join(out_dir, "summary.txt")
summary_md = os.path.join(out_dir, "summary.md")

lines = []
lines.append(f"rounds={rounds}")
lines.append(f"duration_sec={duration}")
lines.append(f"gt_source={gt_source}")
lines.append(f"runs_collected={len(eval_files)}")
lines.append(f"gt_objects_avg={statistics.mean(gt_objects) if gt_objects else 0:.2f}")
lines.append(f"skew_warn_avg={statistics.mean(skew_counts) if skew_counts else 0:.2f}")
lines.append("")
for k in keys:
    rm, rs = mean_std(stats[k]["recall"])
    em, es = mean_std(stats[k]["err"])
    lines.append(f"{k}_recall_mean={rm:.4f} std={rs:.4f}")
    lines.append(f"{k}_err_m_mean={em:.4f} std={es:.4f}")
ym, ys = mean_std(yolo_recall)
lines.append(f"yolo2d_recall_mean={ym:.4f} std={ys:.4f}")

with open(summary_txt, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")

md = []
md.append("# GT Eval Suite Summary")
md.append("")
md.append(f"- rounds: {rounds}")
md.append(f"- duration_sec: {duration}")
md.append(f"- gt_source: `{gt_source}`")
md.append(f"- runs_collected: {len(eval_files)}")
md.append(f"- gt_objects_avg: {statistics.mean(gt_objects) if gt_objects else 0:.2f}")
md.append(f"- skew_warn_avg: {statistics.mean(skew_counts) if skew_counts else 0:.2f}")
md.append("")
md.append("## Metrics (mean ± std)")
md.append("")
for k in keys:
    rm, rs = mean_std(stats[k]["recall"])
    em, es = mean_std(stats[k]["err"])
    md.append(f"- `{k}` recall: {rm:.3f} ± {rs:.3f}, center_err(m): {em:.3f} ± {es:.3f}")
ym, ys = mean_std(yolo_recall)
md.append(f"- `yolo_2d` recall: {ym:.3f} ± {ys:.3f}")
md.append("")
md.append("## Artifacts")
md.append("")
md.append(f"- summary.txt: `{summary_txt}`")
md.append(f"- eval logs: `{out_dir}/eval_run*.txt`")
md.append(f"- launch logs: `{out_dir}/launch_run*.log`")

with open(summary_md, "w", encoding="utf-8") as f:
    f.write("\n".join(md) + "\n")

print(summary_txt)
print(summary_md)
PY

echo "[suite] done. artifacts: $OUT_DIR"

