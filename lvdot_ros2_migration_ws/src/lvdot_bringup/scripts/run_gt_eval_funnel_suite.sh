#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   run_gt_eval_funnel_suite.sh [rounds] [duration_sec] [gt_source] [min_consecutive_nonzero] [ready_timeout_sec]
#
# Example:
#   run_gt_eval_funnel_suite.sh 5 60 agents 3 90

ROUNDS="${1:-5}"
DURATION_SEC="${2:-60}"
GT_SOURCE="${3:-agents}"                  # agents | pose_info
MIN_CONSEC_NONZERO="${4:-3}"              # require this many consecutive yolo_input_count>0
READY_TIMEOUT_SEC="${5:-90}"              # wait timeout per run

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
REPORT_ROOT="$WS_DIR/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$REPORT_ROOT/gt_eval_funnel_suite_${STAMP}"
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

wait_for_yolo_ready() {
  local need="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"
  local consec=0
  local last_val=0

  while true; do
    local now
    now="$(date +%s)"
    if (( now - start_ts >= timeout_sec )); then
      echo "[funnel] timeout waiting yolo_input_count>0 (last=${last_val}, consec=${consec})"
      return 1
    fi

    local sample
    sample="$(timeout 5 ros2 topic echo /onboard_detector/pipeline_stats_status --once 2>/dev/null || true)"
    if [[ -z "${sample}" ]]; then
      consec=0
      sleep 1
      continue
    fi

    local yolo_in
    yolo_in="$(printf '%s\n' "${sample}" | sed -n 's/^yolo_input_count: //p' | head -n1)"
    if [[ -z "${yolo_in}" ]]; then
      yolo_in=0
    fi
    last_val="${yolo_in}"

    if (( yolo_in > 0 )); then
      consec=$((consec + 1))
      if (( consec >= need )); then
        echo "[funnel] yolo ready: yolo_input_count=${yolo_in}, consec=${consec}/${need}"
        return 0
      fi
    else
      consec=0
    fi
    sleep 1
  done
}

for ((i=1; i<=ROUNDS; i++)); do
  echo "[funnel] run ${i}/${ROUNDS}: cleanup"
  cleanup_all

  LAUNCH_LOG="$OUT_DIR/launch_run${i}.log"
  EVAL_LOG="$OUT_DIR/eval_run${i}.txt"
  READY_LOG="$OUT_DIR/ready_run${i}.txt"

  echo "[funnel] run ${i}/${ROUNDS}: launch scene+detector"
  nohup ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
    gazebo_gui:=false rviz:=false detector_rviz:=false \
    enable_uav_controller:=false launch_pose_stub:=true \
    enable_yolo:=true launch_yolo_node:=true fusion_mode:=dual \
    enable_stage_timers:=true enable_vis_stage:=true executor_threads:=4 \
    > "$LAUNCH_LOG" 2>&1 &
  LPID=$!

  # Let graph initialize first, then gate on YOLO readiness.
  sleep 10
  if wait_for_yolo_ready "$MIN_CONSEC_NONZERO" "$READY_TIMEOUT_SEC" | tee "$READY_LOG"; then
    echo "[funnel] run ${i}/${ROUNDS}: eval duration=${DURATION_SEC}s gt_source=${GT_SOURCE}"
    timeout "$((DURATION_SEC + 35))" \
      python3 "$WS_DIR/src/lvdot_bringup/scripts/eval_detection_vs_gazebo_gt.py" \
        --duration "$DURATION_SEC" \
        --gt-source "$GT_SOURCE" \
        > "$EVAL_LOG" 2>&1 || true
  else
    echo "[funnel] run ${i}/${ROUNDS}: skip eval (yolo not ready)" | tee "$EVAL_LOG"
  fi

  kill "$LPID" 2>/dev/null || true
  sleep 1
  kill -9 "$LPID" 2>/dev/null || true
  cleanup_all
done

echo "[funnel] building summary..."
python3 - "$OUT_DIR" "$ROUNDS" "$DURATION_SEC" "$GT_SOURCE" "$MIN_CONSEC_NONZERO" "$READY_TIMEOUT_SEC" <<'PY'
import glob
import os
import re
import statistics
import sys

out_dir = sys.argv[1]
rounds = int(sys.argv[2])
duration = int(sys.argv[3])
gt_source = sys.argv[4]
min_consec = int(sys.argv[5])
ready_timeout = int(sys.argv[6])

eval_files = sorted(glob.glob(os.path.join(out_dir, "eval_run*.txt")))
launch_files = sorted(glob.glob(os.path.join(out_dir, "launch_run*.log")))
ready_files = sorted(glob.glob(os.path.join(out_dir, "ready_run*.txt")))

metric_pat = re.compile(
    r'^(uv|db|lidar|fused|tracked)\s+recall=([0-9.]+)\s+matched=([0-9]+)/([0-9]+)\s+mean_center_err=([0-9.]+)m$',
    re.M
)
yolo_pat = re.compile(r'^yolo_2d recall=([0-9.]+)\s+matched=([0-9]+)/([0-9]+)', re.M)
gt_pat = re.compile(r'^Current GT objects:\s+([0-9]+)', re.M)
funnel_pat = re.compile(
    r'yolo_in=(\d+).*?yolo_candidate3d=(\d+).*?yolo_match3d=(\d+).*?yolo_fused_used=(\d+)'
)

keys = ["uv", "db", "lidar", "fused", "tracked"]
stats = {k: {"recall": [], "err": []} for k in keys}
yolo_recall = []
gt_objects = []
ready_ok = 0
funnel_rows = []

for f in ready_files:
    text = open(f, "r", encoding="utf-8", errors="ignore").read()
    if "yolo ready" in text:
        ready_ok += 1

for f in eval_files:
    text = open(f, "r", encoding="utf-8", errors="ignore").read()
    gtm = gt_pat.search(text)
    if gtm:
        gt_objects.append(int(gtm.group(1)))
    for k, r, _, _, e in metric_pat.findall(text):
        stats[k]["recall"].append(float(r))
        stats[k]["err"].append(float(e))
    ym = yolo_pat.search(text)
    if ym:
        yolo_recall.append(float(ym.group(1)))

skew_counts = []
for f in launch_files:
    text = open(f, "r", encoding="utf-8", errors="ignore").read()
    skew_counts.append(text.count("skew"))
    matches = [tuple(map(int, m)) for m in funnel_pat.findall(text)]
    nonzero = [m for m in matches if m[0] > 0]
    if nonzero:
        funnel_rows.append(nonzero[-1])

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
lines.append(f"ready_gate=min_consecutive_nonzero={min_consec}, timeout_sec={ready_timeout}")
lines.append(f"ready_ok_runs={ready_ok}/{rounds}")
lines.append(f"runs_collected={len(eval_files)}")
lines.append(f"gt_objects_avg={statistics.mean(gt_objects) if gt_objects else 0:.2f}")
lines.append(f"skew_warn_avg={statistics.mean(skew_counts) if skew_counts else 0:.2f}")
for k in keys:
    rm, rs = mean_std(stats[k]["recall"])
    em, es = mean_std(stats[k]["err"])
    lines.append(f"{k}_recall_mean={rm:.4f} std={rs:.4f}")
    lines.append(f"{k}_err_m_mean={em:.4f} std={es:.4f}")
ym, ys = mean_std(yolo_recall)
lines.append(f"yolo2d_recall_mean={ym:.4f} std={ys:.4f}")

if funnel_rows:
    y_in = statistics.mean([r[0] for r in funnel_rows])
    cand = statistics.mean([r[1] for r in funnel_rows])
    match = statistics.mean([r[2] for r in funnel_rows])
    used = statistics.mean([r[3] for r in funnel_rows])
    lines.append(f"funnel_avg_yolo_in={y_in:.2f}")
    lines.append(f"funnel_avg_candidate3d={cand:.2f}")
    lines.append(f"funnel_avg_match3d={match:.2f}")
    lines.append(f"funnel_avg_fused_used={used:.2f}")
    lines.append(f"funnel_rate_candidate_over_in={(cand / y_in) if y_in > 0 else 0.0:.4f}")
    lines.append(f"funnel_rate_match_over_candidate={(match / cand) if cand > 0 else 0.0:.4f}")
    lines.append(f"funnel_rate_used_over_match={(used / match) if match > 0 else 0.0:.4f}")
    lines.append(f"funnel_rate_used_over_in={(used / y_in) if y_in > 0 else 0.0:.4f}")
else:
    lines.append("funnel_rows=0")

with open(summary_txt, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")

md = []
md.append("# GT Funnel Eval Suite Summary")
md.append("")
md.append(f"- rounds: {rounds}")
md.append(f"- duration_sec: {duration}")
md.append(f"- gt_source: `{gt_source}`")
md.append(f"- ready_gate: `min_consecutive_nonzero={min_consec}, timeout_sec={ready_timeout}`")
md.append(f"- ready_ok_runs: {ready_ok}/{rounds}")
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
md.append("## YOLO Funnel (last nonzero snapshot per run)")
md.append("")
if funnel_rows:
    y_in = statistics.mean([r[0] for r in funnel_rows])
    cand = statistics.mean([r[1] for r in funnel_rows])
    match = statistics.mean([r[2] for r in funnel_rows])
    used = statistics.mean([r[3] for r in funnel_rows])
    md.append(f"- avg `yolo_in`: {y_in:.2f}")
    md.append(f"- avg `yolo_candidate3d`: {cand:.2f}")
    md.append(f"- avg `yolo_match3d`: {match:.2f}")
    md.append(f"- avg `yolo_fused_used`: {used:.2f}")
    md.append(f"- rate `candidate/in`: {(cand / y_in) if y_in > 0 else 0.0:.3f}")
    md.append(f"- rate `match/candidate`: {(match / cand) if cand > 0 else 0.0:.3f}")
    md.append(f"- rate `used/match`: {(used / match) if match > 0 else 0.0:.3f}")
    md.append(f"- rate `used/in`: {(used / y_in) if y_in > 0 else 0.0:.3f}")
else:
    md.append("- no nonzero funnel rows detected")
md.append("")
md.append("## Artifacts")
md.append("")
md.append(f"- summary.txt: `{summary_txt}`")
md.append(f"- eval logs: `{out_dir}/eval_run*.txt`")
md.append(f"- launch logs: `{out_dir}/launch_run*.log`")
md.append(f"- ready logs: `{out_dir}/ready_run*.txt`")

with open(summary_md, "w", encoding="utf-8") as f:
    f.write("\n".join(md) + "\n")

print(summary_txt)
print(summary_md)
PY

echo "[funnel] done. artifacts: $OUT_DIR"
