#!/usr/bin/env bash
set -euo pipefail

ROUNDS="${1:-3}"
DURATION_SEC="${2:-60}"
GT_SOURCE="${3:-agents}"

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
REPORT_ROOT="$WS_DIR/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$REPORT_ROOT/gt_eval_visible_suite_${STAMP}"
mkdir -p "$OUT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

cleanup_all() {
  pkill -9 -f 'run_detector_with_scene.launch.py|uav_pedestrian_prototype.launch.py|lvdot_detector_main|lvdot_yolo_node|parameter_bridge|rviz2|gz sim|ros2 launch|gt_detection_publisher|image_pointcloud_relay' >/dev/null 2>&1 || true
  sleep 2
}

for ((i=1; i<=ROUNDS; i++)); do
  echo "[visible_suite] run ${i}/${ROUNDS}: cleanup"
  cleanup_all
  LAUNCH_LOG="$OUT_DIR/launch_run${i}.log"
  EVAL_LOG="$OUT_DIR/eval_run${i}.txt"

  echo "[visible_suite] run ${i}/${ROUNDS}: launch"
  nohup ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
    gazebo_gui:=false rviz:=false detector_rviz:=false \
    enable_uav_controller:=false launch_pose_stub:=true \
    enable_yolo:=true launch_yolo_node:=true fusion_mode:=dual \
    enable_stage_timers:=true enable_vis_stage:=true executor_threads:=4 \
    > "$LAUNCH_LOG" 2>&1 &
  LPID=$!

  sleep 14

  echo "[visible_suite] run ${i}/${ROUNDS}: eval visible-only"
  timeout "$((DURATION_SEC + 35))" \
    python3 "$WS_DIR/src/lvdot_bringup/scripts/eval_detection_vs_gazebo_gt.py" \
      --duration "$DURATION_SEC" \
      --gt-source "$GT_SOURCE" \
      --visible-only \
      > "$EVAL_LOG" 2>&1 || true

  kill "$LPID" 2>/dev/null || true
  sleep 1
  kill -9 "$LPID" 2>/dev/null || true
  cleanup_all
done

python3 - "$OUT_DIR" "$ROUNDS" "$DURATION_SEC" "$GT_SOURCE" <<'PY'
import glob, os, re, statistics, sys
out_dir, rounds, duration, gt_source = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
metric_pat = re.compile(r'^(uv|db|lidar|fused|tracked)\s+recall=([0-9.]+)\s+matched=([0-9]+)/([0-9]+)\s+mean_center_err=([0-9.]+)m$', re.M)
yolo_pat = re.compile(r'^yolo_2d recall=([0-9.]+)\s+matched=([0-9]+)/([0-9]+)', re.M)
vis_pat = re.compile(r'^Visible-only scoring:\s+([01])', re.M)
gt_pat = re.compile(r'^Current GT objects:\s+([0-9]+)', re.M)
keys=["uv","db","lidar","fused","tracked"]
stats={k:{"recall":[],"err":[]} for k in keys}
y=[]; gt=[]; vis=[]
for f in sorted(glob.glob(os.path.join(out_dir,'eval_run*.txt'))):
    t=open(f,encoding='utf-8',errors='ignore').read()
    m=gt_pat.search(t)
    if m: gt.append(int(m.group(1)))
    m=vis_pat.search(t)
    if m: vis.append(int(m.group(1)))
    for k,r,_,_,e in metric_pat.findall(t):
        stats[k]['recall'].append(float(r)); stats[k]['err'].append(float(e))
    m=yolo_pat.search(t)
    if m: y.append(float(m.group(1)))
def ms(a):
    if not a:return (0.0,0.0)
    if len(a)==1:return (a[0],0.0)
    return statistics.mean(a), statistics.pstdev(a)
md=[]
md.append('# GT Eval Visible-only Suite Summary\n')
md.append(f'- rounds: {rounds}')
md.append(f'- duration_sec: {duration}')
md.append(f'- gt_source: `{gt_source}`')
md.append(f'- visible_only: {all(v==1 for v in vis) if vis else False}')
md.append(f'- runs_collected: {len(glob.glob(os.path.join(out_dir,"eval_run*.txt")))}')
md.append(f'- gt_objects_avg: {statistics.mean(gt) if gt else 0:.2f}\n')
md.append('## Metrics (mean ± std)\n')
for k in keys:
    rm,rs=ms(stats[k]['recall']); em,es=ms(stats[k]['err'])
    md.append(f'- `{k}` recall: {rm:.3f} ± {rs:.3f}, center_err(m): {em:.3f} ± {es:.3f}')
ym,ys=ms(y); md.append(f'- `yolo_2d` recall: {ym:.3f} ± {ys:.3f}')
open(os.path.join(out_dir,'summary.md'),'w',encoding='utf-8').write('\n'.join(md)+'\n')
print(os.path.join(out_dir,'summary.md'))
PY

echo "[visible_suite] done: $OUT_DIR"
