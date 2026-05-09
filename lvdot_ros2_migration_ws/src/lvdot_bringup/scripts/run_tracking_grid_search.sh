#!/usr/bin/env bash
set -euo pipefail

# Grid search on tracking association params
# Usage: run_tracking_grid_search.sh [rounds] [duration] [gt_source]
ROUNDS="${1:-3}"
DURATION="${2:-60}"
GT_SOURCE="${3:-agents}"

WS_DIR="/home/skbt2/lvdot_ros2_migration_ws"
CFG="$WS_DIR/src/lvdot_bringup/config/detector_param.yaml"
BASE_TUNED="$WS_DIR/src/lvdot_bringup/config/detector_param_tuned_v1.yaml"
RUN_SUITE="$WS_DIR/src/lvdot_bringup/scripts/run_gt_eval_suite.sh"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/reports/tracking_grid_${STAMP}"
mkdir -p "$OUT_DIR"

cp "$CFG" "$OUT_DIR/detector_param.backup.yaml"
restore_cfg(){ cp "$OUT_DIR/detector_param.backup.yaml" "$CFG"; }
trap restore_cfg EXIT

cleanup_all(){
  pkill -9 -f 'run_detector_with_scene.launch.py|run_full_stack_qcgaf_gru.launch.py|uav_pedestrian_prototype.launch.py|qcgaf_fusion_node|gru_prediction_node|lvdot_detector_main|lvdot_yolo_node|parameter_bridge|rviz2|gz sim|ros2 launch|gt_detection_publisher|image_pointcloud_relay' >/dev/null 2>&1 || true
  sleep 2
}

make_cfg(){
  local out_cfg="$1"; local prop_w="$2"; local min_sim="$3"
  cp "$BASE_TUNED" "$out_cfg"
  python3 - "$out_cfg" "$prop_w" "$min_sim" <<'PY'
import sys
p=sys.argv[1]; w=sys.argv[2]; m=sys.argv[3]
repl={
 'adaptive_similarity_weight:':'    adaptive_similarity_weight: false\n',
 'sim_prev_weight:':'    sim_prev_weight: 1.0\n',
 'sim_proped_weight:':f'    sim_proped_weight: {w}\n',
 'min_match_similarity:':f'    min_match_similarity: {m}\n',
}
lines=[]
with open(p,'r',encoding='utf-8') as f:
    for line in f:
        s=line.strip()
        done=False
        for k,v in repl.items():
            if s.startswith(k):
                lines.append(v); done=True; break
        if not done: lines.append(line)
with open(p,'w',encoding='utf-8') as f: f.writelines(lines)
PY
}

summaries=()
for w in 1.0 1.2 1.4; do
  for m in -1.2 -1.0 -0.8; do
    tag="w${w}_m${m}"
    cfg_tmp="$OUT_DIR/${tag}.yaml"
    make_cfg "$cfg_tmp" "$w" "$m"
    cp "$cfg_tmp" "$CFG"
    cleanup_all
    "$RUN_SUITE" "$ROUNDS" "$DURATION" "$GT_SOURCE"
    latest="$(ls -dt "$WS_DIR"/reports/gt_eval_suite_* | head -n 1)"
    cp -r "$latest" "$OUT_DIR/$tag"
    summaries+=("$tag:$OUT_DIR/$tag/summary.txt")
  done
done

python3 - "$OUT_DIR" <<'PY'
import os,glob
root=os.sys.argv[1]
rows=[]
for d in sorted(glob.glob(os.path.join(root,'w*_m*'))):
    s=os.path.join(d,'summary.txt')
    if not os.path.isfile(s): continue
    m={}
    for line in open(s,encoding='utf-8'):
        line=line.strip()
        if '=' in line:
            k,v=line.split('=',1)
            if ' std=' in v: v=v.split(' std=')[0]
            m[k]=v
    def f(k):
        try:return float(m.get(k,'0'))
        except:return 0.0
    rows.append((os.path.basename(d),f('tracked_recall_mean'),f('tracked_err_m_mean'),f('fused_recall_mean'),f('skew_warn_avg')))
rows.sort(key=lambda x:(-x[1], x[2]))
out=os.path.join(root,'grid_report.md')
with open(out,'w',encoding='utf-8') as fp:
    fp.write('# Tracking Grid Report\n\n')
    fp.write('| config | tracked_recall | tracked_err_m | fused_recall | skew_warn_avg |\n')
    fp.write('|---|---:|---:|---:|---:|\n')
    for r in rows:
      fp.write(f'| {r[0]} | {r[1]:.4f} | {r[2]:.4f} | {r[3]:.4f} | {r[4]:.4f} |\n')
print(out)
PY

restore_cfg
cleanup_all
echo "[grid] done: $OUT_DIR"
