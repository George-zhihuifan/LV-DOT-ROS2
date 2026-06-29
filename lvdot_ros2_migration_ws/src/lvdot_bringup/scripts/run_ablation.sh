#!/usr/bin/env bash
set -euo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"

# Auto-detect workspace roots from this script's location (src/lvdot_bringup/scripts/).
# Override by exporting WS_ROOT / DEPTH_WS before calling.
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_MIGRATION_WS="$(cd "${_SCRIPT_DIR}/../../.." && pwd)"
WS_ROOT="${WS_ROOT:-${_MIGRATION_WS}}"
DEPTH_WS="${DEPTH_WS:-$(cd "${_MIGRATION_WS}/.." && pwd)/ros2_depth_eval_ws}"
OUT_ROOT="${OUT_ROOT:-${WS_ROOT}/logs/ablation_$(date +%Y%m%d_%H%M%S)}"
EVAL_DURATION_SEC="${EVAL_DURATION_SEC:-60.0}"
WARMUP_SEC="${WARMUP_SEC:-15.0}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
RVIZ="${RVIZ:-false}"
POSE_MODE="${POSE_MODE:-}"
MISSION_SPEED="${MISSION_SPEED:-}"
MISSION_ALTITUDE="${MISSION_ALTITUDE:-}"
GT_BBOX_WIDTH_M="${GT_BBOX_WIDTH_M:-0.36}"
GT_BBOX_DEPTH_M="${GT_BBOX_DEPTH_M:-0.36}"
GT_BBOX_HEIGHT_M="${GT_BBOX_HEIGHT_M:-1.70}"
GT_BBOX_CENTER_Y_OFFSET_M="${GT_BBOX_CENTER_Y_OFFSET_M:--0.03}"
GT_BBOX_CENTER_Z_OFFSET_M="${GT_BBOX_CENTER_Z_OFFSET_M:-0.88}"
EVAL_INCLUDE_STATIC_OBSTACLES="${EVAL_INCLUDE_STATIC_OBSTACLES:-false}"
EVAL_GT_OBSTACLE_TOPIC="${EVAL_GT_OBSTACLE_TOPIC:-/pedestrian_sim/agent_markers}"
EVAL_GT_OBSTACLE_NAMESPACE="${EVAL_GT_OBSTACLE_NAMESPACE:-pedestrian_obstacles}"
EVAL_TRACKING_DET_TOPIC="${EVAL_TRACKING_DET_TOPIC:-}"
EVAL_TRACKING_DET_MARKER_TYPE="${EVAL_TRACKING_DET_MARKER_TYPE:--1}"
EVAL_TRACKING_DET_NAMESPACE="${EVAL_TRACKING_DET_NAMESPACE:-}"
NUM_RUNS="${NUM_RUNS:-3}"
CONFIGS_FILTER="${CONFIGS_FILTER:-}"
SCENARIOS_FILTER="${SCENARIOS_FILTER:-}"
LOCK_FILE="${WS_ROOT}/logs/.run_ablation.lock"
LOCK_FD=201

mkdir -p "${WS_ROOT}/logs"
eval "exec ${LOCK_FD}>\"${LOCK_FILE}\""
if ! flock -n "${LOCK_FD}"; then
  echo "Another run_ablation.sh instance is running (lock: ${LOCK_FILE}). Exit."
  exit 1
fi
echo "$$" 1>&"${LOCK_FD}"

set +u
source /opt/ros/humble/setup.bash
source "${DEPTH_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

mkdir -p "${OUT_ROOT}"

DEFAULT_SCENARIOS=(
  "dense_open:${DEPTH_WS}/install/depth_eval_bringup/share/depth_eval_bringup/config/pedestrian_dense.yaml"
  "sparse_wide:${DEPTH_WS}/install/depth_eval_bringup/share/depth_eval_bringup/config/pedestrian_prototype.yaml"
)

DEFAULT_ABLATION_GROUPS=(
  "A0_yolo_sort:na:true:false:false:${WS_ROOT}/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml:/yolo_sort/tracked_bboxes:1:yolo_sort"
  "A1_lidar_only:lidar_driven:false:false:false:${WS_ROOT}/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml:/onboard_detector/dynamic_bboxes:5:dynamic"
  "A3_lvdot_baseline:dual:true:false:false:${WS_ROOT}/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml:/onboard_detector/dynamic_bboxes:5:dynamic"
  "A4_qcgaf:dual:true:true:false:${WS_ROOT}/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml:/qcgaf/fused_bboxes:1:qcgaf_fused"
  "A5_qcgaf_gru:dual:true:true:true:${WS_ROOT}/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline.yaml:/qcgaf/fused_bboxes:1:qcgaf_fused"
  "A6_noise_adapt:dual:true:true:true:${WS_ROOT}/install/lvdot_bringup/share/lvdot_bringup/config/detector_param_baseline_noise.yaml:/qcgaf/fused_bboxes:1:qcgaf_fused"
)

SCENARIOS=("${DEFAULT_SCENARIOS[@]}")
if [[ -n "${SCENARIOS_OVERRIDE:-}" ]]; then
  IFS='|' read -r -a SCENARIOS <<< "${SCENARIOS_OVERRIDE}"
fi

ABLATION_GROUPS=("${DEFAULT_ABLATION_GROUPS[@]}")
if [[ -n "${ABLATION_GROUPS_OVERRIDE:-}" ]]; then
  IFS='|' read -r -a ABLATION_GROUPS <<< "${ABLATION_GROUPS_OVERRIDE}"
fi

if [[ -n "${SCENARIOS_FILTER}" ]]; then
  FILTERED_SCENARIOS=()
  IFS=',' read -r -a scenario_filters <<< "${SCENARIOS_FILTER}"
  for scenario in "${SCENARIOS[@]}"; do
    scenario_name="${scenario%%:*}"
    for filter in "${scenario_filters[@]}"; do
      if [[ "${scenario_name}" == "${filter}" ]]; then
        FILTERED_SCENARIOS+=("${scenario}")
        break
      fi
    done
  done
  SCENARIOS=("${FILTERED_SCENARIOS[@]}")
fi

if [[ -n "${CONFIGS_FILTER}" ]]; then
  FILTERED_GROUPS=()
  IFS=',' read -r -a config_filters <<< "${CONFIGS_FILTER}"
  for group in "${ABLATION_GROUPS[@]}"; do
    group_name="${group%%:*}"
    for filter in "${config_filters[@]}"; do
      if [[ "${group_name}" == "${filter}" ]]; then
        FILTERED_GROUPS+=("${group}")
        break
      fi
    done
  done
  ABLATION_GROUPS=("${FILTERED_GROUPS[@]}")
fi

cleanup() {
  local pids=()
  local patterns=()
  if [[ -n "${launch_pid:-}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -- -"${launch_pid}" 2>/dev/null || kill "${launch_pid}" 2>/dev/null || true
  fi
  for proc in gz ruby_gz ruby ign parameter_bridge ros_gz_bridge lvdot_detector_main lvdot_yolo_node advanced_evaluator fusion_node predict_node pose_stub uav_waypoint_mission pedestrian_state_publisher rviz2 d435i_sim mid360_sim; do
    mapfile -t pids < <(pgrep -x "${proc}" 2>/dev/null || true)
    for pid in "${pids[@]}"; do
      kill "${pid}" 2>/dev/null || true
    done
  done
  patterns=(
    'ros2 launch lvdot_bringup run_full_pipeline.launch.py'
    'ros2 launch lvdot_bringup run_yolo_sort_baseline.launch.py'
    '/lvdot_ros2_adapter/.*/advanced_evaluator'
    '/lvdot_ros2_adapter/.*/uav_waypoint_mission'
    '/lvdot_ros2_adapter/.*/lvdot_yolo_node'
    '/qcgaf_fusion/.*/fusion_node'
    '/gru_predictor/.*/predict_node'
    '/lvdot_ros2/.*/lvdot_detector_main'
    'ign gazebo -s -r'
    'ros_gz_bridge'
    'parameter_bridge'
    'pedestrian_state_publisher'
    'd435i_sim'
    'mid360_sim'
  )
  for pattern in "${patterns[@]}"; do
    mapfile -t pids < <(pgrep -f "${pattern}" 2>/dev/null || true)
    for pid in "${pids[@]}"; do
      kill "${pid}" 2>/dev/null || true
    done
  done
  sleep 30
}

deep_cleanup() {
  local pids=()
  local patterns=()
  cleanup
  # Extra hard cleanup for simulator leftovers.
  for proc in gzserver gzclient gz-sim-server gz-sim-gui ruby; do
    mapfile -t pids < <(pgrep -f "${proc}" 2>/dev/null || true)
    for pid in "${pids[@]}"; do
      kill -9 "${pid}" 2>/dev/null || true
    done
  done
  patterns=(
    'ros2 launch lvdot_bringup run_full_pipeline.launch.py'
    'ros2 launch lvdot_bringup run_yolo_sort_baseline.launch.py'
    '/lvdot_ros2_adapter/.*/advanced_evaluator'
    '/lvdot_ros2_adapter/.*/uav_waypoint_mission'
    '/lvdot_ros2_adapter/.*/lvdot_yolo_node'
    '/qcgaf_fusion/.*/fusion_node'
    '/gru_predictor/.*/predict_node'
    '/lvdot_ros2/.*/lvdot_detector_main'
    'ign gazebo -s -r'
    'ros_gz_bridge'
    'parameter_bridge'
    'pedestrian_state_publisher'
    'd435i_sim'
    'mid360_sim'
  )
  for pattern in "${patterns[@]}"; do
    mapfile -t pids < <(pgrep -f "${pattern}" 2>/dev/null || true)
    for pid in "${pids[@]}"; do
      kill -9 "${pid}" 2>/dev/null || true
    done
  done
  rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  sleep 60
}

on_exit() {
  cleanup
  flock -u "${LOCK_FD}" 2>/dev/null || true
  rm -f "${LOCK_FILE}" 2>/dev/null || true
}

trap on_exit EXIT

run_once() {
  local run_dir="$1"
  local scenario_config="$2"
  local detector_config="$3"
  local fusion_mode="$4"
  local enable_yolo="$5"
  local enable_qcgaf="$6"
  local enable_gru="$7"
  local det_topic="$8"
  local marker_type="$9"
  local det_ns="${10}"
  local group_name="${11}"
  local pose_mode="${12}"
  local mission_speed="${13}"
  local mission_altitude="${14}"

  if [[ "${group_name}" == "A0_yolo_sort" ]]; then
    ros2 launch lvdot_bringup run_yolo_sort_baseline.launch.py \
      gazebo_gui:="${GAZEBO_GUI}" \
      rviz:="${RVIZ}" \
      scenario_config:="${scenario_config}" \
      evaluator_csv_path:="${run_dir}/frames.csv" \
      evaluator_summary_path:="${run_dir}/summary.json" \
      evaluator_eval_duration_sec:="${EVAL_DURATION_SEC}" \
      evaluator_warmup_sec:="${WARMUP_SEC}" \
      evaluator_gt_bbox_width_m:="${GT_BBOX_WIDTH_M}" \
      evaluator_gt_bbox_depth_m:="${GT_BBOX_DEPTH_M}" \
      evaluator_gt_bbox_height_m:="${GT_BBOX_HEIGHT_M}" \
      evaluator_gt_bbox_center_y_offset_m:="${GT_BBOX_CENTER_Y_OFFSET_M}" \
      evaluator_gt_bbox_center_z_offset_m:="${GT_BBOX_CENTER_Z_OFFSET_M}" \
      > "${run_dir}/launch.log" 2>&1 &
  else
    local -a launch_args=(
      gazebo_gui:="${GAZEBO_GUI}"
      rviz:="${RVIZ}"
      scenario_config:="${scenario_config}"
      pose_mode:="${pose_mode}"
      mission_speed:="${mission_speed}"
      mission_altitude:="${mission_altitude}"
      detector_config:="${detector_config}"
      fusion_mode:="${fusion_mode}"
      enable_yolo:="${enable_yolo}"
      launch_yolo_node:="${enable_yolo}"
      conf_threshold:="0.72"
      inference_hz:="10.0"
      imgsz:="352"
      max_det:="15"
      enable_qcgaf:="${enable_qcgaf}"
      enable_gru:="${enable_gru}"
      launch_advanced_evaluator:=true
      evaluator_csv_path:="${run_dir}/frames.csv"
      evaluator_summary_path:="${run_dir}/summary.json"
      evaluator_include_static_obstacles:="${EVAL_INCLUDE_STATIC_OBSTACLES}"
      evaluator_gt_obstacle_topic:="${EVAL_GT_OBSTACLE_TOPIC}"
      evaluator_gt_obstacle_namespace:="${EVAL_GT_OBSTACLE_NAMESPACE}"
      evaluator_det_topic:="${det_topic}"
      evaluator_det_marker_type:="${marker_type}"
      evaluator_det_namespace:="${det_ns}"
      evaluator_eval_duration_sec:="${EVAL_DURATION_SEC}"
      evaluator_warmup_sec:="${WARMUP_SEC}"
      evaluator_gt_bbox_width_m:="${GT_BBOX_WIDTH_M}"
      evaluator_gt_bbox_depth_m:="${GT_BBOX_DEPTH_M}"
      evaluator_gt_bbox_height_m:="${GT_BBOX_HEIGHT_M}"
      evaluator_gt_bbox_center_y_offset_m:="${GT_BBOX_CENTER_Y_OFFSET_M}"
      evaluator_gt_bbox_center_z_offset_m:="${GT_BBOX_CENTER_Z_OFFSET_M}"
    )
    if [[ -n "${EVAL_TRACKING_DET_TOPIC}" ]]; then
      launch_args+=(
        evaluator_tracking_det_topic:="${EVAL_TRACKING_DET_TOPIC}"
        evaluator_tracking_det_marker_type:="${EVAL_TRACKING_DET_MARKER_TYPE}"
        evaluator_tracking_det_namespace:="${EVAL_TRACKING_DET_NAMESPACE}"
      )
    fi
    if [[ -n "${QCGAF_EXTRA_ARGS:-}" ]]; then
      # Optional extra args are shell-style user overrides; split intentionally.
      # shellcheck disable=SC2206
      local -a extra_args=( ${QCGAF_EXTRA_ARGS} )
      launch_args+=("${extra_args[@]}")
    fi
    ros2 launch lvdot_bringup run_full_pipeline.launch.py \
      "${launch_args[@]}" \
      > "${run_dir}/launch.log" 2>&1 &
  fi

  launch_pid=$!
  launch_exit_rc=0
  timed_out=0
  max_wait=$(python3 - <<PY
print(int(float("${EVAL_DURATION_SEC}") + float("${WARMUP_SEC}") + 480))
PY
)
  for _ in $(seq 1 "${max_wait}"); do
    if [[ -f "${run_dir}/summary.json" ]]; then
      break
    fi
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done

  if kill -0 "${launch_pid}" 2>/dev/null; then
    timed_out=1
  fi
  cleanup
  if kill -0 "${launch_pid}" 2>/dev/null; then
    launch_exit_rc=143
  else
    launch_exit_rc=0
  fi
}

for run_idx in $(seq 1 "${NUM_RUNS}"); do
  for scenario in "${SCENARIOS[@]}"; do
    scenario_name="${scenario%%:*}"
    scenario_config="${scenario#*:}"
    scenario_pose_mode="${POSE_MODE}"
    scenario_mission_speed="${MISSION_SPEED}"
    scenario_mission_altitude="${MISSION_ALTITUDE}"
    if [[ "${scenario_name}" == linear* ]]; then
      scenario_pose_mode="${scenario_pose_mode:-mission}"
      scenario_mission_speed="${scenario_mission_speed:-1.0}"
      scenario_mission_altitude="${scenario_mission_altitude:-1.6}"
    else
      scenario_pose_mode="${scenario_pose_mode:-orbit}"
      scenario_mission_speed="${scenario_mission_speed:-1.5}"
      scenario_mission_altitude="${scenario_mission_altitude:-1.2}"
    fi
    for group in "${ABLATION_GROUPS[@]}"; do
      group_name="${group%%:*}"
      group_rest="${group#*:}"
      IFS=":" read -r fusion_mode enable_yolo enable_qcgaf enable_gru detector_config det_topic marker_type det_ns <<< "${group_rest}"
      run_dir="${OUT_ROOT}/run${run_idx}_${scenario_name}_${group_name}"
      if [[ -f "${run_dir}/summary.json" ]]; then
        echo "[$(date --iso-8601=seconds)] skip run${run_idx} ${scenario_name} ${group_name} (already done)"
        continue
      fi
      mkdir -p "${run_dir}"

      deep_cleanup
      echo "[$(date --iso-8601=seconds)] start run${run_idx} ${scenario_name} ${group_name}" | tee "${run_dir}/run.log"
      echo "scenario_config=${scenario_config}" >> "${run_dir}/run.log"
      echo "detector_config=${detector_config}" >> "${run_dir}/run.log"
      echo "pose_mode=${scenario_pose_mode} mission_speed=${scenario_mission_speed} mission_altitude=${scenario_mission_altitude}" >> "${run_dir}/run.log"
      echo "fusion_mode=${fusion_mode} enable_yolo=${enable_yolo} enable_qcgaf=${enable_qcgaf} enable_gru=${enable_gru}" >> "${run_dir}/run.log"

      success=0
      for attempt in 1 2; do
        [[ "${attempt}" -gt 1 ]] && echo "[$(date --iso-8601=seconds)] retry attempt ${attempt} run${run_idx} ${scenario_name} ${group_name}" | tee -a "${run_dir}/run.log"
        run_once "${run_dir}" "${scenario_config}" "${detector_config}" "${fusion_mode}" "${enable_yolo}" "${enable_qcgaf}" "${enable_gru}" "${det_topic}" "${marker_type}" "${det_ns}" "${group_name}" "${scenario_pose_mode}" "${scenario_mission_speed}" "${scenario_mission_altitude}"

        is_ok=$(python3 - <<PY
import json
grp = "${group_name}"
path = "${run_dir}/summary.json"
try:
    s = json.load(open(path, "r", encoding="utf-8"))
    tf = s.get("total_frames", 0) or 0
    f1 = (((s.get("center_distance") or {}).get("1.0m") or {}).get("f1", 0) or 0)
    ok = (tf > 1000) and ((f1 > 0.01) or ("A1" in grp) or ("A0" in grp))
    print("OK" if ok else "BAD")
except Exception:
    print("BAD")
PY
)
        if [[ "${is_ok}" == "OK" ]]; then
          success=1
          break
        fi

        mv "${run_dir}/summary.json" "${run_dir}/summary.failed_attempt${attempt}.json" 2>/dev/null || true
        echo "[$(date --iso-8601=seconds)] attempt ${attempt} marked BAD run${run_idx} ${scenario_name} ${group_name}" | tee -a "${run_dir}/run.log"
        deep_cleanup
      done

      if [[ "${success}" -eq 1 ]]; then
        echo "[$(date --iso-8601=seconds)] done run${run_idx} ${scenario_name} ${group_name} (launch_rc=${launch_exit_rc})" | tee -a "${run_dir}/run.log"
      else
        if [[ "${timed_out}" -eq 1 ]]; then
          echo "[$(date --iso-8601=seconds)] missing summary run${run_idx} ${scenario_name} ${group_name} (reason=timeout_or_bad launch_rc=${launch_exit_rc})" | tee -a "${run_dir}/run.log"
        else
          echo "[$(date --iso-8601=seconds)] missing summary run${run_idx} ${scenario_name} ${group_name} (reason=launch_exit_or_bad launch_rc=${launch_exit_rc})" | tee -a "${run_dir}/run.log"
        fi
      fi
    done
  done
done

python3 "${WS_ROOT}/src/lvdot_bringup/scripts/summarize_results.py" --root "${OUT_ROOT}" --output "${OUT_ROOT}/SUMMARY.md"
echo "Ablation output: ${OUT_ROOT}"
