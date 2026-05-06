#!/usr/bin/env bash
set -euo pipefail

source_setup() {
  local setup_path="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_path}"
  set -u
}

source_setup /opt/ros/jazzy/setup.bash
source_setup /home/skbt2/ros2_depth_eval_ws/install/setup.bash
source_setup /home/skbt2/lvdot_ros2_ws/install/setup.bash

READY_TIMEOUT_SECONDS="${READY_TIMEOUT_SECONDS:-900}"
READY_POLL_SECONDS="${READY_POLL_SECONDS:-5}"
READY_MIN_UPTIME_SECONDS="${READY_MIN_UPTIME_SECONDS:-20}"
POST_READY_SAMPLE_SECONDS="${POST_READY_SAMPLE_SECONDS:-60}"
ENABLE_YOLO="${ENABLE_YOLO:-true}"
LAUNCH_YOLO_NODE="${LAUNCH_YOLO_NODE:-${ENABLE_YOLO}}"
USE_ALL_CLASSES="${USE_ALL_CLASSES:-false}"
ENABLE_COLOR_FALLBACK="${ENABLE_COLOR_FALLBACK:-false}"
ENABLE_VIS_STAGE="${ENABLE_VIS_STAGE:-false}"
EXECUTOR_THREADS="${EXECUTOR_THREADS:-1}"
YOLO_CONF_THRESHOLD="${YOLO_CONF_THRESHOLD:-0.25}"
YOLO_INFERENCE_HZ="${YOLO_INFERENCE_HZ:-10.0}"
WINDOW_SECONDS="${WINDOW_SECONDS:-30}"
SAMPLE_HZ="${SAMPLE_HZ:-1}"
HIT_MIN_COUNT="${HIT_MIN_COUNT:-5}"
COUNT_TOLERANCE="${COUNT_TOLERANCE:-0}"
SERVICE_QUERY_RANGE="${SERVICE_QUERY_RANGE:-120.0}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-/home/skbt2/lvdot_ros2_ws/artifacts}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${ARTIFACT_ROOT}/real_scene_regression_${STAMP}"
mkdir -p "${ARTIFACT_DIR}"
STATUS_SNAPSHOT_CMD="/home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/capture_status_snapshot.py"

FAILURE_REASON="unknown"
EXECUTION_STATUS="unknown"
READY_WAIT_SECONDS=0
READY_STABLE_UPTIME_SECONDS=0
LAUNCH_LOG_FILE="${ARTIFACT_DIR}/launch.log"
WINDOW_DYNAMIC_COUNT=0
WINDOW_SERVICE_POSITION_COUNT=0
WINDOW_ABS_DIFF=0
WINDOW_TOTAL_SAMPLES=0
WINDOW_HIT_COUNT=0
EXECUTOR_THREADS_APPLIED="unknown"
ENABLE_VIS_STAGE_APPLIED="unknown"

section_line() {
  local section="$1"
  if [[ ! -f "${LAUNCH_LOG_FILE}" ]]; then
    return
  fi
  grep "${section}:" "${LAUNCH_LOG_FILE}" | tail -n1 || true
}

line_field_value() {
  local line="$1"
  local key="$2"
  awk -v target="${key}" '
    {
      for (i = 1; i <= NF; ++i) {
        field = $i
        gsub(/,/, "", field)
        if (index(field, "=") == 0) {
          continue
        }
        split(field, kv, "=")
        if (kv[1] == target) {
          if (kv[2] ~ /^[0-9]+$/) {
            print kv[2]
            exit
          }
          if (kv[2] ~ /^[0-9]+\/[0-9]+$/) {
            split(kv[2], pair, "/")
            print pair[1]
            exit
          }
        }
      }
    }' <<< "${line}" | head -n1
}

capture_log_snapshot() {
  local output_file="$1"
  local input_line stage_line stats_line

  input_line="$(section_line "Input health")"
  stage_line="$(section_line "Stage timers")"
  stats_line="$(section_line "Pipeline stats")"

  if [[ -z "${input_line}" && -z "${stage_line}" && -z "${stats_line}" ]]; then
    return 1
  fi

  : > "${output_file}"
  printf 'input_health_received=%s\n' "$([[ -n "${input_line}" ]] && echo 1 || echo 0)" >> "${output_file}"
  printf 'stage_timers_received=%s\n' "$([[ -n "${stage_line}" ]] && echo 1 || echo 0)" >> "${output_file}"
  printf 'pipeline_stats_received=%s\n' "$([[ -n "${stats_line}" ]] && echo 1 || echo 0)" >> "${output_file}"

  printf 'depth_count=%s\n' "$(line_field_value "${input_line}" depth || true)" >> "${output_file}"
  printf 'color_count=%s\n' "$(line_field_value "${input_line}" color || true)" >> "${output_file}"
  printf 'lidar_count=%s\n' "$(line_field_value "${input_line}" lidar || true)" >> "${output_file}"
  printf 'pose_count=%s\n' "$(line_field_value "${input_line}" pose || true)" >> "${output_file}"
  printf 'odom_count=%s\n' "$(line_field_value "${input_line}" odom || true)" >> "${output_file}"
  printf 'yolo_count=%s\n' "$(line_field_value "${input_line}" yolo || true)" >> "${output_file}"

  printf 'detection_tick_count=%s\n' "$(line_field_value "${stage_line}" detection || true)" >> "${output_file}"
  printf 'lidar_detection_tick_count=%s\n' "$(line_field_value "${stage_line}" lidar_detection || true)" >> "${output_file}"
  printf 'tracking_tick_count=%s\n' "$(line_field_value "${stage_line}" tracking || true)" >> "${output_file}"
  printf 'classification_tick_count=%s\n' "$(line_field_value "${stage_line}" classification || true)" >> "${output_file}"
  printf 'vis_tick_count=%s\n' "$(line_field_value "${stage_line}" vis || true)" >> "${output_file}"

  printf 'visual_bbox_count=%s\n' "$(line_field_value "${stats_line}" uv || true)" >> "${output_file}"
  printf 'db_bbox_count=%s\n' "$(line_field_value "${stats_line}" db || true)" >> "${output_file}"
  printf 'lidar_bbox_count=%s\n' "$(line_field_value "${stats_line}" lidar || true)" >> "${output_file}"
  printf 'filtered_bbox_count=%s\n' "$(line_field_value "${stats_line}" filtered || true)" >> "${output_file}"
  printf 'track_count=%s\n' "$(line_field_value "${stats_line}" tracks || true)" >> "${output_file}"
  printf 'dynamic_count=%s\n' "$(line_field_value "${stats_line}" dynamic || true)" >> "${output_file}"
  printf 'service_call_count=%s\n' "$(line_field_value "${stats_line}" service || true)" >> "${output_file}"
  printf 'u_map_box_count=%s\n' "$(line_field_value "${stats_line}" u_map || true)" >> "${output_file}"
  printf 'projected_depth_box_count=%s\n' "$(line_field_value "${stats_line}" depth_boxes || true)" >> "${output_file}"
  printf 'u_map_enhanced_db_count=%s\n' "$(line_field_value "${stats_line}" u_map_db || true)" >> "${output_file}"
  printf 'u_map_enhanced_visual_count=%s\n' "$(line_field_value "${stats_line}" u_map_visual || true)" >> "${output_file}"
  printf 'u_map_enhanced_filtered_before_yolo_count=%s\n' "$(line_field_value "${stats_line}" u_map_fused || true)" >> "${output_file}"
  printf 'u_map_enhanced_filtered_count=%s\n' "$(line_field_value "${stats_line}" u_map_filtered || true)" >> "${output_file}"
  return 0
}

capture_sample_snapshot() {
  local output_file="$1"
  if capture_log_snapshot "${output_file}"; then
    :
  elif "${STATUS_SNAPSHOT_CMD}" --timeout 8 > "${output_file}"; then
    :
  else
    : > "${output_file}"
  fi
}

sample_value() {
  local file="$1"
  local key="$2"
  local value
  value="$(sed -n "s/^${key}=//p" "${file}" | head -n1)"
  if [[ ! "${value}" =~ ^[0-9]+$ ]]; then
    value=0
  fi
  printf '%s' "${value}"
}

write_sample_delta() {
  local before_file="$1"
  local after_file="$2"
  local delta_file="$3"
  local keys=(
    depth_count color_count lidar_count pose_count odom_count yolo_count
    detection_tick_count lidar_detection_tick_count tracking_tick_count classification_tick_count vis_tick_count
    visual_bbox_count db_bbox_count lidar_bbox_count filtered_bbox_count track_count dynamic_count service_call_count
  )
  : > "${delta_file}"
  for key in "${keys[@]}"; do
    local before after delta
    before="$(sample_value "${before_file}" "${key}")"
    after="$(sample_value "${after_file}" "${key}")"
    delta=$((after - before))
    printf '%s_before=%s\n' "${key}" "${before}" >> "${delta_file}"
    printf '%s_after=%s\n' "${key}" "${after}" >> "${delta_file}"
    printf '%s_delta=%s\n' "${key}" "${delta}" >> "${delta_file}"
  done
}

write_summary() {
  cat > "${ARTIFACT_DIR}/summary.txt" <<EOF
validation=${VALIDATION_RESULT}
failure_reason=${FAILURE_REASON}
execution_status=${EXECUTION_STATUS}
artifact_dir=${ARTIFACT_DIR}
ready_timeout_seconds=${READY_TIMEOUT_SECONDS}
ready_poll_seconds=${READY_POLL_SECONDS}
ready_wait_seconds=${READY_WAIT_SECONDS}
ready_min_uptime_seconds=${READY_MIN_UPTIME_SECONDS}
ready_stable_uptime_seconds=${READY_STABLE_UPTIME_SECONDS}
post_ready_sample_seconds=${POST_READY_SAMPLE_SECONDS}
enable_yolo=${ENABLE_YOLO}
launch_yolo_node=${LAUNCH_YOLO_NODE}
use_all_classes=${USE_ALL_CLASSES}
enable_color_fallback=${ENABLE_COLOR_FALLBACK}
enable_vis_stage=${ENABLE_VIS_STAGE}
enable_vis_stage_applied=${ENABLE_VIS_STAGE_APPLIED}
executor_threads=${EXECUTOR_THREADS}
executor_threads_applied=${EXECUTOR_THREADS_APPLIED}
yolo_conf_threshold=${YOLO_CONF_THRESHOLD}
yolo_inference_hz=${YOLO_INFERENCE_HZ}
window_seconds=${WINDOW_SECONDS}
sample_hz=${SAMPLE_HZ}
hit_min_count=${HIT_MIN_COUNT}
count_tolerance=${COUNT_TOLERANCE}
service_query_range=${SERVICE_QUERY_RANGE}
window_dynamic_count=${WINDOW_DYNAMIC_COUNT}
window_service_position_count=${WINDOW_SERVICE_POSITION_COUNT}
window_abs_diff=${WINDOW_ABS_DIFF}
window_total_samples=${WINDOW_TOTAL_SAMPLES}
window_hit_count=${WINDOW_HIT_COUNT}
preflight=${ARTIFACT_DIR}/preflight.txt
launch_log=${ARTIFACT_DIR}/launch.log
ready_probe=${ARTIFACT_DIR}/ready_probe.txt
sample_before=${ARTIFACT_DIR}/sample_before.txt
sample_after=${ARTIFACT_DIR}/sample_after.txt
sample_delta=${ARTIFACT_DIR}/sample_delta.txt
check_topics=${ARTIFACT_DIR}/check_topics.txt
validate=${ARTIFACT_DIR}/validate.txt
validate_failure_keys=${VALIDATE_FAILURE_KEYS:-unknown}
current_status_snapshot=${ARTIFACT_DIR}/current_status_snapshot.txt
input_health_status=${ARTIFACT_DIR}/input_health_status.yaml
stage_timers_status=${ARTIFACT_DIR}/stage_timers_status.yaml
pipeline_stats_status=${ARTIFACT_DIR}/pipeline_stats_status.yaml
service_response=${ARTIFACT_DIR}/service_response.txt
window_series=${ARTIFACT_DIR}/window_series.csv
EOF
}

fail_with_status() {
  local execution_status="$1"
  local failure_reason="$2"
  local ready_result="$3"
  EXECUTION_STATUS="${execution_status}"
  FAILURE_REASON="${failure_reason}"
  VALIDATION_RESULT="FAIL"
  printf 'ready_result=%s\nready_wait_seconds=%s\n' "${ready_result}" "${READY_WAIT_SECONDS}" > "${ARTIFACT_DIR}/ready_probe.txt"
  write_summary
  exit 1
}

ensure_launch_alive_or_fail() {
  local phase="$1"
  if [[ -z "${LAUNCH_PID:-}" ]]; then
    return
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[suite] launch exited unexpectedly during ${phase}; see ${ARTIFACT_DIR}/launch.log"
    EXECUTION_STATUS="runtime_crash"
    FAILURE_REASON="runtime_crash"
    VALIDATION_RESULT="FAIL"
    write_summary
    exit 1
  fi
}

service_position_count_from_response() {
  local response_file="$1"
  local position_segment

  if [[ ! -f "${response_file}" ]]; then
    printf '0'
    return
  fi

  if ! grep -q 'position=' "${response_file}"; then
    printf '0'
    return
  fi

  if grep -q 'position=\[\]' "${response_file}"; then
    printf '0'
    return
  fi

  position_segment="$(sed -n 's/.*position=\[\(.*\)\], velocity=.*/\1/p' "${response_file}" | head -n1)"
  if [[ -z "${position_segment}" ]]; then
    printf '0'
    return
  fi

  grep -o 'geometry_msgs.msg.Vector3(' <<< "${position_segment}" | wc -l | tr -d '[:space:]'
}

# Read UAV current XYZ from /mavros/local_position/pose (one message, 3s timeout).
# Falls back to 0.0 0.0 0.0 if the topic is unavailable.
get_uav_position() {
  local raw
  raw="$(timeout 3 ros2 topic echo --once /mavros/local_position/pose 2>/dev/null \
         | grep -A3 'position:' | head -4 || true)"
  UAV_X="$(awk '/^    x:/{print $2; exit}' <<< "${raw}")"
  UAV_Y="$(awk '/^    y:/{print $2; exit}' <<< "${raw}")"
  UAV_Z="$(awk '/^    z:/{print $2; exit}' <<< "${raw}")"
  UAV_X="${UAV_X:-0.0}"
  UAV_Y="${UAV_Y:-0.0}"
  UAV_Z="${UAV_Z:-0.0}"
}

collect_window_series() {
  local output_csv="$1"
  local total_samples=0
  local hit_count=0
  local sample_interval
  local i ts dynamic_count service_count abs_diff hit
  local loop_count
  local snapshot_file response_file

  if ! [[ "${WINDOW_SECONDS}" =~ ^[0-9]+$ ]] || (( WINDOW_SECONDS <= 0 )); then
    WINDOW_SECONDS=30
  fi
  if ! [[ "${SAMPLE_HZ}" =~ ^[0-9]+$ ]] || (( SAMPLE_HZ <= 0 )); then
    SAMPLE_HZ=1
  fi
  if ! [[ "${COUNT_TOLERANCE}" =~ ^[0-9]+$ ]]; then
    COUNT_TOLERANCE=0
  fi

  loop_count=$(( WINDOW_SECONDS * SAMPLE_HZ ))
  if (( loop_count <= 0 )); then
    loop_count=1
  fi

  sample_interval="$(awk -v hz="${SAMPLE_HZ}" 'BEGIN { printf "%.6f", 1.0 / hz }')"
  snapshot_file="${ARTIFACT_DIR}/window_sample_snapshot.txt"
  response_file="${ARTIFACT_DIR}/window_sample_service_response.txt"

  printf 'ts,dynamic_count,service_position_count,abs_diff,hit\n' > "${output_csv}"
  for ((i = 1; i <= loop_count; ++i)); do
    ts="$(date +%s)"
    capture_sample_snapshot "${snapshot_file}"
    dynamic_count="$(sample_value "${snapshot_file}" "dynamic_count")"
    get_uav_position
    timeout 15 ros2 service call /onboard_detector/get_dynamic_obstacles \
      lvdot_interfaces/srv/GetDynamicObstacles \
      "{current_position: {x: ${UAV_X}, y: ${UAV_Y}, z: ${UAV_Z}}, range: ${SERVICE_QUERY_RANGE}}" \
      > "${response_file}" 2>&1 || true
    service_count="$(service_position_count_from_response "${response_file}")"
    abs_diff=$((dynamic_count - service_count))
    if (( abs_diff < 0 )); then
      abs_diff=$(( -abs_diff ))
    fi

    hit=0
    if (( service_count > 0 )); then
      hit=1
      hit_count=$((hit_count + 1))
    fi
    total_samples=$((total_samples + 1))
    printf '%s,%s,%s,%s,%s\n' "${ts}" "${dynamic_count}" "${service_count}" "${abs_diff}" "${hit}" >> "${output_csv}"
    sleep "${sample_interval}"
  done

  WINDOW_TOTAL_SAMPLES="${total_samples}"
  WINDOW_HIT_COUNT="${hit_count}"
}

verify_runtime_param() {
  local name="$1"
  local value="$2"
  local apply_flag_var="$3"
  local get_file current_value

  get_file="${ARTIFACT_DIR}/param_get_${name}.txt"
  if timeout 5 ros2 param get /lvdot_detector_node "${name}" > "${get_file}" 2>&1; then
    current_value="$(awk -F': ' 'NR==1{print $2}' "${get_file}" | tr -d '[:space:]')"
    if [[ "${current_value,,}" == "${value,,}" ]]; then
      printf -v "${apply_flag_var}" "true"
    else
      printf -v "${apply_flag_var}" "false"
    fi
  else
    printf -v "${apply_flag_var}" "false"
  fi
}

apply_runtime_stability_overrides() {
  verify_runtime_param "executor_threads" "${EXECUTOR_THREADS}" EXECUTOR_THREADS_APPLIED
  verify_runtime_param "enable_vis_stage" "${ENABLE_VIS_STAGE}" ENABLE_VIS_STAGE_APPLIED
}

classify_failure_reason() {
  local launch_log="${ARTIFACT_DIR}/launch.log"

  if [[ -f "${launch_log}" ]] && grep -Eq 'getifaddrs: Operation not permitted|TRANSPORT_UDP.*Operation not permitted|RTPS_PARTICIPANT.*failed to register' "${launch_log}"; then
    FAILURE_REASON="dds_environment_restricted"
  elif [[ -f "${launch_log}" ]] && grep -Eq 'process has died.*parameter_bridge|process has died.*gazebo-' "${launch_log}"; then
    FAILURE_REASON="scene_or_bridge_startup_failed"
  elif [[ -f "${ARTIFACT_DIR}/ready_probe.txt" ]] && grep -q '^ready_result=timeout$' "${ARTIFACT_DIR}/ready_probe.txt"; then
    FAILURE_REASON="ready_timeout"
  else
    FAILURE_REASON="validation_failed"
  fi
}

update_latest_link() {
  ln -sfn "${ARTIFACT_DIR}" "${ARTIFACT_ROOT}/latest_real_scene_regression"
}

is_environment_restricted() {
  local launch_log="${ARTIFACT_DIR}/launch.log"
  [[ -f "${launch_log}" ]] && grep -Eq 'getifaddrs: Operation not permitted|TRANSPORT_UDP.*Operation not permitted|RTPS_PARTICIPANT.*failed to register' "${launch_log}"
}

positive_integer_field() {
  local snapshot_file="$1"
  local key="$2"
  local value

  value="$(sample_value "${snapshot_file}" "${key}")"
  if [[ ! "${value}" =~ ^[0-9]+$ ]]; then
    return 1
  fi

  (( value > 0 ))
}

detector_ready() {
  local ready_probe_file="${ARTIFACT_DIR}/ready_probe.txt"
  capture_sample_snapshot "${ready_probe_file}"
  positive_integer_field "${ready_probe_file}" depth_count || return 1
  positive_integer_field "${ready_probe_file}" color_count || return 1
  positive_integer_field "${ready_probe_file}" lidar_count || return 1
  positive_integer_field "${ready_probe_file}" pose_count || return 1
  positive_integer_field "${ready_probe_file}" odom_count || return 1
  positive_integer_field "${ready_probe_file}" detection_tick_count || return 1
  positive_integer_field "${ready_probe_file}" lidar_detection_tick_count || return 1
  positive_integer_field "${ready_probe_file}" tracking_tick_count || return 1
  positive_integer_field "${ready_probe_file}" classification_tick_count || return 1
  if [[ "${ENABLE_VIS_STAGE,,}" == "true" ]]; then
    positive_integer_field "${ready_probe_file}" vis_tick_count || return 1
  fi
  return 0
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

update_latest_link

echo "[suite] artifact_dir=${ARTIFACT_DIR}"
echo "[suite] running preflight"
if /home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/preflight_real_scene_environment.sh \
  > "${ARTIFACT_DIR}/preflight.txt" 2>&1; then
  :
else
  echo "[suite] preflight failed"
  if grep -Eq 'gazebo_smoke_test: runtime environment blocks Gazebo/DDS startup|ros2_cli_runtime: runtime environment blocks DDS/network setup' "${ARTIFACT_DIR}/preflight.txt"; then
    EXECUTION_STATUS="preflight_fail"
    FAILURE_REASON="dds_environment_restricted"
  else
    EXECUTION_STATUS="preflight_fail"
    FAILURE_REASON="preflight_failed"
  fi
  VALIDATION_RESULT="FAIL"
  write_summary
  exit 1
fi

echo "[suite] launching scene + detector"
ros2 launch lvdot_bringup run_detector_with_scene.launch.py \
  gazebo_gui:=false \
  rviz:=false \
  detector_rviz:=false \
  enable_stage_timers:=true \
  enable_vis_stage:=${ENABLE_VIS_STAGE} \
  executor_threads:=${EXECUTOR_THREADS} \
  enable_yolo:=${ENABLE_YOLO} \
  launch_yolo_node:=${LAUNCH_YOLO_NODE} \
  use_all_classes:=${USE_ALL_CLASSES} \
  enable_color_fallback:=${ENABLE_COLOR_FALLBACK} \
  conf_threshold:=${YOLO_CONF_THRESHOLD} \
  inference_hz:=${YOLO_INFERENCE_HZ} \
  > "${ARTIFACT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[suite] waiting for detector readiness (timeout=${READY_TIMEOUT_SECONDS}s, poll=${READY_POLL_SECONDS}s, min_uptime=${READY_MIN_UPTIME_SECONDS}s)"
READY_RESULT="timeout"
READY_STABLE_SINCE=-1
while (( READY_WAIT_SECONDS < READY_TIMEOUT_SECONDS )); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[suite] launch exited before detector became ready; see ${ARTIFACT_DIR}/launch.log"
    classify_failure_reason
    if [[ "${FAILURE_REASON}" == "unknown" || "${FAILURE_REASON}" == "validation_failed" ]]; then
      FAILURE_REASON="launch_exited_early"
    fi
    EXECUTION_STATUS="launch_fail"
    fail_with_status "launch_fail" "${FAILURE_REASON}" "launch_exited_early"
  fi

  if is_environment_restricted; then
    echo "[suite] environment restriction detected from launch log"
    fail_with_status "launch_fail" "dds_environment_restricted" "dds_environment_restricted"
  fi

  if detector_ready; then
    local_now="$(date +%s)"
    if (( READY_STABLE_SINCE < 0 )); then
      READY_STABLE_SINCE="${local_now}"
    fi
    READY_STABLE_UPTIME_SECONDS=$((local_now - READY_STABLE_SINCE))
    if (( READY_STABLE_UPTIME_SECONDS >= READY_MIN_UPTIME_SECONDS )); then
      READY_RESULT="ready"
      break
    fi
  else
    READY_STABLE_SINCE=-1
    READY_STABLE_UPTIME_SECONDS=0
  fi

  sleep "${READY_POLL_SECONDS}"
  READY_WAIT_SECONDS=$((READY_WAIT_SECONDS + READY_POLL_SECONDS))
done

printf 'ready_result=%s\nready_wait_seconds=%s\nready_min_uptime_seconds=%s\nready_stable_uptime_seconds=%s\nenable_yolo=%s\nlaunch_yolo_node=%s\nuse_all_classes=%s\nenable_color_fallback=%s\nenable_vis_stage=%s\nexecutor_threads=%s\nyolo_conf_threshold=%s\nyolo_inference_hz=%s\n' \
  "${READY_RESULT}" "${READY_WAIT_SECONDS}" "${READY_MIN_UPTIME_SECONDS}" "${READY_STABLE_UPTIME_SECONDS}" "${ENABLE_YOLO}" "${LAUNCH_YOLO_NODE}" "${USE_ALL_CLASSES}" "${ENABLE_COLOR_FALLBACK}" "${ENABLE_VIS_STAGE}" "${EXECUTOR_THREADS}" "${YOLO_CONF_THRESHOLD}" "${YOLO_INFERENCE_HZ}" > "${ARTIFACT_DIR}/ready_probe.txt"

if [[ "${READY_RESULT}" != "ready" ]]; then
  echo "[suite] detector did not become ready within ${READY_TIMEOUT_SECONDS}s"
  classify_failure_reason
  if [[ "${FAILURE_REASON}" == "unknown" || "${FAILURE_REASON}" == "validation_failed" ]]; then
    FAILURE_REASON="ready_timeout"
  fi
  fail_with_status "launch_fail" "${FAILURE_REASON}" "${READY_RESULT}"
fi

echo "[suite] detector ready after ${READY_WAIT_SECONDS}s"
ensure_launch_alive_or_fail "post_ready"
echo "[suite] applying runtime stability overrides"
apply_runtime_stability_overrides
{
  printf 'executor_threads_applied=%s\n' "${EXECUTOR_THREADS_APPLIED}"
  printf 'enable_vis_stage_applied=%s\n' "${ENABLE_VIS_STAGE_APPLIED}"
} >> "${ARTIFACT_DIR}/ready_probe.txt"
echo "[suite] capturing baseline counters"
capture_sample_snapshot "${ARTIFACT_DIR}/sample_before.txt"
echo "[suite] collecting window series (window=${WINDOW_SECONDS}s, hz=${SAMPLE_HZ})"
collect_window_series "${ARTIFACT_DIR}/window_series.csv"
ensure_launch_alive_or_fail "window_collection"
echo "[suite] sampling for ${POST_READY_SAMPLE_SECONDS}s"
sleep "${POST_READY_SAMPLE_SECONDS}"
ensure_launch_alive_or_fail "post_ready_sampling"
echo "[suite] capturing post-sample counters"
capture_sample_snapshot "${ARTIFACT_DIR}/sample_after.txt"
write_sample_delta \
  "${ARTIFACT_DIR}/sample_before.txt" \
  "${ARTIFACT_DIR}/sample_after.txt" \
  "${ARTIFACT_DIR}/sample_delta.txt"

echo "[suite] checking topics"
/home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/check_real_scene_topics.sh \
  > "${ARTIFACT_DIR}/check_topics.txt" 2>&1 || true

echo "[suite] capturing service response"
get_uav_position
timeout 15 ros2 service call /onboard_detector/get_dynamic_obstacles \
  lvdot_interfaces/srv/GetDynamicObstacles \
  "{current_position: {x: ${UAV_X}, y: ${UAV_Y}, z: ${UAV_Z}}, range: ${SERVICE_QUERY_RANGE}}" \
  > "${ARTIFACT_DIR}/service_response.txt" 2>&1 || true

echo "[suite] validating regression"
cp "${ARTIFACT_DIR}/sample_after.txt" "${ARTIFACT_DIR}/current_status_snapshot.txt"
WINDOW_DYNAMIC_COUNT="$(sample_value "${ARTIFACT_DIR}/sample_after.txt" "dynamic_count")"
WINDOW_SERVICE_POSITION_COUNT="$(service_position_count_from_response "${ARTIFACT_DIR}/service_response.txt")"
WINDOW_ABS_DIFF=$((WINDOW_DYNAMIC_COUNT - WINDOW_SERVICE_POSITION_COUNT))
if (( WINDOW_ABS_DIFF < 0 )); then
  WINDOW_ABS_DIFF=$(( -WINDOW_ABS_DIFF ))
fi
{
  printf 'service_position_count=%s\n' "${WINDOW_SERVICE_POSITION_COUNT}"
  printf 'window_dynamic_count=%s\n' "${WINDOW_DYNAMIC_COUNT}"
  printf 'window_service_position_count=%s\n' "${WINDOW_SERVICE_POSITION_COUNT}"
  printf 'window_abs_diff=%s\n' "${WINDOW_ABS_DIFF}"
  printf 'window_total_samples=%s\n' "${WINDOW_TOTAL_SAMPLES}"
  printf 'window_hit_count=%s\n' "${WINDOW_HIT_COUNT}"
  printf 'hit_min_count=%s\n' "${HIT_MIN_COUNT}"
  printf 'count_tolerance=%s\n' "${COUNT_TOLERANCE}"
} >> "${ARTIFACT_DIR}/current_status_snapshot.txt"
if REQUIRE_YOLO_INPUT="${ENABLE_YOLO}" \
  CURRENT_SNAPSHOT_FILE="${ARTIFACT_DIR}/current_status_snapshot.txt" \
  SAMPLE_DELTA_FILE="${ARTIFACT_DIR}/sample_delta.txt" \
  SERVICE_RESPONSE_FILE="${ARTIFACT_DIR}/service_response.txt" \
  HIT_MIN_COUNT="${HIT_MIN_COUNT}" \
  /home/skbt2/lvdot_ros2_ws/install/lvdot_bringup/lib/lvdot_bringup/validate_real_scene_regression.sh \
  > "${ARTIFACT_DIR}/validate.txt" 2>&1; then
  VALIDATION_RESULT="PASS"
else
  VALIDATION_RESULT="FAIL"
fi
VALIDATE_FAILURE_KEYS="$(sed -n 's/^failure_keys=//p' "${ARTIFACT_DIR}/validate.txt" | tail -n1)"
if [[ -z "${VALIDATE_FAILURE_KEYS}" ]]; then
  VALIDATE_FAILURE_KEYS="unknown"
fi

echo "[suite] capturing structured status"
"${STATUS_SNAPSHOT_CMD}" --timeout 8 > "${ARTIFACT_DIR}/current_status_snapshot.txt" 2>/dev/null || true
{
  printf 'service_position_count=%s\n' "${WINDOW_SERVICE_POSITION_COUNT}"
  printf 'window_dynamic_count=%s\n' "${WINDOW_DYNAMIC_COUNT}"
  printf 'window_service_position_count=%s\n' "${WINDOW_SERVICE_POSITION_COUNT}"
  printf 'window_abs_diff=%s\n' "${WINDOW_ABS_DIFF}"
  printf 'window_total_samples=%s\n' "${WINDOW_TOTAL_SAMPLES}"
  printf 'window_hit_count=%s\n' "${WINDOW_HIT_COUNT}"
  printf 'hit_min_count=%s\n' "${HIT_MIN_COUNT}"
  printf 'count_tolerance=%s\n' "${COUNT_TOLERANCE}"
} >> "${ARTIFACT_DIR}/current_status_snapshot.txt"
timeout 8 ros2 topic echo /onboard_detector/input_health_status --once > "${ARTIFACT_DIR}/input_health_status.yaml" 2>&1 || true
timeout 8 ros2 topic echo /onboard_detector/stage_timers_status --once > "${ARTIFACT_DIR}/stage_timers_status.yaml" 2>&1 || true
timeout 8 ros2 topic echo /onboard_detector/pipeline_stats_status --once > "${ARTIFACT_DIR}/pipeline_stats_status.yaml" 2>&1 || true

echo "[suite] writing summary"
if [[ "${VALIDATION_RESULT}" == "PASS" ]]; then
  EXECUTION_STATUS="completed"
  FAILURE_REASON="none"
else
  if [[ "${EXECUTION_STATUS}" == "unknown" ]]; then
    EXECUTION_STATUS="completed"
  fi
  if [[ "${FAILURE_REASON}" == "unknown" || "${FAILURE_REASON}" == "validation_failed" ]]; then
    classify_failure_reason
  fi
fi
write_summary

echo "[suite] validation=${VALIDATION_RESULT}"
echo "[suite] summary=${ARTIFACT_DIR}/summary.txt"

if [[ "${VALIDATION_RESULT}" != "PASS" ]]; then
  exit 1
fi
