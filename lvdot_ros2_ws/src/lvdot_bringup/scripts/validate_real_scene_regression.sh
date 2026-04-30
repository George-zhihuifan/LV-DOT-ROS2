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

REQUIRE_YOLO_INPUT="${REQUIRE_YOLO_INPUT:-false}"
SAMPLE_DELTA_FILE="${SAMPLE_DELTA_FILE:-}"
LAUNCH_LOG_FILE="${LAUNCH_LOG_FILE:-}"
CURRENT_SNAPSHOT_FILE="${CURRENT_SNAPSHOT_FILE:-}"
SERVICE_RESPONSE_FILE="${SERVICE_RESPONSE_FILE:-}"
HIT_MIN_COUNT="${HIT_MIN_COUNT:-5}"
fail=0
failure_keys=()

log_field_value() {
  local section="$1"
  local key="$2"
  local line
  if [[ -z "${LAUNCH_LOG_FILE}" || ! -f "${LAUNCH_LOG_FILE}" ]]; then
    return
  fi
  line="$(grep "${section}:" "${LAUNCH_LOG_FILE}" | tail -n1 || true)"
  if [[ -z "${line}" ]]; then
    return
  fi
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

snapshot_value() {
  local key="$1"
  local value
  if [[ -z "${CURRENT_SNAPSHOT_FILE}" || ! -f "${CURRENT_SNAPSHOT_FILE}" ]]; then
    return
  fi
  value="$(sed -n "s/^${key}=//p" "${CURRENT_SNAPSHOT_FILE}" | head -n1)"
  if [[ "${value}" =~ ^[0-9]+$ ]]; then
    printf '%s' "${value}"
  fi
}

snapshot_or_log_value() {
  local snapshot_key="$1"
  local section="$2"
  local key="$3"
  local value

  value="$(snapshot_value "${snapshot_key}")"
  if [[ "${value}" =~ ^[0-9]+$ ]]; then
    printf '%s' "${value}"
    return
  fi

  value="$(log_field_value "${section}" "${key}")"
  if [[ "$value" =~ ^[0-9]+$ ]]; then
    printf '%s' "$value"
    return
  fi

  printf ''
}

check_positive_field() {
  local snapshot_key="$1"
  local label="$2"
  local section="$3"
  local key="$4"
  local value

  value="$(snapshot_or_log_value "$snapshot_key" "$section" "$key")"
  if [[ -z "$value" ]]; then
    echo "[FAIL] $label: no message"
    fail=1
    failure_keys+=("${label// /_}_missing")
    return
  fi

  if ! [[ "$value" =~ ^[0-9]+$ ]]; then
    echo "[FAIL] $label: non-integer value '$value'"
    fail=1
    failure_keys+=("${label// /_}_non_integer")
    return
  fi

  if (( value > 0 )); then
    echo "[PASS] $label: $value"
  else
    echo "[FAIL] $label: $value"
    fail=1
    failure_keys+=("${label// /_}_zero")
  fi
}

check_nonnegative_field() {
  local snapshot_key="$1"
  local label="$2"
  local section="$3"
  local key="$4"
  local value

  value="$(snapshot_or_log_value "$snapshot_key" "$section" "$key")"
  if [[ -z "$value" ]]; then
    echo "[FAIL] $label: no message"
    fail=1
    failure_keys+=("${label// /_}_missing")
    return
  fi

  if ! [[ "$value" =~ ^[0-9]+$ ]]; then
    echo "[FAIL] $label: non-integer value '$value'"
    fail=1
    failure_keys+=("${label// /_}_non_integer")
    return
  fi

  echo "[PASS] $label: $value"
}

check_dynamic_positive_field() {
  local snapshot_key="$1"
  local label="$2"
  local section="$3"
  local key="$4"
  local value

  value="$(snapshot_or_log_value "$snapshot_key" "$section" "$key")"
  if [[ -z "$value" ]]; then
    echo "[FAIL] $label: no message"
    fail=1
    failure_keys+=("${label// /_}_missing")
    return
  fi

  if ! [[ "$value" =~ ^[0-9]+$ ]]; then
    echo "[FAIL] $label: non-integer value '$value'"
    fail=1
    failure_keys+=("${label// /_}_non_integer_value_${value}")
    return
  fi

  if (( value > 0 )); then
    echo "[PASS] $label: $value"
  else
    echo "[FAIL] $label: $value (expected > 0)"
    fail=1
    failure_keys+=("${label// /_}_zero_value_${value}")
  fi
}

check_service_position_nonempty() {
  if [[ -z "${SERVICE_RESPONSE_FILE}" || ! -f "${SERVICE_RESPONSE_FILE}" ]]; then
    echo "[FAIL] service response file: missing"
    fail=1
    failure_keys+=("service_response_file_missing")
    return
  fi

  if grep -Eq 'position=\[[^]]+\]' "${SERVICE_RESPONSE_FILE}"; then
    echo "[PASS] service_response.position: non-empty"
  else
    echo "[FAIL] service_response.position: empty_or_missing"
    fail=1
    failure_keys+=("service_response_position_empty")
  fi
}

count_value_or_fail() {
  local snapshot_key="$1"
  local section="$2"
  local key="$3"
  local failure_key="$4"
  local value

  if [[ -z "${section}" || -z "${key}" ]]; then
    value="$(snapshot_value "${snapshot_key}")"
  else
    value="$(snapshot_or_log_value "${snapshot_key}" "${section}" "${key}")"
  fi
  if [[ -z "${value}" ]]; then
    echo "[FAIL] ${snapshot_key}: no message"
    fail=1
    failure_keys+=("${failure_key}_missing")
    printf ''
    return
  fi
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[FAIL] ${snapshot_key}: non-integer value '${value}'"
    fail=1
    failure_keys+=("${failure_key}_non_integer")
    printf ''
    return
  fi
  printf '%s' "${value}"
}

check_window_hit_count() {
  local total_samples hit_count hit_min_count

  total_samples="$(count_value_or_fail "window_total_samples" "" "" "window_total_samples")"
  hit_count="$(count_value_or_fail "window_hit_count" "" "" "window_hit_count")"

  if ! [[ "${HIT_MIN_COUNT}" =~ ^[0-9]+$ ]]; then
    hit_min_count=8
  else
    hit_min_count="${HIT_MIN_COUNT}"
  fi

  if [[ -z "${total_samples}" || -z "${hit_count}" ]]; then
    return
  fi

  echo "total_samples=${total_samples}"
  echo "hit_count=${hit_count}"
  echo "hit_min_count=${hit_min_count}"

  if (( hit_count >= hit_min_count )); then
    echo "[PASS] window hit count: ${hit_count} >= ${hit_min_count}"
  else
    echo "[FAIL] window hit count: ${hit_count} < ${hit_min_count}"
    fail=1
    failure_keys+=("window_hit_count_insufficient")
  fi
}

delta_value() {
  local key="$1"
  if [[ -z "${SAMPLE_DELTA_FILE}" || ! -f "${SAMPLE_DELTA_FILE}" ]]; then
    echo ""
    return
  fi
  sed -n "s/^${key}_delta=//p" "${SAMPLE_DELTA_FILE}" | head -n1
}

check_positive_delta() {
  local key="$1"
  local label="$2"
  local value

  value="$(delta_value "${key}")"
  if [[ -z "${value}" ]]; then
    echo "[FAIL] ${label} delta: missing"
    fail=1
    failure_keys+=("${label// /_}_delta_missing")
    return
  fi
  if ! [[ "${value}" =~ ^-?[0-9]+$ ]]; then
    echo "[FAIL] ${label} delta: non-integer value '${value}'"
    fail=1
    failure_keys+=("${label// /_}_delta_non_integer")
    return
  fi
  if (( value > 0 )); then
    echo "[PASS] ${label} delta: ${value}"
  else
    echo "[FAIL] ${label} delta: ${value}"
    fail=1
    failure_keys+=("${label// /_}_delta_non_positive")
  fi
}

check_nonnegative_delta() {
  local key="$1"
  local label="$2"
  local value

  value="$(delta_value "${key}")"
  if [[ -z "${value}" ]]; then
    echo "[FAIL] ${label} delta: missing"
    fail=1
    failure_keys+=("${label// /_}_delta_missing")
    return
  fi
  if ! [[ "${value}" =~ ^-?[0-9]+$ ]]; then
    echo "[FAIL] ${label} delta: non-integer value '${value}'"
    fail=1
    failure_keys+=("${label// /_}_delta_non_integer")
    return
  fi
  if (( value >= 0 )); then
    echo "[PASS] ${label} delta: ${value}"
  else
    echo "[FAIL] ${label} delta: ${value}"
    fail=1
    failure_keys+=("${label// /_}_delta_negative")
  fi
}

check_yolo_input_if_required() {
  if [[ "${REQUIRE_YOLO_INPUT}" != "true" ]]; then
    return
  fi
  local value
  value="$(snapshot_or_log_value "yolo_count" "" "")"
  if [[ -z "${value}" ]]; then
    echo "[FAIL] yolo_count: no message (REQUIRE_YOLO_INPUT=true)"
    fail=1
    failure_keys+=("yolo_count_missing")
    return
  fi
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[FAIL] yolo_count: non-integer value '${value}'"
    fail=1
    failure_keys+=("yolo_count_non_integer")
    return
  fi
  if (( value > 0 )); then
    echo "[PASS] yolo_count: ${value}"
  else
    echo "[FAIL] yolo_count: ${value} (REQUIRE_YOLO_INPUT=true requires > 0)"
    fail=1
    failure_keys+=("yolo_count_zero")
  fi
}

echo "=== Regression Validation ==="
check_window_hit_count
check_service_position_nonempty
check_yolo_input_if_required

if (( ${#failure_keys[@]} > 0 )); then
  echo "failure_keys=$(IFS=,; echo "${failure_keys[*]}")"
else
  echo "failure_keys=none"
fi

if (( fail == 0 )); then
  echo "=== RESULT: PASS ==="
else
  echo "=== RESULT: FAIL ==="
  exit 1
fi
