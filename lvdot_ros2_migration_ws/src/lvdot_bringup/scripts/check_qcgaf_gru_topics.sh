#!/usr/bin/env bash
set -euo pipefail

WAIT_SEC="${1:-20}"

required=(
  /onboard_detector/visual_bboxes_qcgaf
  /onboard_detector/lidar_bboxes_qcgaf
  /onboard_detector/dynamic_bboxes
  /qcgaf/fused_bboxes
  /gru_predictor/predicted_positions
)

wait_for_topics() {
  local timeout="$1"
  local start_ts now_ts
  start_ts="$(date +%s)"
  while true; do
    local topics
    topics="$(ros2 topic list || true)"
    local all_exist=1
    for t in "${required[@]}"; do
      if ! grep -qx "$t" <<< "$topics"; then
        all_exist=0
        break
      fi
    done
    if [[ "$all_exist" -eq 1 ]]; then
      return 0
    fi
    now_ts="$(date +%s)"
    if (( now_ts - start_ts >= timeout )); then
      return 1
    fi
    sleep 1
  done
}

wait_for_topics "$WAIT_SEC" || true
topics="$(ros2 topic list || true)"
missing=0

for t in "${required[@]}"; do
  if ! grep -qx "$t" <<< "$topics"; then
    echo "[MISS] $t (topic missing)"
    missing=1
    continue
  fi

  info="$(ros2 topic info "$t" 2>/dev/null || true)"
  pub_count="$(awk -F': ' '/Publisher count/ {print $2}' <<< "$info" | head -n1)"
  if [[ -z "$pub_count" ]]; then
    pub_count=0
  fi

  if (( pub_count < 1 )); then
    echo "[MISS] $t (publisher_count=$pub_count)"
    missing=1
  else
    echo "[OK] $t (publisher_count=$pub_count)"
  fi
done

if [[ $missing -ne 0 ]]; then
  echo "Topic check failed"
  exit 1
fi

echo "All required topics exist with publishers"
