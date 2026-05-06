#pragma once

#include "lvdot_core/classification_filter.hpp"
#include "lvdot_core/detection_filter.hpp"
#include "lvdot_core/fusion_filter.hpp"
#include "lvdot_core/tracking_filter.hpp"
#include "lvdot_ros2/lvdot_detector_config.hpp"
#include "lvdot_ros2/lvdot_runtime_state.hpp"

namespace lvdot_ros2
{

onboardDetector::DetectionInput build_detection_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config);

void apply_detection_output(
  const onboardDetector::DetectionOutput & output,
  LVdotRuntimeState & state);

onboardDetector::FilterLVBBoxesInput build_filter_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config);

void apply_filter_output(
  const onboardDetector::FilterLVBBoxesOutput & output,
  LVdotRuntimeState & state);

onboardDetector::TrackingInput build_tracking_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config);

void apply_tracking_output(
  const onboardDetector::TrackingOutput & output,
  LVdotRuntimeState & state);

onboardDetector::ClassificationInput build_classification_input(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config);

void apply_classification_output(
  const onboardDetector::ClassificationOutput & output,
  LVdotRuntimeState & state,
  std::size_t & dynamic_rejected_by_size_out);

}  // namespace lvdot_ros2

