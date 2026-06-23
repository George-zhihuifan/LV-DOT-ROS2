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

// Path X (replacement mode): build tracking input using QC-GAF boxes as the
// observation list.  Clusters are emitted as empty arrays — the tracker
// degrades to box-only operation.  Returns an empty filteredBBoxes list if
// state.qcgaf_filtered_bboxes is empty (caller should fall back to baseline).
onboardDetector::TrackingInput build_tracking_input_qcgaf_replacement(
  const LVdotRuntimeState & state,
  const LVdotDetectorConfig & config);

// Path Z (refinement mode): for each box in state.filtered_bboxes, find the
// nearest QC-GAF box within max_match_distance and overwrite its geometry
// (x, y, z, x_width, y_width, z_width).  Clusters are not touched so the
// 1:1 correspondence with filtered_bboxes is preserved.  Boxes with no QC-GAF
// match within range keep their rule-fusion geometry.
void apply_qcgaf_geometry_refinement(
  LVdotRuntimeState & state,
  double max_match_distance);

void apply_depth_auxiliary_correction(
  LVdotRuntimeState & state,
  double max_match_distance);

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
