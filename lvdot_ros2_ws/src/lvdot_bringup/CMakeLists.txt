cmake_minimum_required(VERSION 3.10)
project(lvdot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/capture_status_snapshot.py
  scripts/preflight_real_scene_environment.sh
  scripts/smoke_test_detector_only.sh
  scripts/run_detector_only_regression_suite.sh
  scripts/run_regression_matrix.sh
  scripts/run_scene_environment_diagnostics.sh
  scripts/run_scene_staged_diagnostics.sh
  scripts/show_latest_regression_report.sh
  scripts/show_latest_scene_diagnostics_report.sh
  scripts/show_latest_scene_staged_report.sh
  scripts/run_real_scene_regression.sh
  scripts/run_real_scene_regression_suite.sh
  scripts/watch_pipeline_stats.sh
  scripts/check_real_scene_topics.sh
  scripts/validate_real_scene_regression.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
