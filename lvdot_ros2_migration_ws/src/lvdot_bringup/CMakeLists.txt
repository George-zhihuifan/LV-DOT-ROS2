cmake_minimum_required(VERSION 3.10)
project(lvdot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_ablation.sh
  scripts/run_depth_branch_offset_sweep.sh
  scripts/summarize_results.py
  scripts/u_map_sweep.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
