cmake_minimum_required(VERSION 3.10)
project(depth_eval_ped_gz_plugin)

find_package(ament_cmake REQUIRED)
find_package(depth_eval_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

add_library(pedestrian_pose_sync_system SHARED
  src/pedestrian_pose_sync_system.cpp
)

target_compile_features(pedestrian_pose_sync_system PUBLIC cxx_std_17)

target_link_libraries(pedestrian_pose_sync_system
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

ament_target_dependencies(pedestrian_pose_sync_system
  depth_eval_msgs
  rclcpp
  geometry_msgs
)

add_library(uav_pose_sync_system SHARED
  src/uav_pose_sync_system.cpp
)
target_compile_features(uav_pose_sync_system PUBLIC cxx_std_17)
target_link_libraries(uav_pose_sync_system
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)
ament_target_dependencies(uav_pose_sync_system
  rclcpp
  geometry_msgs
)

install(
  TARGETS pedestrian_pose_sync_system uav_pose_sync_system
  LIBRARY DESTINATION lib
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
