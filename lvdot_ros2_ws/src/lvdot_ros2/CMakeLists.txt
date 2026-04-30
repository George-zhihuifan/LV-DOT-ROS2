cmake_minimum_required(VERSION 3.10)
project(lvdot_ros2)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lvdot_core REQUIRED)
find_package(lvdot_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(lvdot_detector_main
  src/lvdot_detector_main.cpp
  src/lvdot_detector_node.cpp
  src/lvdot_runtime_bridge.cpp
  src/lvdot_visualization_bridge.cpp
  src/ros2_sync_context.cpp
)

target_include_directories(lvdot_detector_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lvdot_detector_main
  cv_bridge
  geometry_msgs
  lvdot_core
  lvdot_interfaces
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  vision_msgs
)

target_link_libraries(lvdot_detector_main
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Export symbols so backtrace_symbols_fd can resolve function names in the
# SIGSEGV handler installed in lvdot_detector_main.cpp.
target_link_options(lvdot_detector_main PRIVATE -rdynamic)

install(TARGETS lvdot_detector_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
