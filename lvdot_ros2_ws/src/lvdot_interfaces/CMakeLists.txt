cmake_minimum_required(VERSION 3.10)
project(lvdot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InputHealth.msg"
  "msg/StageTimers.msg"
  "msg/PipelineStats.msg"
  "srv/GetDynamicObstacles.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
