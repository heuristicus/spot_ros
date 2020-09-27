cmake_minimum_required(VERSION 3.5)
project(spot_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStateArray.msg"
  "msg/BehaviorFault.msg"
  "msg/EStopStateArray.msg"
  "msg/FootStateArray.msg"
  "msg/LeaseArray.msg"
  "msg/LeaseOwner.msg"
  "msg/Metrics.msg"
  "msg/SystemFault.msg"
  "msg/WiFiState.msg"
  "msg/BatteryState.msg"
  "msg/BehaviorFaultState.msg"
  "msg/EStopState.msg"
  "msg/Feedback.msg"
  "msg/FootState.msg"
  "msg/Lease.msg"
  "msg/LeaseResource.msg"
  "msg/PowerState.msg"
  "msg/SystemFaultState.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
