cmake_minimum_required(VERSION 3.0.2)
project(spot_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BatteryStateArray.msg
  BehaviorFault.msg
  EStopStateArray.msg
  FootStateArray.msg
  LeaseArray.msg
  LeaseOwner.msg
  Metrics.msg
  SystemFault.msg
  WiFiState.msg
  BatteryState.msg
  BehaviorFaultState.msg
  EStopState.msg
  Feedback.msg
  FootState.msg
  Lease.msg
  LeaseResource.msg
  PowerState.msg
  SystemFaultState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
