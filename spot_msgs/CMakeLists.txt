cmake_minimum_required(VERSION 3.0.2)
project(spot_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  actionlib
  actionlib_msgs
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
  MobilityParams.msg
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

add_service_files(
  FILES
  ListGraph.srv
  SetLocomotion.srv
  SetVelocity.srv
  ClearBehaviorFault.srv
)

add_action_files(
  FILES
  NavigateTo.action
  Trajectory.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
