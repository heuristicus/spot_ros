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
  LeaseArray.msg
  Lease.msg
  LeaseOwner.msg
  LeaseResource.msg
  Metrics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
